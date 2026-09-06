#!/usr/bin/env python3
"""
Test complet du robot remorque.
Lit les joints et l'odométrie PENDANT le mouvement (état stable).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import time, sys

JOINT_NAMES = ['rear_left_joint', 'rear_right_joint',
               'front_left_joint', 'front_right_joint',
               'hitch_joint']

class RobotTester(Node):
    def __init__(self):
        super().__init__('robot_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Direct cmd_vel (safety node disabled)
        self.joint_vel = {}
        self.odom_linear = 0.0
        self.odom_angular = 0.0
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        time.sleep(0.5)

    def _js_cb(self, msg):
        for name, vel in zip(msg.name, msg.velocity):
            self.joint_vel[name] = vel

    def _odom_cb(self, msg):
        self.odom_linear  = msg.twist.twist.linear.x
        self.odom_angular = msg.twist.twist.angular.z

    def send(self, lx, az, duration=2.5):
        """Envoie la commande, attend l'état stable, lit les mesures."""
        msg = Twist()
        msg.linear.x  = lx
        msg.angular.z = az
        t_end = time.time() + duration
        while time.time() < t_end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self):
        msg = Twist()
        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.5)

    def snapshot(self):
        """Lit 10 samples et fait la moyenne pour stabilité."""
        samples = {n: [] for n in JOINT_NAMES}
        lin, ang = [], []
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            for n in JOINT_NAMES:
                if n in self.joint_vel:
                    samples[n].append(self.joint_vel[n])
            lin.append(self.odom_linear)
            ang.append(self.odom_angular)
            time.sleep(0.05)
        avg = {n: (sum(v)/len(v) if v else 0.0) for n, v in samples.items()}
        return avg, sum(lin)/len(lin), sum(ang)/len(ang)

    def run_test(self, label, lx, az):
        SEP = "━" * 52
        print(f"\n{SEP}")
        print(f"  {label}")
        print(f"  cmd_vel → linear.x={lx:+.1f}  angular.z={az:+.1f}")
        print(SEP)

        # Envoie 1s pour démarrer, puis lit pendant 1.5s supplémentaires
        self.send(lx, az, duration=1.0)

        # Collecte des données PENDANT la commande active (timing corrigé)
        samples = {n: [] for n in JOINT_NAMES}
        lin, ang = [], []
        msg = Twist(); msg.linear.x = lx; msg.angular.z = az
        t_end = time.time() + 1.5
        while time.time() < t_end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            # Collecte les données pendant que la commande est active
            for n in JOINT_NAMES:
                if n in self.joint_vel:
                    samples[n].append(self.joint_vel[n])
            lin.append(self.odom_linear)
            ang.append(self.odom_angular)
            time.sleep(0.05)

        # Calcule les moyennes
        avg = {n: (sum(v)/len(v) if v else 0.0) for n, v in samples.items()}
        lin_odom = sum(lin)/len(lin)
        ang_odom = sum(ang)/len(ang)

        # Affichage joints
        RL = avg.get('rear_left_joint',  0.0)
        RR = avg.get('rear_right_joint', 0.0)
        FL = avg.get('front_left_joint', 0.0)
        FR = avg.get('front_right_joint',0.0)
        HJ = avg.get('hitch_joint',      0.0)

        def bar(v, maxv=3.0, width=20):
            n = int(abs(v) / maxv * width)
            n = min(n, width)
            return ('◀' if v < 0 else '▶') + '█' * n + '░' * (width - n)

        print(f"  JOINTS (vitesse rad/s):")
        print(f"  rear_left_joint   {RL:+7.3f}  {bar(RL)}")
        print(f"  rear_right_joint  {RR:+7.3f}  {bar(RR)}")
        print(f"  front_left_joint  {FL:+7.3f}  {bar(FL)}")
        print(f"  front_right_joint {FR:+7.3f}  {bar(FR)}")
        print(f"  hitch_joint       {HJ:+7.4f}  (angle attelage)")
        print(f"  ODOMETRIE:")
        print(f"  linear.x   = {lin_odom:+.4f} m/s  (commandé: {lx:+.1f})")
        print(f"  angular.z  = {ang_odom:+.4f} rad/s  (commandé: {az:+.1f})")

        # Diagnostic
        issues = []
        if lx != 0 and abs(lin_odom) < 0.05:
            issues.append("⚠️  Vitesse linéaire quasi nulle")
        if az != 0 and abs(ang_odom) < 0.05:
            issues.append("⚠️  Pas de rotation détectée")
        if lx == 0 and az > 0 and RL >= 0:
            issues.append("⚠️  rear_left devrait être négatif pour tourner à gauche")
        if lx == 0 and az < 0 and RR >= 0:
            issues.append("⚠️  rear_right devrait être négatif pour tourner à droite")
        if not issues:
            print("  ✅ Comportement CORRECT")
        else:
            for iss in issues:
                print(f"  {iss}")

        self.stop()
        time.sleep(0.5)


def main():
    rclpy.init()
    node = RobotTester()

    print("\n" + "═"*52)
    print("  TESTS COMPLETS ROBOT REMORQUE")
    print("═"*52)

    node.run_test("↑  TEST 1 — AVANCE",          lx=+0.3, az=0.0)
    node.run_test("↓  TEST 2 — RECULE",           lx=-0.3, az=0.0)
    node.run_test("↺  TEST 3 — ROTATION GAUCHE",  lx=0.0,  az=+0.8)
    node.run_test("↻  TEST 4 — ROTATION DROITE",  lx=0.0,  az=-0.8)
    node.run_test("↙  TEST 5 — COURBE GAUCHE",    lx=+0.3, az=+0.5)
    node.run_test("↘  TEST 6 — COURBE DROITE",    lx=+0.3, az=-0.5)

    print("\n" + "═"*52)
    print("  ✅ TOUS LES TESTS TERMINÉS")
    print("═"*52 + "\n")

    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
