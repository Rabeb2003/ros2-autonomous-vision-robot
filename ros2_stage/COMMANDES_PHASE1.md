# Commandes phase 1

## 1. Compiler

```bash
cd /home/rabeb/ros2_diff_drive_robot/ros2_stage
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Lancer la simulation cinematique + RViz

```bash
cd /home/rabeb/ros2_diff_drive_robot/ros2_stage
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2_stage phase1.launch.py
```

Sans RViz:

```bash
ros2 launch ros2_stage phase1.launch.py rviz:=false
```

## 3. Envoyer des commandes cmd_vel

Important: `ros2 topic pub` publie en boucle par defaut. C'est normal de voir:

```text
publishing #1
publishing #2
...
```

Pour arreter la publication continue, utiliser `Ctrl+C`.

Ligne droite:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

Virage lent:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.25}}"
```

Rotation sur place:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Marche arriere:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.25}, angular: {z: 0.15}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Publier une commande une seule fois:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.2}}"
```

Publier plus vite, par exemple a 20 Hz:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.2}}"
```

## 3 bis. Demo automatique

Cette commande lance une sequence: ligne droite, virage lent, rotation, marche arriere, stop.

```bash
ros2 run ros2_stage cmd_vel_demo_node
```

La sequence est volontairement prudente: apres chaque virage, elle ajoute une ligne droite pour ramener `beta` vers zero.
Si `beta` reste bloque proche de `+/-70 deg`, envoyer une commande de recuperation:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

Puis arreter:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 4. Observer les sorties

Odometry:

```bash
ros2 topic echo /odom
```

Etat trailer:

```bash
ros2 topic echo /trailer/state
```

Angle beta seulement:

```bash
ros2 topic echo /trailer/beta
```

Mesures completes:

```bash
ros2 topic echo /trailer/metrics
```

Risque lisible:

```bash
ros2 topic echo /trailer/risk
```

Ordre des valeurs de `/trailer/metrics`:

```text
[x, y, theta_rad, theta_deg, beta_rad, beta_deg, beta_dot_rad_s,
 beta_dot_deg_s, distance_m, v_m_s, omega_rad_s, radius_m, risk_code]
```

`risk_code`:

```text
0 = SAFE
1 = WARNING
2 = DANGER
```

TF robot:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

TF trailer:

```bash
ros2 run tf2_ros tf2_echo base_link trailer_link
```

## 5. Parametres utiles

Changer la distance attelage -> essieu trailer:

```bash
ros2 launch ros2_stage phase1.launch.py trailer_length:=1.0
```

Changer la distance robot -> attelage:

```bash
ros2 launch ros2_stage phase1.launch.py hitch_offset:=0.45
```

Changer la limite jackknife:

```bash
ros2 launch ros2_stage phase1.launch.py beta_limit_deg:=70.0
```

## 6. Test attendu

- Si `v > 0` et `omega = 0`, le robot avance et `beta` reste proche de zero.
- Si `v > 0` et `omega != 0`, `theta` change et `beta` evolue.
- Si `abs(beta)` approche `beta_limit_deg`, le noeud publie un warning.
- La chaine TF attendue est:

```text
odom -> base_link -> hitch_link -> trailer_link
```
