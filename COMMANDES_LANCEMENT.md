# Commandes de lancement du projet

Ce fichier resume les commandes utiles pour lancer le robot differentiel avec Gazebo, RViz et Nav2.

## 1. Preparation

Dans chaque nouveau terminal, charger ROS 2 et le workspace:

```bash
cd /home/rabeb/ros2_diff_drive_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Apres modification du code, recompiler:

```bash
cd /home/rabeb/ros2_diff_drive_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Lancement principal

### Terminal 1: Gazebo + robot

```bash
cd /home/rabeb/ros2_diff_drive_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch diff_robot robot.launch.py rviz:=false
```

### Terminal 2: Nav2 + map + AMCL + RViz

```bash
cd /home/rabeb/ros2_diff_drive_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch diff_robot nav2.launch.py
```

Dans RViz:

1. Verifier que `Fixed Frame` est `map`.
2. Attendre que la carte apparaisse.
3. Cliquer sur `2D Pose Estimate` et placer le robot sur la carte.
4. Cliquer sur `Nav2 Goal` et choisir une destination libre.

## 3. Lancement de secours

Si le frame `map` n'apparait pas ou si `map -> odom` manque:

```bash
cd /home/rabeb/ros2_diff_drive_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch diff_robot nav2.launch.py static_map_to_odom:=true
```

Cette option publie temporairement une transformation statique `map -> odom`.

## 4. Tests rapides

### Verifier le lidar

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
ros2 topic hz /scan
```

### Verifier l'odometrie

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

### Verifier les frames TF importants

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

### Verifier la carte et AMCL

```bash
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

### Verifier les commandes de vitesse Nav2

```bash
ros2 topic echo /cmd_vel
```

## 5. Tester le robot sans Nav2

Avancer:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Tourner:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Arreter:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 6. Controler avec le clavier

```bash
cd /home/rabeb/ros2_diff_drive_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 control/keyboard_control.py
```

Touches:

- `w`: avancer
- `s`: reculer
- `a`: tourner a gauche
- `d`: tourner a droite
- `espace`: stop
- `q`: quitter

## 7. Sauvegarder une nouvelle carte

Pendant SLAM ou navigation:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/rabeb/ros2_diff_drive_robot/map/my_new_map
```

## 8. Commandes utiles de debug

Lister les topics:

```bash
ros2 topic list
```

Lister les nodes:

```bash
ros2 node list
```

Voir le graphe TF:

```bash
ros2 run tf2_tools view_frames
```

Voir les logs Nav2 pendant le lancement:

```bash
ros2 launch diff_robot nav2.launch.py log_level:=debug
```
