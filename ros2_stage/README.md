# ros2_stage

Projet de stage: developpement et simulation d'un robot mobile differentiel avec remorque sous ROS2 Humble.

Cette base correspond a la **phase 1** du stage:

- modele cinematique robot differentiel + trailer;
- lecture de `/cmd_vel`;
- integration des etats `x`, `y`, `theta`, `beta`;
- publication de `/odom`;
- publication du TF `odom -> base_link`;
- publication de `/joint_states` pour que `robot_state_publisher` calcule `base_link -> hitch_link -> trailer_link`;
- visualisation dans RViz2;
- tests simples sans Nav2.

Nav2, cartographie et navigation autonome seront ajoutes en phase 2.

## Lancement rapide

```bash
cd /home/rabeb/ros2_diff_drive_robot/ros2_stage
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ros2_stage phase1.launch.py
```

Dans un autre terminal:

```bash
cd /home/rabeb/ros2_diff_drive_robot/ros2_stage
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.2}}"
```

Ou lancer une sequence de test automatique:

```bash
ros2 run ros2_stage cmd_vel_demo_node
```

## Fichiers importants

- `src/ros2_stage/ros2_stage/kinematic_trailer_node.py`: noeud cinematique.
- `src/ros2_stage/urdf/diff_trailer.urdf.xacro`: modele robot + trailer.
- `src/ros2_stage/launch/phase1.launch.py`: lancement RViz + robot_state_publisher + noeud cinematique.
- `docs/theorie_phase1.md`: etude theorique pour le rapport.
- `docs/explication_phase1.md`: explication des frames, parametres, masses, limites et securite.
- `COMMANDES_PHASE1.md`: commandes de lancement et de test.
- `PLAN_STAGE.md`: organisation phase par phase du stage.
