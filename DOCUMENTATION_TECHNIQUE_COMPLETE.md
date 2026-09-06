# Documentation Technique Complète - Robot Trailer ROS2

## 1. VUE D'ENSEMBLE DU PROJET

### 1.1 Packages ROS2

| Nom du Package | Description | Type de Build |
|---------------|-------------|---------------|
| `diff_robot` | Package principal du robot tracteur avec remorque | ament_cmake |
| `trailer_kinematics` | Nœuds de cinématique de remorque (EKF beta, footprint multi-cercles) | ament_python |

### 1.2 Versions

- **ROS2**: Humble (chemins `/opt/ros/humble` détectés)
- **Gazebo**: Ignition Fortress (gz_ros2_control, ros_gz_sim)
- **Python**: 3.10+

### 1.3 Structure des Dossiers

```
ros2_diff_drive_robot/
├── src/
│   ├── diff_robot/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/          # Fichiers de configuration
│   │   ├── control/         # Scripts de contrôle
│   │   ├── launch/          # Fichiers de launch
│   │   ├── map/             # Cartes et SLAM
│   │   ├── planning/        # Configuration de planification
│   │   ├── rviz/            # Configurations RViz
│   │   ├── scripts/         # Scripts Python utilitaires
│   │   ├── src/             # Sources C++
│   │   ├── urdf/            # Modèles URDF/XACRO
│   │   └── world/           # Mondes Gazebo SDF
│   └── trailer_kinematics/
│       ├── package.xml
│       ├── setup.py
│       └── trailer_kinematics/
│           ├── beta_ekf_node.py
│           ├── multi_circle_footprint_node.py
│           ├── odometry_publisher.py
│           └── hitch_angle_stabilizer.py
├── build/
├── install/
└── log/
```

### 1.4 Dépendances Principales

**diff_robot (package.xml):**
- `ament_cmake` (buildtool)
- `rclcpp`, `robot_state_publisher` (build/exec)
- `ros_gz_sim`, `ros_gz_bridge` (build/exec)
- `controller_manager`, `diff_drive_controller`, `gz_ros2_control` (depend)
- `joint_state_publisher`, `joint_state_broadcaster` (depend)
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros` (depend)

**trailer_kinematics (package.xml):**
- `ament_python` (buildtool)
- `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros` (depend)

---

## 2. DESCRIPTION DU ROBOT (URDF/SDF)

### 2.1 Modèle Robot

- **Nom du modèle**: `robot_remorque`
- **Fichier URDF principal**: `/src/diff_robot/urdf/robot_remorque.urdf` (465 lignes)
- **Type de base**: Différentielle (2 roues motrices arrière, 2 roues avant passives)

### 2.2 Liens Principaux (Links)

| Nom du Link | Description | Position relative |
|-------------|-------------|-------------------|
| `base_footprint` | Frame racine du robot (au sol) | (0, 0, 0) |
| `base_link` | Châssis principal du tracteur | (0, 0, 0.085) |
| `rear_left` | Roue motrice gauche | (-0.203, 0.201, -0.085) |
| `rear_right` | Roue motrice droite | (-0.203, -0.201, -0.085) |
| `front_left` | Roue avant passive (caster) | (0.171, 0.242, -0.085) |
| `front_right` | Roue avant passive (caster) | (0.171, -0.159, -0.085) |
| `support` | Support capteurs | (0, 0, 0) |
| `lidar_link` | LIDAR YDLidar | (-0.02, 0.04, 0.20) |
| `camera_link` | Caméra | (0.08, 0, 0.14) |
| `imu_link` | IMU | (0, 0, 0.05) |
| `gps_link` | GPS | (0.2, 0, 0.1) |
| `trailer_base_link` | Châssis de la remorque | (-0.3575, 0.0414, -0.085) |
| `trailer_wheel_left` | Roue remorque gauche | (0.5000, -0.19187, 0.0) |
| `trailer_wheel_right` | Roue remorque droite | (0.5000, 0.19183, 0.0) |

### 2.3 Joints Principaux

| Nom du Joint | Type | Limites | Description |
|--------------|------|---------|-------------|
| `base_footprint_joint` | fixed | - | base_footprint → base_link |
| `rear_left_joint` | continuous | effort: 50, vel: 2.0 | Roue motrice gauche |
| `rear_right_joint` | continuous | effort: 50, vel: 2.0 | Roue motrice droite |
| `front_left_joint` | continuous | effort: 50, vel: 2.0 | Caster avant gauche |
| `front_right_joint` | continuous | effort: 50, vel: 2.0 | Caster avant droit |
| `hitch_joint` | revolute | [-0.785, 0.785] rad (±45°) | Attelage tracteur-remorque |
| `trailer_wheel_left_joint` | continuous | effort: 50, vel: 2.0 | Roue remorque gauche |
| `trailer_wheel_right_joint` | continuous | effort: 50, vel: 2.0 | Roue remorque droite |

### 2.4 Capteurs et Topics

| Capteur | Type | Topic ROS | Frame | Fréquence | Portée/Spécifications |
|---------|------|-----------|-------|-----------|---------------------|
| **LIDAR** | gpu_lidar | `/scan` | `lidar_link` | 10 Hz | 0.15-12.0m, 240 samples, FOV: ±120° |
| **IMU** | imu | `/imu` | `imu_link` | 50 Hz | Angular velocity + linear acceleration |
| **GPS** | navsat | `/gps/fix` | `gps_link` | 10 Hz | Position LLA avec bruit gaussien (stddev: 0.5m) |
| **Camera** | camera | `/image_raw` | `camera_link` | 30 Hz | 640x480, FOV: 90° |

### 2.5 Plugins Gazebo

**Plugin ros2_control** (`tractor_ros2_control.xacro`, lignes 11-80):
- **Plugin**: `gz_ros2_control/GazeboSimSystem`
- **Joints actionnés**: `rear_left_joint`, `rear_right_joint` (velocity command)
- **Joints passifs**: `front_left_joint`, `front_right_joint`, `hitch_joint`, `trailer_wheel_left_joint`, `trailer_wheel_right_joint` (state only)
- **Fichier config**: `/src/diff_robot/config/diff_drive_controller.yaml`
- **Remapping**: `/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel`

**Plugins capteurs** (`robot_remorque.urdf`):
- **LIDAR** (lignes 404-431): `gpu_lidar` avec visualisation
- **Camera** (lignes 440-459): `camera` avec rendu Ogre2
- **IMU** (`sensors_gps_imu.xacro`, lignes 81-100): `imu` avec bruit gaussien
- **GPS** (`sensors_gps_imu.xacro`, lignes 28-58): `navsat` avec bruit de position

### 2.6 Arborescence TF Complète

```
map
 └─ odom (publié par EKF global ou AMCL)
     └─ base_footprint
         ├─ base_link
         │   ├─ support
         │   │   ├─ camera_link
         │   │   ├─ lidar_link
         │   │   ├─ imu_link
         │   │   └─ gps_link
         │   ├─ rear_left
         │   ├─ rear_right
         │   ├─ front_left
         │   └─ front_right
         └─ trailer_base_link (via hitch_joint)
             ├─ trailer_wheel_left
             └─ trailer_wheel_right
```

**Frame racine TF**: `map` (pour navigation) ou `odom` (pour odométrie locale)

---

## 3. STACK DE NAVIGATION

### 3.1 Stack Utilisée

**Nav2 (Navigation2)** avec configuration personnalisée pour robot tracteur-remorque

### 3.2 Nœuds Nav2 Lancés

| Nœud | Package | Description | Fichier config |
|------|---------|-------------|----------------|
| `map_server` | nav2_map_server | Charge et publie la carte statique | nav2_params.yaml |
| `lifecycle_manager_localization` | nav2_lifecycle_manager | Gère le cycle de vie du map_server | - |
| `planner_server` | nav2_planner | Planificateur global (SMAC Lattice) | nav2_params.yaml |
| `controller_server` | nav2_controller | Contrôleur local (Regulated Pure Pursuit) | nav2_params.yaml |
| `smoother_server` | nav2_smoother | Lissage de trajectoire | nav2_params.yaml |
| `behavior_server` | nav2_behaviors | Comportements de récupération (spin, backup, wait) | nav2_params.yaml |
| `bt_navigator` | nav2_bt_navigator | Exécute l'arbre de comportement Nav2 | nav2_params.yaml |
| `waypoint_follower` | nav2_waypoint_follower | Suivi de waypoints | nav2_params.yaml |
| `velocity_smoother` | nav2_velocity_smoother | Lissage des commandes de vitesse | nav2_params.yaml |
| `lifecycle_manager_navigation` | nav2_lifecycle_manager | Gère le cycle de vie des nœuds Nav2 | - |

### 3.3 Fichiers de Configuration

#### 3.3.1 Configuration Nav2 Principale

**Fichier**: `/src/diff_robot/config/nav2_params.yaml` (370 lignes)

**Planificateur Global** (lignes 84-133):
- **Plugin**: `nav2_smac_planner/SmacPlannerLattice`
- **Motion primitives**: `/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json`
- **Motion model**: `DIFF_DRIVE`
- **Minimum turning radius**: 0.5 m
- **Allow reverse**: true
- **Use cost function heuristic**: true

**Contrôleur Local** (lignes 135-202):
- **Plugin**: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- **Desired linear velocity**: 0.70 m/s
- **Max linear accel/decel**: 1.0 m/s²
- **Lookahead distance**: 1.0 m (min: 0.6, max: 1.5)
- **Rotate to heading angular velocity**: 1.5 rad/s
- **Use regulated linear velocity scaling**: true (réduit vitesse dans les virages)
- **Use cost regulated linear velocity scaling**: true
- **Collision detection**: désactivé (`use_collision_detection: false`)
- **Goal tolerances**: xy: 0.50 m, yaw: 0.79 rad (~45°)

**Costmaps** (lignes 269-350):
- **Local costmap**: 10x10m, rolling window, résolution 0.05m
- **Global costmap**: carte complète, résolution 0.05m
- **Footprint**: `[[0.30, 0.21], [0.30, -0.21], [-0.30, -0.21], [-0.30, 0.21]]` (tracteur uniquement)
- **Inflation radius**: 0.25 m
- **Cost scaling factor**: 5.0
- **Layers**: static_layer, obstacle_layer (scan), inflation_layer

#### 3.3.2 Configuration Localisation

**Fichier**: `/src/diff_robot/config/gps_ekf.yaml` (83 lignes)

**EKF Local** (`ekf_filter_node_odom`, lignes 1-26):
- **TF publiée**: `odom -> base_footprint`
- **Sensors fusionnés**: wheel odometry (`/diff_drive_controller/odom`) + IMU (`/imu`)
- **Frequency**: 30 Hz
- **World frame**: `odom` (continue dans le temps, REP 105)

**EKF Global** (`ekf_filter_node_map`, lignes 28-60):
- **TF publiée**: `map -> odom`
- **Sensors fusionnés**: wheel odometry + IMU + GPS (via `navsat_transform`)
- **Frequency**: 30 Hz
- **World frame**: `map`

**NavSat Transform** (`navsat_transform`, lignes 62-83):
- **Frequency**: 30 Hz
- **Delay**: 3.0 s
- **Magnetic declination**: 0.0 rad
- **Broadcast cartesian transform**: true
- **Use odometry yaw**: true
- **Wait for datum**: true
- **Datum fixe**: [49.0, 3.0, 0.0] (évite saut au démarrage)

#### 3.3.3 Configuration Contrôleur Différentiel

**Fichier**: `/src/diff_robot/config/diff_drive_controller.yaml` (33 lignes)

**Paramètres** (lignes 15-32):
- **Left wheel**: `rear_left_joint`
- **Right wheel**: `rear_right_joint`
- **Wheel separation**: 0.402 m
- **Wheel radius**: 0.085 m
- **Publish rate**: 50 Hz
- **Odom frame**: `odom`
- **Base frame**: `base_footprint`
- **Enable odom TF**: `false` (publié par EKF)
- **Cmd vel timeout**: 0.5 s

### 3.4 Localisation

**Approche actuelle**: GPS + EKF (robot_localization)
- **EKF local**: Fusionne odométrie des roues + IMU pour `odom -> base_footprint`
- **EKF global**: Fusionne odométrie + IMU + GPS pour `map -> odom`
- **NavSat transform**: Convertit GPS LLA en coordonnées cartésiennes

**Note**: Le fichier `nav2.launch.py` a été modifié pour utiliser AMCL à la place du GPS/EKF (voir section 8 pour l'état actuel).

### 3.5 Topics Clés

| Topic | Type | Description | Direction |
|-------|------|-------------|-----------|
| `/cmd_vel` | geometry_msgs/Twist | Commande de vitesse finale vers contrôleur | Sub (diff_drive_controller) |
| `/cmd_vel_nav2` | geometry_msgs/Twist | Sortie Nav2 après velocity_smoother | Sub (trailer_aware_controller) |
| `/cmd_vel_raw` | geometry_msgs/Twist | Sortie trailer_aware_controller | Sub (cmd_vel_safety_node) |
| `/scan` | sensor_msgs/LaserScan | Données LIDAR | Sub (costmaps) |
| `/odom` | nav_msgs/Odometry | Odométrie brute du contrôleur | Sub (EKF) |
| `/odometry/local` | nav_msgs/Odometry | Odométrie filtrée locale (EKF) | Pub (EKF local) |
| `/odometry/global` | nav_msgs/Odometry | Odométrie filtrée globale (EKF) | Pub (EKF global) |
| `/map` | nav_msgs/OccupancyGrid | Carte statique | Pub (map_server) |
| `/gps/fix` | sensor_msgs/NavSatFix | Position GPS | Pub (Gazebo bridge) |
| `/imu` | sensor_msgs/Imu | Données IMU | Pub (Gazebo bridge) |
| `/trailer/beta` | std_msgs/Float64 | Angle d'attelage remorque | Pub (trailer_joint_publisher) |

### 3.6 Envoi d'Objectif de Navigation

**Action**: `NavigateToPose` (Nav2)
- **Action server**: `/navigate_to_pose`
- **Type**: `nav2_msgs/action/NavigateToPose`
- **Utilisation**: Via RViz2 (outil "2D Goal Pose") ou ligne de commande:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}}}"
```

---

## 4. LOGIQUE D'ÉVITEMENT D'OBSTACLES

### 4.1 Algorithme/Mécanisme

**Approche multi-couche**:
1. **Planification globale** (SMAC Lattice): génère des chemins respectant les contraintes cinématiques
2. **Suivi local** (Regulated Pure Pursuit): suit le chemin avec lissage de vitesse
3. **Costmaps**: 
   - **Global costmap**: obstacles statiques (carte) + dynamiques (scan)
   - **Local costmap**: obstacles dynamiques dans fenêtre glissante (10x10m)
4. **Inflation layer**: crée une zone de coût autour des obstacles (rayon: 0.25m)

### 4.2 Paramètres de Sécurité

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| `inflation_radius` | 0.25 m | Rayon d'inflation des obstacles |
| `cost_scaling_factor` | 5.0 | Facteur d'échelle du coût |
| `footprint_padding` | 0.03 m | Marge de sécurité autour du footprint |
| `obstacle_max_range` | 2.5 m | Portée max des obstacles pour costmap |
| `obstacle_min_range` | 0.15 m | Portée min (évite auto-détection) |
| `raytrace_max_range` | 3.0 m | Portée raytracing pour clearing |

### 4.3 Comportement en Cas de Blocage

**Recovery behaviors** (configurés dans `behavior_server`, lignes 217-240):
1. **Clear costmap**: Nettoie les obstacles obsolètes
2. **Wait**: Attend 0.5s (configurable)
3. **BackUp**: Recule de 0.3m à 0.15 m/s
4. **Spin**: Tourne sur place de 360°

**Ordre personnalisé** (fichier BT XML `navigate_to_pose_trailer_recovery.xml`):
- ClearCostmap → Wait → BackUp → Spin (évite que spin/backup soient bloqués par des données obsolètes)

**Progress checker** (lignes 152-157):
- **Required movement radius**: 0.3 m
- **Movement time allowance**: 300.0 s (très généreux pour systèmes remorque lents)

---

## 5. LAUNCH FILES

### 5.1 Liste des Launch Files

| Fichier | Description | Arguments principaux |
|---------|---------------------|---------------------|
| `robot_remorque_ignition.launch.py` | Simulation Gazebo Ignition + robot + capteurs | `use_rviz`, `x_pose`, `y_pose`, `spawn_controllers`, `use_static_map_odom`, `use_gps_localization`, `use_local_ekf` |
| `nav2.launch.py` | Stack Nav2 complète avec localisation | `use_sim_time`, `map`, `params_file`, `trailer_params_file`, `rviz` |
| `nav2_trailer_safety.launch.py` | Intégration Nav2 + nœuds sécurité remorque | `use_nav2`, `use_rviz`, `use_gps_localization` |
| `gps_localization.launch.py` | Localisation GPS + EKF (map->odom) | Aucun (délais codés en dur) |
| `slam_toolbox_ignition.launch.py` | SLAM avec Gazebo Ignition | `use_sim_time`, `use_rviz` |

### 5.2 robot_remorque_ignition.launch.py

**Fichier**: `/src/diff_robot/launch/robot_remorque_ignition.launch.py` (319 lignes)

**Composants lancés**:
1. **Gazebo Ignition** (ligne 58): Monde `warehouse.world`
2. **Robot State Publisher** (ligne 74): Publie TF robot
3. **Joint State Publisher** (ligne 87): Agrège états des joints
4. **Ponts ros_gz_bridge** (lignes 102-132): Clock, scan, IMU, GPS
5. **Spawn robot** (ligne 137, TimerAction 3s): Spawn à position configurable
6. **Contrôleurs** (lignes 160-189):
   - `joint_state_broadcaster_spawner` (8s)
   - `diff_drive_controller_spawner` (10s)
7. **Localisation EKF** (ligne 212): Inclusion de `gps_localization.launch.py`
8. **Nœuds remorque** (lignes 221-260):
   - `trailer_joint_publisher`: Extrait hitch_joint de /joint_states
   - `beta_ekf_node`: Filtre EKF pour angle d'attelage
   - `multi_circle_footprint_node`: Footprint dynamique multi-cercles
9. **RViz2** (ligne 268, TimerAction 25s): Après stabilisation horloge

**Arguments par défaut** (lignes 51-60):
- `use_rviz`: `false` (modifié récemment, était `true`)
- `x_pose`: `-0.14`
- `y_pose`: `-2.07`
- `spawn_controllers`: `false` (gz_ros2_control charge automatiquement)
- `use_static_map_odom`: `false`
- `use_gps_localization`: `false`
- `use_local_ekf`: `false`

### 5.3 nav2.launch.py

**Fichier**: `/src/diff_robot/launch/nav2.launch.py` (195 lignes)

**Composants lancés** (sans délais - version originale):
1. **Map Server** (ligne 62): Charge carte YAML
2. **Lifecycle Manager Localization** (ligne 71): Gère map_server
3. **Planner Server** (ligne 81): SMAC Lattice
4. **Controller Server** (ligne 90): Regulated Pure Pursuit (output: `/cmd_vel_nav2_raw`)
5. **Smoother Server** (ligne 99): Lissage trajectoire
6. **Behavior Server** (ligne 107): Recoveries
7. **BT Navigator** (ligne 115): Arbre comportement
8. **Waypoint Follower** (ligne 123): Suivi waypoints
9. **Velocity Smoother** (ligne 132): Lissage vitesse (`/cmd_vel_nav2_raw` → `/cmd_vel_nav2`)
10. **Lifecycle Manager Navigation** (ligne 144): Gère nœuds Nav2
11. **Trailer-Aware Controller** (ligne 157): Transforme commandes pour remorque
12. **CMD Vel Safety Node** (ligne 169): Protection anti-jackknife
13. **RViz2** (ligne 182): Visualisation

**Arguments par défaut** (lignes 40-59):
- `use_sim_time`: `true`
- `map`: `/src/diff_robot/map/my_map.yaml`
- `params_file`: `/src/diff_robot/config/nav2_params.yaml`
- `trailer_params_file`: `/src/diff_robot/config/trailer_params.yaml`
- `rviz`: `true`

**Note**: Une version modifiée avec TimerActions (délais 45-75s) a été créée pour utiliser AMCL au lieu de GPS/EKF, mais n'est pas la version active.

### 5.4 gps_localization.launch.py

**Fichier**: `/src/diff_robot/launch/gps_localization.launch.py` (85 lignes)

**Composants lancés** (avec TimerActions):
1. **EKF Local** (ligne 32, TimerAction 15s): `ekf_filter_node_odom` → `odom -> base_footprint`
2. **NavSat Transform** (ligne 50, TimerAction 17s): Convertit GPS en cartésien
3. **EKF Global** (ligne 68, TimerAction 19s): `ekf_filter_node_map` → `map -> odom`

**Délais**: Importants pour éviter "jump back in time" au démarrage de Gazebo

### 5.5 Ordre de Démarrage Recommandé

**Pour simulation complète**:
```bash
# Terminal 1: Robot simulation
ros2 launch diff_robot robot_remorque_ignition.launch.py use_rviz:=false

# Terminal 2: Nav2 (après stabilisation ~30s)
ros2 launch diff_robot nav2.launch.py map:=/path/to/map.yaml rviz:=true
```

---

## 6. WORLD GAZEBO

### 6.1 Fichier World

**Fichier**: `/src/diff_robot/world/warehouse.world` (248 lignes)
**Nom**: `mecanum_warehouse`
**Format**: SDF version 1.9

### 6.2 Obstacles Statiques

| Modèle | Position (x, y, z) | Dimensions | Description |
|--------|-------------------|------------|-------------|
| `ground_plane` | (0, 0, 0) | 30x30m | Sol |
| `south_wall` | (0, -7, 1.0) | 16x0.2x2.0m | Mur sud |
| `north_wall` | (0, 7, 1.0) | 16x0.2x2.0m | Mur nord |
| `west_wall` | (-8, 0, 1.0) | 0.2x14x2.0m | Mur ouest |
| `east_wall` | (8, 0, 1.0) | 0.2x14x2.0m | Mur est |
| `divider_west` | (-1.5, 0, 0.75) | 5x0.2x1.5m | Séparateur ouest |
| `divider_east` | (4.5, 0, 0.75) | 5x0.2x1.5m | Séparateur est |
| `room_wall_north` | (-4, 3.5, 0.75) | 5x0.2x1.5m | Mur nord pièce |
| `room_wall_south` | (-4, -3.5, 0.75) | 5x0.2x1.5m | Mur sud pièce |
| `shelf_1` | (2.0, 4.5, 0.4) | 3.0x0.5x0.8m | Étagère 1 |
| `shelf_2` | (2.0, -4.5, 0.4) | 3.0x0.5x0.8m | Étagère 2 |
| `box_1` à `box_4` | (6.5, 3.0-5.0, 0.25/0.75) | 0.5x0.5x0.5m | Caisses variées |
| `box_5`, `box_6` | (-6.0, 2.0) et (-6.5, -2.0, 0.3) | 0.6x0.6x0.6m | Grandes caisses |
| `pillar_1` à `pillar_4` | (1.0, ±2.0) et (-2.5, ±2.0, 0.75) | r=0.15m, h=1.5m | Piliers |
| `corridor_block_1`, `2` | (3.5, ±1.5, 0.25) | 0.4x0.4x0.5m | Blocs couloir |

### 6.3 Point de Spawn Initial

**Position par défaut** (arguments launch):
- `x_pose`: `-0.14`
- `y_pose`: `-2.07`
- `z_pose`: `0.10`

Cette position place le robot dans la zone ouest du warehouse, près des séparateurs.

### 6.4 Éclairage

- **Soleil** (`sun`): Lumière directionnelle avec ombres
- **Overhead** (`overhead`): Lumière zénithale sans ombres

---

## 7. INTERFACES ROS2 EXPOSÉES

### 7.1 Topics Publiés

| Topic | Type de Message | Fréquence | Description |
|-------|----------------|-----------|-------------|
| `/scan` | sensor_msgs/LaserScan | 10 Hz | Données LIDAR |
| `/imu` | sensor_msgs/Imu | 50 Hz | Données IMU (gyro + accéléro) |
| `/gps/fix` | sensor_msgs/NavSatFix | 10 Hz | Position GPS |
| `/clock` | rosgraph_msgs/Clock | - | Horloge simulation |
| `/diff_drive_controller/odom` | nav_msgs/Odometry | 50 Hz | Odometry brute contrôleur |
| `/odometry/local` | nav_msgs/Odometry | 30 Hz | Odometry filtrée locale (EKF) |
| `/odometry/global` | nav_msgs/Odometry | 30 Hz | Odometry filtrée globale (EKF) |
| `/odometry/gps` | nav_msgs/Odometry | 30 Hz | Odometry dérivée GPS |
| `/map` | nav_msgs/OccupancyGrid | 1 Hz | Carte statique |
| `/joint_states` | sensor_msgs/JointState | 50 Hz | États des joints |
| `/trailer/beta` | std_msgs/Float64 | 10 Hz | Angle d'attelage remorque |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | 2 Hz | Costmap local |
| `/local_costmap/costmap_raw` | nav_msgs/OccupancyGrid | 5 Hz | Costmap local brut |
| `/local_costmap/published_footprint` | geometry_msgs/Polygon | 2 Hz | Footprint publié |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | 1 Hz | Costmap global |
| `/image_raw` | sensor_msgs/Image | 30 Hz | Image caméra |

### 7.2 Topics Souscrits

| Topic | Type de Message | Description |
|-------|----------------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Commande vitesse vers diff_drive_controller |
| `/cmd_vel_nav2` | geometry_msgs/Twist | Commande Nav2 lissée |
| `/cmd_vel_raw` | geometry_msgs/Twist | Commande transformée pour remorque |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | Pose initiale pour localisation |

### 7.3 Services

| Service | Type | Description |
|---------|------|-------------|
| `/spawn_robot_remorque` | ros_gz_sim/Create | Spawn robot dans Gazebo |
| `/controller_manager/list_controllers` | controller_manager/ListControllers | Liste contrôleurs |
| `/controller_manager/{controller_name}/configure` | controller_manager/Configure | Configurer contrôleur |
| `/controller_manager/{controller_name}/activate` | controller_manager/Activate | Activer contrôleur |

### 7.4 Actions

| Action | Type | Description |
|--------|------|-------------|
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | Navigation vers pose |
| `/navigate_through_poses` | nav2_msgs/action/NavigateThroughPoses | Navigation via waypoints |
| `/follow_waypoints` | nav2_msgs/action/FollowWaypoints | Suivi de waypoints |

### 7.5 Frames TF Utiles

| Frame | Parent | Description |
|-------|--------|-------------|
| `map` | - | Frame global de navigation |
| `odom` | `map` | Frame d'odométrie |
| `base_footprint` | `odom` | Frame base du robot (au sol) |
| `base_link` | `base_footprint` | Châssis principal |
| `lidar_link` | `base_link` | Position LIDAR |
| `imu_link` | `base_link` | Position IMU |
| `gps_link` | `base_link` | Position GPS |
| `trailer_base_link` | `base_link` (via hitch) | Châssis remorque |

---

## 8. ÉTAT ACTUEL / LIMITATIONS CONNUES

### 8.1 Ce Qui Fonctionne de Façon Fiable

- **Simulation Gazebo Ignition**: Robot spawn correctement avec tous les capteurs
- **Contrôleurs ros2_control**: diff_drive_controller avec gz_ros2_control fonctionne
- **Ponts ros_gz_bridge**: Clock, scan, IMU, GPS bridgés correctement
- **Nœuds remorque**: beta_ekf_node, multi_circle_footprint_node, trailer_joint_publisher fonctionnent
- **Nav2 stack**: Tous les nœuds Nav2 démarrent correctement
- **Planificateur SMAC Lattice**: Génère des chemins respectant contraintes différentielles
- **Contrôleur Regulated Pure Pursuit**: Suit les chemins avec lissage de vitesse
- **RViz2**: Visualisation TF et capteurs fonctionne

### 8.2 Ce Qui Est Fragile ou Non Testé

- **Localisation GPS/EKF**: 
  - Problèmes de "jump back in time" au démarrage (nécessite délais 15-19s)
  - Covariance GPS nulle causant sauts de position
  - Nécessite datum fixe pour éviter saut au spawn
  
- **Séparation des launch files**:
  - `robot_remorque_ignition.launch.py` et `nav2.launch.py` doivent être lancés séparément
  - Timing délicat: Nav2 doit attendre ~30s après robot pour éviter erreurs TF
  
- **Configuration AMCL**:
  - `nav2.launch.py` a été modifié pour utiliser AMCL mais n'est pas testé
  - Fichier `amcl_params.yaml` référencé mais n'existe pas encore
  
- **Nœuds trailer_kinematics**:
  - `beta_ekf_node` et `multi_circle_footprint_node` ont crashé dans certains tests
  - `odometry_publisher` et `hitch_angle_stabilizer` non testés

### 8.3 Dépendances à Chemins Absolus

**Chemins absolus détectés** (à corriger pour portabilité):

1. **tractor_ros2_control.xacro** (ligne 74):
```xml
<parameters>/home/rabeb/ros2_diff_drive_robot/src/diff_robot/config/diff_drive_controller.yaml</parameters>
```
   **Devrait être**: `$(find diff_robot)/config/diff_drive_controller.yaml`

2. **nav2_params.yaml** (ligne 24):
```yaml
default_nav_to_pose_bt_xml: "/home/rabeb/ros2_diff_drive_robot/install/diff_robot/share/diff_robot/config/navigate_to_pose_trailer_recovery.xml"
```
   **Devrait être**: `$(find diff_robot)/config/navigate_to_pose_trailer_recovery.xml`

3. **nav2_params.yaml** (ligne 106):
```yaml
lattice_filepath: "/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json"
```
   **Dépendance système**: Acceptable mais documentée

### 8.4 Noms de Topics Fixes

Les topics suivants sont codés en dur et devront être remappés pour intégration:

- `/cmd_vel` (entrée diff_drive_controller)
- `/cmd_vel_nav2` (sortie Nav2)
- `/cmd_vel_raw` (sortie trailer_aware_controller)
- `/trailer/beta` (angle attelage)
- `/diff_drive_controller/odom` (odométrie brute)
- `/scan`, `/imu`, `/gps/fix` (capteurs)

### 8.5 Paramètres Codés en Dur

- **Délais TimerAction** dans `gps_localization.launch.py`: 15s, 17s, 19s
- **Délais TimerAction** dans `robot_remorque_ignition.launch.py`: 3s (spawn), 8s (JSB), 10s (diff_drive), 25s (RViz)
- **Datum GPS** dans `gps_ekf.yaml`: [49.0, 3.0, 0.0]
- **Position spawn par défaut**: x=-0.14, y=-2.07

### 8.6 Limitations Connues

1. **Pas de mode SLAM actif**: Le fichier `slam_toolbox_ignition.launch.py` existe mais n'est pas intégré au flux principal
2. **Pas de carte dynamique**: Utilise uniquement carte statique pré-enregistrée
3. **Footprint tracteur uniquement**: La remorque n'est pas incluse dans le footprint Nav2 (géré par nœud custom)
4. **Pas de gestion multi-robot**: Configuration mono-robot uniquement
5. **Dépendance Gazebo Fortress**: Non compatible avec Gazebo Classic ou Garden sans modifications

### 8.7 Recommandations pour Fusion

Pour intégrer ce projet avec un autre système:

1. **Remplacer les chemins absolus** par des chemins `$(find package)`
2. **Paramétrer les noms de topics** via launch arguments
3. **Créer un fichier amcl_params.yaml** si localisation AMCL souhaitée
4. **Unifier les launch files** en un seul launch file avec gestion des dépendances
5. **Documenter les paramètres géométriques** (hitch_distance, wheelbase) pour adaptation
6. **Tester la stabilité TF** sur longue durée avant déploiement

---

**Fin de la Documentation Technique**

*Généré automatiquement le 30 juillet 2026*
*Projet: ROS2 Differential Drive Robot with Trailer*
*Version ROS2: Humble | Gazebo: Ignition Fortress*
