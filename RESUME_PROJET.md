# Résumé du Projet : ros2_diff_drive_robot (Version Gazebo Sim/Ignition)

Ce document résume l'architecture, la configuration et le flux de navigation du robot différentiel articulé avec remorque (`robot_remorque`) sous ROS 2 Humble et Gazebo Simulation (anciennement Ignition Fortress/Harmonic).

---

## 1. Architecture Générale

Le projet simule un tracteur à entraînement différentiel tirant une remorque passive reliée par un attelage pivotant (`hitch_joint`). La localisation fusionne l'odométrie, une centrale inertielle (IMU) et un récepteur GPS pour obtenir une position globale sur carte sans dérive.

```mermaid
graph TD
    subgraph Gazebo Simulation
        GZ_IMU[Capteur IMU] -->|/imu| IMU_Bridge[imu_bridge]
        GZ_GPS[Capteur GPS] -->|/gps/fix| GPS_Bridge[gps_bridge]
        GZ_Lidar[LiDAR gpu_lidar] -->|/scan| Scan_Bridge[scan_bridge]
        GZ_Control[ign_ros2_control]
    end

    subgraph Localisation (robot_localization)
        IMU_Bridge -->|/imu| EKF_Local[ekf_filter_node_odom]
        IMU_Bridge -->|/imu| NavSat[navsat_transform_node]
        GPS_Bridge -->|/gps/fix| NavSat
        
        EKF_Local -->|odom -> base_footprint| TF_Tree[Arbre TF]
        NavSat -->|/odometry/gps| EKF_Global[ekf_filter_node_map]
        EKF_Global -->|map -> odom| TF_Tree
    end

    subgraph Contrôle & Sécurité
        Teleop[teleop_twist_keyboard] -->|/cmd_vel| GZ_Control
        Nav2[Nav2 Stack] -->|/cmd_vel_nav2| Safety[Trailer Safety Nodes]
        Safety -->|/cmd_vel| GZ_Control
    end
```

---

## 2. Composants Clés et Fichiers de Configuration

### A. Modèle Physique et Capteurs (URDF/Xacro)
*   **`robot_remorque.urdf`** : Décrit les liaisons rigides, les joints de roues, et le joint d'attelage révolu passif `hitch_joint` reliant le tracteur et la remorque.
*   **`sensors_gps_imu.xacro`** : Configure les plugins de capteurs GPS (`navsat`) et IMU avec des bruits réalistes pour Gazebo Sim.
*   **`tractor_ros2_control.xacro`** : Instancie le plugin `ign_ros2_control-system` et déclare les interfaces de commande en vitesse pour les deux roues motrices (`rear_left_joint` et `rear_right_joint`).

### B. Contrôleur de Roues (`diff_drive_controller.yaml`)
*   Gère la cinématique différentielle du tracteur.
*   Écoute sur le topic de commande `/diff_drive_controller/cmd_vel_unstamped` (qui est redirigé vers `/cmd_vel` dans le fichier URDF Xacro).
*   Publie l'odométrie brute des roues sur `/diff_drive_controller/odom`.

### C. Localisation Double-EKF (`gps_ekf.yaml`)
1.  **Local EKF (`ekf_filter_node_odom`)** : Fusionne l'odométrie des roues et l'IMU pour calculer une estimation locale continue (`odom` $\rightarrow$ `base_footprint`).
2.  **GPS Transform (`navsat_transform_node`)** : Convertit les coordonnées globales Latitude/Longitude du GPS en coordonnées cartésiennes plates (X, Y) par rapport à un point de référence (`datum`), publiées sur `/odometry/gps`.
3.  **Global EKF (`ekf_filter_node_map`)** : Fusionne la position GPS convertie avec les roues et l'IMU pour publier la pose globale corrigée et la transformation `map` $\rightarrow$ `odom`.

### D. Navigation et Sécurité de la Remorque
*   **`beta_ekf_node`** : Estime précisément l'angle d'attelage $\beta$ à partir de `/joint_states`.
*   **`dynamic_footprint_node.py`** : Recalcule en temps réel l'empreinte de collision articulée (multi-cercles) du tracteur et de sa remorque pour les costmaps de Nav2.
*   **`cmd_vel_safety_node.py`** : Empêche la remorque de se plier en portefeuille (anti-jackknifing) en limitant dynamiquement les commandes angulaires envoyées aux roues.

---

## 3. Procédure de Lancement de la Navigation GPS

Suivez ces étapes dans des terminaux distincts après avoir sourcé votre espace de travail :

### Étape 1 : Lancement de la Simulation + EKF + RViz
```bash
cd /home/rabeb/ros2_diff_drive_robot
colcon build --symlink-install --packages-select diff_robot
source install/setup.bash
ros2 launch diff_robot robot_remorque_ignition.launch.py
```
*   **Ce que cela fait** : Ouvre Gazebo Sim (Ignition), fait apparaître le robot-remorque, démarre la chaîne double EKF, lance l'estimation d'angle $\beta$, et ouvre RViz2 configuré sur le repère `map` avec l'affichage de l'AerialMap (satellite) et des incertitudes de position (covariance).

### Étape 2 : Lancement de la Navigation (Nav2)
```bash
cd /home/rabeb/ros2_diff_drive_robot
source install/setup.bash
ros2 launch diff_robot nav2.launch.py rviz:=false
```
*   **Ce que cela fait** : Démarre le serveur de cartes (`map_server` chargeant la carte `my_map.yaml`), le planificateur global de trajectoires, le contrôleur local et le gestionnaire de cycle de vie de Nav2.

### Étape 3 : Envoyer un Objectif Autonome
*   Dans RViz2, cliquez sur le bouton **Nav2 Goal** (ou **2D Goal Pose**) de la barre supérieure.
*   Cliquez et faites glisser sur la carte satellite pour définir une destination.
*   Le robot calculera sa trajectoire (ligne cyan) et s'y déplacera de façon autonome en toute sécurité.
