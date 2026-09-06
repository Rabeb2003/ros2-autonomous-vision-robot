# Plan de travail du stage

Sujet: developpement et simulation d'un robot mobile differentiel avec remorque sous ROS2 Humble.

## Phase 1: modele cinematique et visualisation ROS2

Objectif: valider la theorie et le comportement cinematique sans Nav2.

Contenu:

- etude des reperes `{odom}`, `{base_link}`, `{hitch_link}`, `{trailer_link}`;
- equations du robot differentiel;
- equation de l'angle d'articulation `beta`;
- matrices de transformation homogenes;
- implementation d'un noeud ROS2 Python;
- lecture de `/cmd_vel`;
- publication de `/odom`;
- publication de `/trailer/beta` et `/trailer/state`;
- publication TF avec `odom -> base_link`;
- publication de `/joint_states` pour animer le trailer;
- visualisation RViz.

Livrables phase 1:

- `docs/theorie_phase1.md`;
- `src/ros2_stage/ros2_stage/kinematic_trailer_node.py`;
- `src/ros2_stage/urdf/diff_trailer.urdf.xacro`;
- `src/ros2_stage/launch/phase1.launch.py`;
- `COMMANDES_PHASE1.md`.

Tests phase 1:

- ligne droite: `v > 0`, `omega = 0`;
- virage lent: `v > 0`, `omega != 0`;
- rotation sur place: `v = 0`, `omega != 0`;
- marche arriere;
- verification de la limite `beta_max`.

## Phase 2: simulation Gazebo

Objectif: passer du modele cinematique abstrait a une simulation physique.

Travail prevu:

- ajouter collisions et inerties au modele Xacro;
- ajouter roues et joints compatibles Gazebo;
- integrer un plugin de controle differentiel ou `ros2_control`;
- simuler le robot + trailer dans Gazebo;
- comparer le comportement Gazebo avec le modele cinematique phase 1;
- tester les collisions et les limites de l'angle d'articulation.

Livrables phase 2:

- monde Gazebo;
- URDF/Xacro enrichi avec inerties/collisions;
- launch Gazebo;
- rapport de comparaison theorie/simulation.

## Phase 3: navigation et scenarios avances

Objectif: ajouter navigation autonome et scenarios complets.

Travail prevu:

- integrer lidar;
- ajouter map et localisation;
- integrer Nav2;
- envoyer des objectifs dans RViz;
- tester les trajectoires avec trailer;
- analyser les limites: virage serre, marche arriere, jackknife;
- ajouter des courbes de validation `x(t)`, `y(t)`, `theta(t)`, `beta(t)`.

Livrables phase 3:

- launch Nav2;
- configuration RViz;
- cartes et scenarios;
- resultats experimentaux.

## Pourquoi separer les phases

La phase 1 isole la partie scientifique du sujet: equations, TF et integration.
La phase 2 valide la physique dans Gazebo.
La phase 3 ajoute la navigation autonome.

Cette separation evite de confondre:

- erreur de modele cinematique;
- erreur URDF/Gazebo;
- erreur Nav2/localisation.
