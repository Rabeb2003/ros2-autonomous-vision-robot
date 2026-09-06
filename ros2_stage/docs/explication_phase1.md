# Explication phase 1: frames, parametres et securite

## 1. Que representent `odom`, `base_link` et le trailer ?

Dans cette phase, on utilise la chaine TF suivante:

```text
odom -> base_link -> hitch_link -> trailer_link
```

### `odom`

`odom` represente le repere monde local de la simulation cinematique.
Il ne bouge pas.
C'est dans ce repere que l'on exprime:

- la position `x`;
- la position `y`;
- l'orientation du robot `theta`.

Dans le code, `odom` est defini par le parametre:

```python
odom_frame = 'odom'
```

### `base_link`

`base_link` represente le corps principal du robot tracteur.
C'est le repere du robot.

Convention ROS:

- X vers l'avant du robot;
- Y vers la gauche;
- Z vers le haut.

Le noeud cinematique publie la transformation:

```text
odom -> base_link
```

Cette transformation contient `x`, `y` et `theta`.

### `hitch_link`

`hitch_link` represente le point d'attelage.
Il est place derriere le robot, a une distance `hitch_offset`.

Dans le Xacro:

```xml
<origin xyz="-$(arg hitch_offset) 0 0.08"/>
```

Le signe moins signifie: derriere le robot, car l'avant est l'axe X positif.

### `trailer_link`

`trailer_link` represente le corps de la remorque.
Il est relie a `hitch_link` par un joint pivot:

```xml
<joint name="trailer_hitch_joint" type="continuous">
```

L'angle de ce joint est `beta`.
Le noeud publie cet angle dans `/joint_states`, et `robot_state_publisher` l'utilise pour calculer:

```text
hitch_link -> trailer_link
```

## 2. D'ou viennent les parametres du code ?

Les parametres principaux sont dans `kinematic_trailer_node.py`:

```python
update_rate = 100.0
hitch_offset = 0.45
trailer_length = 0.85
beta_limit_deg = 70.0
cmd_vel_timeout = 0.5
```

Ils sont aussi passes par le launch:

```python
ros2 launch ros2_stage phase1.launch.py hitch_offset:=0.45 trailer_length:=0.85 beta_limit_deg:=70.0
```

## 3. Justification des parametres

### `update_rate = 100 Hz`

Le noeud integre les equations toutes les `0.01 s`.
C'est assez rapide pour limiter les erreurs numeriques.

Valeur conseillee:

```text
50 Hz <= update_rate <= 200 Hz
```

Si la frequence est trop faible, `theta` et `beta` deviennent moins precis.

### `hitch_offset = 0.45 m`

C'est la distance entre le centre du robot et le point d'attelage.

Dans notre robot:

- longueur robot visuelle: `0.70 m`;
- demi-longueur: `0.35 m`;
- on place l'attelage un peu derriere le robot: `0.45 m`.

Valeurs conseillees:

```text
0.35 m <= hitch_offset <= 0.60 m
```

Si `hitch_offset` est trop petit, la remorque est trop proche du robot.
Si `hitch_offset` est trop grand, l'attelage devient artificiellement long.

### `trailer_length = 0.85 m`

C'est la distance entre le point d'attelage et l'essieu ou centre cinematique du trailer.
Elle apparait directement dans l'equation:

```text
beta_dot = -omega - (v sin(beta)) / trailer_length
```

Valeurs conseillees:

```text
0.60 m <= trailer_length <= 1.20 m
```

Si `trailer_length` est petit, le trailer tourne tres vite et `beta` devient instable.
Si `trailer_length` est grand, le trailer reagit plus lentement.

### `beta_limit_deg = 70 deg`

C'est la limite de securite contre le jackknife.

Valeurs conseillees:

```text
45 deg <= beta_limit_deg <= 75 deg
```

Pour une simulation prudente: `45 deg`.
Pour observer le comportement limite: `70 deg`.
Au-dela de `80 deg`, le systeme est proche d'une configuration dangereuse.

### `cmd_vel_timeout = 0.5 s`

Si aucune commande `/cmd_vel` n'arrive pendant `0.5 s`, le robot s'arrete.
C'est une securite logicielle.

Valeurs conseillees:

```text
0.2 s <= cmd_vel_timeout <= 1.0 s
```

## 4. Comment choisir la distance robot-trailer ?

On distingue deux distances:

```text
hitch_offset    centre robot -> point d'attelage
trailer_length  point d'attelage -> essieu trailer
```

Choix pratique:

1. Mesurer la longueur du robot.
2. Placer l'attelage juste derriere le robot.
3. Mesurer la distance entre l'attelage et l'essieu du trailer.

Dans notre modele:

```text
longueur robot       = 0.70 m
hitch_offset         = 0.45 m
longueur trailer     = 0.56 m visuelle
trailer_length       = 0.85 m cinematique
```

`trailer_length` est plus grand que la longueur visuelle car il represente la distance cinematique entre l'attelage et le centre de suivi du trailer.

## 5. Caracteristiques du robot et du trailer

Pour la phase 1, le modele sert surtout a la visualisation et a la cinematique.
Les masses et inerties sont ajoutees comme valeurs nominales pour preparer Gazebo en phase 2.

### Robot tracteur

```text
longueur: 0.70 m
largeur:  0.42 m
hauteur:  0.20 m
masse:    8.0 kg
roues:    2 roues laterales, rayon 0.10 m
```

### Trailer

```text
longueur visuelle: 0.56 m
largeur:           0.34 m
hauteur:           0.18 m
masse:             4.0 kg
roues:             2 roues, rayon 0.08 m
```

### Capteurs

Phase 1:

```text
aucun capteur physique
```

On utilise seulement:

- `/cmd_vel` comme entree;
- `/odom` comme sortie estimee;
- TF2 pour visualiser les reperes;
- RViz pour verifier la geometrie.

Phase 2/3:

- lidar pour Nav2;
- odometrie;
- eventuellement IMU.

## 6. Quelle est la solution suivie pour un mouvement securise ?

La solution phase 1 est une simulation cinematique controlee:

1. Le noeud lit `/cmd_vel`.
2. Il recupere:

```text
v     = linear.x
omega = angular.z
```

3. Il integre:

```text
x_dot     = v cos(theta)
y_dot     = v sin(theta)
theta_dot = omega
beta_dot  = -omega - (v sin(beta)) / trailer_length
```

4. Il limite `beta` avec `beta_limit_deg`.
5. Il arrete le robot si `/cmd_vel` disparait pendant `cmd_vel_timeout`.

Le noeud publie aussi:

```text
/trailer/metrics
/trailer/risk
```

pour connaitre directement l'angle `beta`, la distance parcourue et le risque.

## 6.1 Comment beta est calcule a chaque commande ?

Quand on envoie une commande:

```text
/cmd_vel.linear.x  = v
/cmd_vel.angular.z = omega
```

le noeud calcule d'abord:

```text
beta_dot = -omega - (v sin(beta)) / trailer_length
```

Puis il integre:

```text
beta_nouveau = beta_ancien + beta_dot * dt
```

avec:

```text
dt = temps entre deux cycles
```

Dans notre cas:

```text
update_rate = 100 Hz
dt environ 0.01 s
```

Exemple si:

```text
v = 0.4 m/s
omega = 0.2 rad/s
beta = 0 deg
trailer_length = 0.85 m
```

alors:

```text
sin(0) = 0
beta_dot = -0.2 - (0.4 * 0) / 0.85
beta_dot = -0.2 rad/s
```

Donc au premier cycle:

```text
beta_nouveau = 0 + (-0.2 * 0.01)
beta_nouveau = -0.002 rad
```

soit environ:

```text
-0.11 deg
```

Ensuite, comme `beta` n'est plus nul, le terme `sin(beta)` intervient.

## 6.2 Comment la distance est calculee ?

La distance cumulee est calculee par:

```text
distance = distance + |v| * dt
```

On utilise `|v|` pour compter aussi la marche arriere comme une distance parcourue.

Exemple:

```text
v = 0.4 m/s
temps = 10 s
distance = 0.4 * 10 = 4 m
```

## 6.3 Comment connaitre le rayon du virage ?

Si `omega` n'est pas nul:

```text
R = v / omega
```

Exemple:

```text
v = 0.4 m/s
omega = 0.2 rad/s
R = 2.0 m
```

Si:

```text
omega = 0
```

alors le rayon est infini: le robot va en ligne droite.

## 6.4 Comment le risque est evalue ?

Le risque depend surtout de `beta`.

Dans le noeud:

```text
SAFE    si |beta| < safe_beta
WARNING si |beta| depasse safe_beta
DANGER  si |beta| depasse warning_beta
```

Valeurs actuelles:

```text
safe_beta    = 35 deg
warning_beta = 55 deg
beta_limit   = 70 deg
```

La marche arriere est plus dangereuse.
Donc si:

```text
v < 0
```

le noeud devient plus prudent.

Pour observer:

```bash
ros2 topic echo /trailer/risk
```

Exemple de sortie:

```text
SAFE: beta=-12.4 deg, beta_dot=-8.1 deg/s, distance=1.30 m, v=0.40 m/s, omega=0.20 rad/s, radius=2.00 m
```

## 6.5 Comment lire `/trailer/metrics` ?

Le topic contient:

```text
[x, y, theta_rad, theta_deg, beta_rad, beta_deg, beta_dot_rad_s,
 beta_dot_deg_s, distance_m, v_m_s, omega_rad_s, radius_m, risk_code]
```

avec:

```text
risk_code = 0 SAFE
risk_code = 1 WARNING
risk_code = 2 DANGER
```

## 7. Bornes conseillees des commandes

Pour les premiers tests:

```text
-0.3 <= v <= 0.5 m/s
-0.5 <= omega <= 0.5 rad/s
```

Pour rester plus stable:

```text
v >= 0
|omega| <= 0.3 rad/s
|beta| <= 45 deg
```

La marche arriere est plus critique avec trailer.
Elle peut faire augmenter rapidement `beta`.

## 8. Tests a presenter

Ligne droite:

```text
v = 0.4, omega = 0
resultat attendu: beta reste proche de 0
```

Virage lent:

```text
v = 0.4, omega = 0.2
resultat attendu: theta change, beta evolue progressivement
```

Rotation sur place:

```text
v = 0, omega = 0.5
resultat attendu: le robot tourne et beta change fortement
```

Marche arriere:

```text
v = -0.2, omega = 0.1
resultat attendu: beta peut devenir instable plus vite
```

## 8.1 Pourquoi `beta = -70 deg` reste apres l'arret ?

Si le topic `/trailer/risk` affiche:

```text
DANGER: beta=-70.0 deg, beta_dot=0.0 deg/s, v=0.00, omega=0.00
```

cela signifie:

- la remorque a atteint la limite jackknife negative;
- le noeud a bloque `beta` a `-70 deg`;
- le robot est arrete;
- donc `beta_dot = 0`, et l'angle ne peut plus revenir tout seul.

Pour recuperer, il faut avancer doucement en ligne droite:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

Avec `v > 0` et `omega = 0`, l'equation devient:

```text
beta_dot = -(v sin(beta)) / trailer_length
```

Si `beta` est negatif, `sin(beta)` est negatif, donc `beta_dot` devient positif.
Cela ramene progressivement `beta` vers zero.

## 9. Phrase simple pour l'encadrant

Le repere `odom` joue le role du monde local.
Le repere `base_link` represente le robot tracteur.
Le repere `hitch_link` represente l'attelage.
Le repere `trailer_link` represente la remorque.
Le noeud recoit `/cmd_vel`, integre les equations cinematiques du robot et du trailer, publie `/odom`, `/trailer/beta` et les informations TF necessaires pour visualiser le comportement dans RViz.
