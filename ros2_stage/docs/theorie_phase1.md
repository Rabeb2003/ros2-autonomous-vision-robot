# Theorie phase 1: robot differentiel avec trailer

## 1. Description du systeme

Le systeme est compose:

- d'un robot mobile a entrainement differentiel;
- d'un point d'attelage situe derriere le robot;
- d'une remorque passive, rigide, reliee par une liaison pivot.

Le mouvement est suppose planaire. Les variables principales sont:

```text
x, y      position du robot dans le repere odom
theta     orientation du robot
beta      angle d'articulation entre le robot et le trailer
v         vitesse lineaire recue sur /cmd_vel.linear.x
omega     vitesse angulaire recue sur /cmd_vel.angular.z
```

Convention ROS:

- X vers l'avant;
- Y vers la gauche;
- Z vers le haut.

## 2. Hypotheses

1. Mouvement dans le plan horizontal: `z = 0`.
2. Roulement sans glissement longitudinal.
3. Pas de glissement lateral des roues.
4. Trailer rigide.
5. Liaison robot-trailer parfaite.

## 3. Robot differentiel

L'etat du robot est:

```text
q = [x, y, theta]^T
```

Les entrees sont:

```text
u = [v, omega]^T
```

Equations cinematiques:

```text
dx/dt     = v cos(theta)
dy/dt     = v sin(theta)
dtheta/dt = omega
```

Forme matricielle:

```text
[dx/dt    ]   [cos(theta)  0] [v    ]
[dy/dt    ] = [sin(theta)  0] [omega]
[dtheta/dt]   [0           1]
```

## 4. Trailer et angle beta

`beta` est l'angle relatif entre l'axe du robot et l'axe du trailer.

Dans ce projet phase 1, le modele cinematique utilise:

```text
dbeta/dt = -omega - (v sin(beta)) / d
```

avec:

```text
d = distance entre le point d'attelage et l'essieu du trailer
```

Interpretation:

- le terme `-omega` represente l'effet direct de la rotation du robot;
- le terme `-(v sin(beta))/d` vient de la contrainte de non-glissement lateral du trailer;
- la relation est non lineaire a cause de `sin(beta)`.

## 5. Limite jackknife

Une configuration critique apparait lorsque l'angle `beta` devient trop grand.

On definit une limite:

```text
|beta| <= beta_max
```

Exemple:

```text
beta_max = 70 deg
```

Si `|beta|` depasse cette limite, le systeme est proche d'une configuration de type jackknife. Dans la simulation phase 1, le noeud limite `beta` et publie un warning.

## 6. Matrices de transformation

Transformation monde/odom vers robot:

```text
T_odom_robot =
[ cos(theta)  -sin(theta)  0  x ]
[ sin(theta)   cos(theta)  0  y ]
[ 0            0           1  0 ]
[ 0            0           0  1 ]
```

Transformation robot vers attelage:

```text
T_robot_hitch =
[ 1  0  0  -Lh ]
[ 0  1  0   0  ]
[ 0  0  1   0  ]
[ 0  0  0   1  ]
```

Transformation attelage vers trailer:

```text
T_hitch_trailer =
[ cos(beta)  -sin(beta)  0  -d ]
[ sin(beta)   cos(beta)  0   0 ]
[ 0           0          1   0 ]
[ 0           0          0   1 ]
```

Transformation complete:

```text
T_odom_trailer = T_odom_robot * T_robot_hitch * T_hitch_trailer
```

Dans ROS2, cette composition est realisee par TF2.
Le noeud cinematique publie `odom -> base_link`.
`robot_state_publisher` publie ensuite les transformations du trailer a partir de l'URDF et de `/joint_states`:

```text
odom -> base_link -> hitch_link -> trailer_link
```

## 7. Correspondance ROS2

| Theorie | ROS2 |
| --- | --- |
| `v` | `/cmd_vel.linear.x` |
| `omega` | `/cmd_vel.angular.z` |
| `x, y, theta` | `/odom` et TF `odom -> base_link` |
| `beta` | `/trailer/beta` et TF `base_link -> trailer_link` |
| matrices de transformation | TF2 |

## 8. Tests de validation

1. Ligne droite: `v > 0`, `omega = 0`.
   - resultat attendu: `beta` reste proche de zero.
2. Virage lent: `v > 0`, `omega != 0`.
   - resultat attendu: `theta` et `beta` evoluent progressivement.
3. Rotation sur place: `v = 0`, `omega != 0`.
   - resultat attendu: le robot tourne, le trailer s'oriente par rapport au robot.
4. Marche arriere: `v < 0`.
   - resultat attendu: `beta` peut croitre rapidement, donc il faut surveiller la limite jackknife.

## 9. Grandeurs observees dans ROS2

Pour relier la theorie a la simulation, le noeud publie:

```text
/odom             position x, y et orientation theta du robot
/trailer/beta     angle beta en radians
/trailer/state    etat brut [x, y, theta, beta, v, omega, beta_dot, distance]
/trailer/metrics  etat lisible avec theta/beta en degres, distance et risque
/trailer/risk     message texte SAFE/WARNING/DANGER
```

Le calcul de distance est:

```text
distance(t + dt) = distance(t) + |v| dt
```

Le rayon de virage est:

```text
R = v / omega
```

si `omega != 0`. Si `omega = 0`, le mouvement est rectiligne et `R` est infini.
