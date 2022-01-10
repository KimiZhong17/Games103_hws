# Games103: Intro to Physics-Based Animation
Instructor: Huamin Wang
<br>Website: http://games-cn.org/games103

## Lab 1: Rigid body collision
### Method 1: Impulse method for the whole body
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw1/bunny_collision_impulse.gif" width="50%"/>

### Method 2: Shape matching equipped with particle impulse
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw1/bunny_collision_shape_matching.gif" width="50%"/>

## Lab 2: Cloth simulation (spring-mass model)
### Method 1: Position based dynamics
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw2/cloth_simulation_PBD.gif" width="50%"/>

### Method 2: Implicit integral (with Chebyshev acceleration)
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw2/cloth_simulation_implicit_int.gif" width="50%"/>

## Lab 3: Elastic body simulation
### Method 1: FVM with Green strain
#### without Laplacian smoothing (very unstable, could explode)
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw3/results/FVM_no_smoothing.gif" width="33%"/>

#### with Laplacian smoothing (unstable with large init height)
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw3/results/FVM_low.gif" width="33%"/> <img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw3/results/FVM_high.gif" width="33%"/>

### Method 2: FVM with hyperelastic models (StVK & Neo-Hookean) + Laplacian smoothing
#### StVK (unstable with large init height)
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw3/results/StVK_low.gif" width="33%"/> <img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw3/results/StVK_high.gif" width="33%"/>

#### Neo-Hookean (stable, but bouncy)
<img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw3/results/NH_low.gif" width="33%"/> <img src="https://github.com/KimiZhong17/Games103_hws/blob/main/hw3/results/NH_high.gif" width="33%"/>

## Lab 4:
