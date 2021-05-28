# Multidisciplinary Optimization (MDO) of Two-Stage to Orbit (TSTO) launch vehicle

Traditionally, the MDAO of launch vehicles it's done using a Multidiscipline Feasible formulation, single shooting methods and evolutionary optimizers. This code uses a quasi-AAO MDO formulation to optimize a TSTO Launch vehicle and its trajectory using pseudospectral optimal control methods and gradient-based optimization.
It includes 3 main disciplines: Propulsion, Mass-Sizing and trajectory.

* Propulsion: Uses a surrogate model (bilinear interpolator) of Rocket CEA outputs to determine performance. Optimization of
	* Thrust at vacuum
	* Combustion chamber pressure
	* Nozzle exit pressure
	* Mixture ratio of propellants
* Mass-Sizing: Uses regressions on data of exhisitng launchers to calculate geometrical parameters and the weight of the launch vehicle. Optimization of:
	* Stage diameter
	* Mass of propellants
the structures are also dimension as a function of:
	* Maximum dynamic pressure
	* Maximum load factor
* Trajectory: Hohmann ascent to an equatorial circular orbit using a parameterized pitch angle guidance. It models the amostphere according ot the U.S Standard Atmosphere from 1962 and 1972. Accounts for variations of drag coefficient as a function of Mach number.Optimization of
	* Duration of 8 different phases
	* Pitch angle guidance parameters

## Requirements
* dymos ( v.0.15.0 included in this repository as the version available on the original repo is not compatible anymore. This is related to the issue #406 in the Dymos Github page.)
* openmdao==3.1.0

## Main features
* Legendre-Gauss-Lobatto pseudo spectral transcription for the trajectory using Dymos
* Analytic gradient calculation for all disciplines
* Chain rule for multidisciplinary gradient calculation using OpenMDAO
* The whole MDAO of the launch vehicle is driven by a single Non-Linear Programming (NLP) solver: SLSQP
* Because of the pseudospectral transcription the optimization problem involves 597 variables and 585 contraints.

## Case study. Minimize gross-lift-of-weight for a vehicle with a mission of inserting 11 tons of payload into an orbit of 400 km.
![alt text](https://github.com/JorgeValderrama/Launch-vehicle-MDO/blob/master/results/0_state_history.png)

## This sequence illustrates how 1 state of the trajectory and some design variables change during an otpimization procedure.
![alt text](https://github.com/JorgeValderrama/Launch-vehicle-MDO/blob/master/results/TSTO_opt.gif)

## Execution
run the main_opt_traj.py file

## Presentation
Valderrama, J., Brevault, L., Balesdent, M. and Urbano, A. 2021. *All-At-Once MDO formulation for coupled
optimization of launch vehicle design and its trajectory using a pseudo spectral method.* Paper accepted
for presentation at the 14th World Congress of Structural and Multidisciplinary Optimization.