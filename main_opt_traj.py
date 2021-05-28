# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 08:51:56 2021

This code uses a quasi-AAO MDO formulation for the feedback coupligns
to optimize a TSTO Launch vehicle.
It includes 3 main disciplines: Propulsion, Mass-Sizing and trajectory.
The trajectory optimization is carried using the Legendre Gauss Lobatto Transcription.
The guidance of the launcher is done using a parameterized pitch angle guidance program.

The main environemnts used are Dymos and OpenMDAO. Two open-source libraries by NASA.

@author: jorge
"""

import openmdao.api as om
import dymos as dm
import numpy as np
import time
from openmdao.api import ScipyOptimizeDriver

from celestialBodies import Earth
from vehicles import TSTO
from plotState_sim import plotState, saveTraj
from importInitialGuess import importInitialGuess, readGuessFile, importInitialGuess_manualInput, importInitialGuess_random
from writeReport import writeReport, updateVehicle
from writeBatchReport import writeBatchReport

from constraints.defineCouplingConstraints import defineCouplingConstraints

from groups.propulsion.propulsion import Propulsion
from groups.massSizing.massSizing import MassSizing

from groups.trajectory.defineTrajectoryPhases import defineTrajectoryPhases
from defineConnections import defineConnections

from external_parameters import external_params



# intialize openmmdao problem
# =========================================================================================================
p = om.Problem(model=om.Group())

# Define NLP solver
# =========================================================================================================
p.driver = ScipyOptimizeDriver()
p.driver.options['optimizer'] = 'SLSQP'
# set tol 1e-4 for a precission of  10 kg. account for objective function scaling.
# adder = -ref0
# scaler = 1 / (ref - ref0) 
# x_sc = scaler (x - ref0)
p.driver.options['tol'] = 1e-3
p.driver.options['maxiter'] = 100
p.driver.declare_coloring()


# initialize earth and vehicle. Set orbit objectives and define intial guess type
# ========================================================================================================

# Initialize earth
earth = Earth()

# initialize TSTO(md, mplf, mass_aux_1, nb_e_first_stage, nb_e_second_stage, centralBody)
rocket = TSTO(11e3 , 1.9e3, 3e3, 9 , 1, 0.64, 0.64,  earth)

# orbit objectives
ha     = 400e3  # height at apogee(m)
hp_min = 145e3  # min height at perigee(m)

# define initial guess
# if guess_type == 'manual'        the initial guess uses manual inputs from the 
#                                  file "importInitialGuess" function "importInitialGuess_manualInput".

# if guess_type == 'saved'         the initial guess is read from a .db file. If any variable name 
#                                  is changed a new .db file  must be generated.

# if guess_type == 'semi-random'   the initial guess is set randomly for design parameters and time duration.
#                                  Initial guess  for states is the same as for guess_type == 'manual'.
#                                  It convverges only 25% of the time 


guess_type = 'saved'

# specify path to initial guess .db file in case guess_type == 'saved'
guess_file = 'initial_guess/F9_11Ton_400km.db'


# add_main modules to the model
# =========================================================================================================
# external parameters. Contains and defines the units, bounds and scaling parameters of 
# the variables to be optimized related to the massSizing and propulsion modules.
external_params = p.model.add_subsystem('external_params', external_params)

# add propulsion module. contains models for first and second stage
propulsion = p.model.add_subsystem('propulsion', Propulsion( g0 = earth.g0, 
                                                            nb_e_first_stage = rocket.stage_1.nb_e, 
                                                            nb_e_second_stage = rocket.stage_2.nb_e))

# add massSizing module. contains models for first and second stage
massSizing = p.model.add_subsystem('massSizing', MassSizing( nb_e_first_stage = rocket.stage_1.nb_e, 
                                                            mass_aux_1 = rocket.stage_1.mass_aux ) )

# add trajectory module. Contains and defines the units, bounds and scaling parameters of 
# the variables to be optimized related to the trajectory
traj, phases = defineTrajectoryPhases(earth, rocket, ha, hp_min)
p.model.add_subsystem('traj', traj)

# define constraints for the trajectory
# ===========================================================================================================
phases['lift_off'].add_boundary_constraint('r', units = 'm', 
                                           loc='final' , shape=(1,),
                                           lower=earth.r0 + 150 , upper = earth.r0 + 2000,
                                           ref= earth.r0 + 2000, ref0= earth.r0 + 150)

phases['gravity_turn'].add_boundary_constraint(name='qDot.qDot', units='kg/m/s**3', 
                                               loc='final', shape=(1,), 
                                               equals = 0.0)

phases['gravity_turn_c'].add_boundary_constraint(name='aero.q_dyn', units='Pa', 
                                                 loc='final', shape=(1,), 
                                                 upper=1e3, 
                                                 ref=2e3, ref0 = 1e3)

phases['exoatmos_a'].add_boundary_constraint('aero.q_heat', units = 'W/m**2',
                                             loc= 'final', shape=(1,),
                                             upper = 1135,  
                                             ref= 2e4, ref0=1135)

phases['exoatmos_b'].add_boundary_constraint('orbitalParameters.ra', units='m', 
                                       loc='final', shape=(1,),
                                       lower = earth.r0 + ha , upper = earth.r0 + ha + 2e4,
                                       ref=earth.r0 + ha + 2e4, ref0= earth.r0 + ha)
    
phases['exoatmos_b'].add_boundary_constraint('orbitalParameters.rp', units='m', 
                                       loc='final', shape=(1,),
                                       lower = earth.r0 + hp_min,
                                       ref=earth.r0 + 2*hp_min,
                                       ref0=earth.r0 + hp_min )

# define optimization objective 
# ======================================================================================================
phases['lift_off'].add_objective('m',loc='initial', ref=4.5E5, ref0 = 3.5E5)

# define connections between modules
# ======================================================================================================
p = defineConnections(p)

# define constraints for constraint coupling components. append constraints components in dictionary
# ======================================================================================================
p, constraintComponents = defineCouplingConstraints(p, rocket)

# setup linear solver and problem
# ======================================================================================================
p.model.linear_solver = om.DirectSolver()
p.setup(check=True,force_alloc_complex=True)

# store constraints and design parameters in lists and dictionaries
# =======================================================================================================
constraints_str = []

for constraintComponent in constraintComponents.values():
    for constraint in constraintComponent.get_constraints().keys():
        constraints_str.append(constraint)

for i in traj.get_constraints().keys():
    if 'final_boundary' in i:
        constraints_str.append(i)
        
for i in propulsion.get_constraints().keys():
    constraints_str.append(i)


# store all the design parameters and phase_durations from trajectory in dictionaries
design_params = {}
traj_phase_duration = {}

for key, value in traj.get_design_vars().items():
    if '.t_duration' in key:
        traj_phase_duration[key] = value
    elif 'design_params' in key :
        design_params[key] = value

# this extra loop is to store phases in the right order
for key, value in traj.get_design_vars().items():
    if '_t_duration' in key:
        traj_phase_duration[key] = value
        
# store external design parameters in list
for key, value in external_params.get_design_vars().items():
    if '_t_duration' not in key:
        design_params[key] = value
    
# import initial guess
# ==========================================================================================================
# group states into dictionary and define lower and upper bounds for random initialization
states = {'r':[earth.r0, earth.r0 + 200e3],
          'v':[1e-3, 8e3],
          'lambda':[1e-3, np.radians(15)],
          'm':[rocket.md, 600e3],
          'phi':[1e-3, np.pi/2]}

if guess_type == 'manual':
    p = importInitialGuess_manualInput(p, phases, design_params, traj_phase_duration, states, False)
    guess_file = 'manual guess'
    
elif guess_type == 'semi-random':
    p = importInitialGuess_manualInput(p, phases, design_params, traj_phase_duration, states, True)
    guess_file = 'semi-random guess'
    
elif guess_type == 'random':
    p = importInitialGuess_random(p, phases, design_params, traj_phase_duration, states)
    guess_file = 'random guess'
    
elif guess_type == 'saved':
    initial_guess = readGuessFile (guess_file)
    p = importInitialGuess(p, phases, design_params.keys(), initial_guess, states.keys())


# solve the problem
# ==========================================================================================================
# define start time to calculate execution time
start_time = time.time()

# run problem
dm.run_problem(p)


# post processing
# ==========================================================================================================
# update vehicle with the results from the optimization
rocket = updateVehicle(p, rocket)

# write optimization reports
# place this function before simulating trajectory or checking partials to get the correct optimization time
writeBatchReport('0', p,  start_time)
writeReport('0', p, guess_file, rocket, design_params.keys(), constraints_str, traj_phase_duration.keys() , start_time, ha, hp_min, False)

# check partials
std_out = open("results/std_out.txt","w")
p.check_partials(method='cs', compact_print=True, show_only_incorrect=True, out_stream=std_out)
std_out.close()

# Get the explicitly simulated results, plot and save trajectory to txt file
try:
    exp_out = traj.simulate()
    plotState(p,earth,phases,exp_out, False, True, '0') 
    plotState(p,earth,phases,exp_out, True, False, '0')
    saveTraj(p, phases, exp_out, True, earth)
except:
    print('An error ocurred during the simulaton of the  results')

# generate n2 diagram
# ==========================================================================================================
om.n2(p, outfile='n2.html', show_browser = False)

