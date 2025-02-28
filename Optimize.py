# Optimize.py

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
print(np.__version__)
import Vehicles
import Analyses
import Missions
import Procedure
import Plot_Mission
import matplotlib.pyplot as plt
from SUAVE.Optimization import Nexus, carpet_plot
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
import time
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    
    file = open('RANGE_ANALYSIS_OUTPUTS.txt', 'a')
    file.write('\n\n\n\n--------------------------------------------NEW ANALYSIS--------------------------------------------\n')
    file.write('Iteration , Wing_span , Chord_root , Taper , S_ref , AR , P_bhp , C_bhp , CL_ideal , CD0 , K , CD_total , V_ideal , L/D , W_0 , W_2 , RANGE\n')
    file.close()
    
    problem = setup()
    # 
    ## Base Input Values
    output = problem.objective() #roda uma vez simples
    
    ## Uncomment to view contours of the design space
    # variable_sweep(problem) 
    
    ## Uncomment for the first optimization
    # output = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    # output = scipy_setup.SciPy_Solve(problem,solver='differential_evolution',pop_size=10)
    # output = scipy_setup.SciPy_Solve(problem,solver='particle_swarm_optimization',pop_size=10)  
    
    print (output)    
    
    ## Uncomment these lines when you want to start an optimization problem from a different initial guess
#    inputs                                   = [1.28, 1.38]
#    scaling                                  = problem.optimization_problem.inputs[:,3] #have to rescale inputs to start problem from here
#    scaled_inputs                            = np.multiply(inputs,scaling)
#    problem.optimization_problem.inputs[:,1] = scaled_inputs
#    output = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    # print (output)    
  
    
    fuel_aux =SUAVE.Attributes.Propellants.Aviation_Gasoline()
    fuel_dens_aux = fuel_aux.density
    # print('\ndensidade do combustivel = ',fuel_dens_aux, '[kg/m3]')
    
    
    
    # print('fuel burn = ', problem.summary.base_mission_fuelburn, '[kg]')
    # print('fuel burn = ', problem.summary.base_mission_fuelburn/Units.lb, '[lb]')
    # print('fuel burn = ', (problem.summary.base_mission_fuelburn/fuel_dens_aux)/Units.gallon, '[gallon]')
    # print('fuel margin = ', problem.all_constraints())
    
    
    
    
    # print_mission_breakdown(problem.results.base ,filename='C182_mission_breakdown.dat', units="si")

    
#    file = open('PROBLEM_w.txt', 'w')
#    file.write(str(problem))
#    file.close()
    
        
    Plot_Mission.plot_mission(problem)
    
    
    return

# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup():

    
    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------
    #   [ tag                            , initial, (lb,ub)             , scaling , units ]
    
    
    # # WING SPAN OPTIMIZATION
    # problem.inputs = np.array([
    #     [ 'wing_span'                    ,  40.    , (   20. ,   65.   ) ,   10. , Units.feet],
    # ])
    # -------------------------------------------------------------------
    
#     # CHORDS OPTIMIZATION
#     problem.inputs = np.array([
#         [ 'Chord_root'                  ,  66.    , (   50. ,   120.   ) ,   10. , Units.inches],
#         # [ 'taper'                   ,  0.7575757575757576    , (   0.3 ,   1.   ) ,   1, Units.less],
#         [ 'wing_span'                    ,  36.    , (   20. ,   150.   ) ,   10. , Units.feet],
# ])
    # -------------------------------------------------------------------
    # CHORDS OPTIMIZATION
    problem.inputs = np.array([
    [ 'chord_root'   , 46.668395125745384 , 30. , 90. , 10. , 1*Units.inches ],
    [ 'taper'        , 0.9                , 0.3 , 0.1 , 1.  , 1*Units.less.magnitude  ],
    [ 'wing_span'    , 40.6               , 30. , 45. , 10. , 1*Units.feet  ],
], dtype=object)


    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([['Range_objective', 0.000001, Units.less.magnitude]], dtype=object)



    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
    ['Pbhp_lower_cons', '>', 0., 1., Units.less.magnitude],  # Lower boundary for Pbhp at 18k feet
    ['Pbhp_upper_cons', '<', 0., 1., Units.less.magnitude],  # Maximum power for Pbhp at 18k feet
    ['V_max_cons', '<', 0., 1., (Units.m / Units.s).magnitude],  # Max design speed allowed
    ['AR_lower_cons', '>', 0., 1., Units.less.magnitude],  # Lower historical boundary for AR
    ['AR_upper_cons', '<', 0., 1., Units.less.magnitude],  # Upper historical boundary for AR
    ['S_ref_lower_cons', '>', 0., 1., (Units.m**2).magnitude],  # Lower historical boundary for reference area
    ['S_ref_uper_cons', '<', 0., 1., (Units.m**2).magnitude]  # Upper historical boundary for reference area
], dtype=object)

    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        [ 'wing_span'                       ,   'vehicle_configurations.*.wings.main_wing.spans.projected'  ],
        [ 'Chord_root'                      ,   'vehicle_configurations.*.wings.main_wing.chords.root'      ],
        [ 'taper'                           ,   'vehicle_configurations.*.wings.main_wing.taper'            ],
        [ 'Range_objective'                 ,   'summary.Range_objective'                                   ],
        [ 'Pbhp_lower_cons'                 ,   'summary.Pbhp_lower_cons'                                   ],
        [ 'Pbhp_upper_cons'                 ,   'summary.Range_objective'                                   ],
        [ 'V_max_cons'                      ,   'summary.V_max_cons'                                   ],
        [ 'AR_lower_cons'                   ,   'summary.AR_lower_cons'                                   ],
        [ 'AR_upper_cons'                   ,   'summary.AR_upper_cons'                                   ],
        [ 'S_ref_lower_cons'                ,   'summary.S_ref_lower_cons'                                   ],
        [ 'S_ref_uper_cons'                 ,   'summary.S_ref_uper_cons'                                   ],
        # [ 'fuel_burn'                       ,   'summary.base_mission_fuelburn'                         ],
        # [ 'W_0'                             ,   'summary.W_0'                                               ],
        # [ 'Margin_fuelburn'                 ,   'summary.Margin_fuelburn'                                   ],
        # [ 'AR'                 ,   'summary.AR'                                   ],
        # [ 'L_D'                             ,   'summary.L_D'                                               ],
        # [ 'takeoff_constraint'              ,   'summary.takeoff_constraint'                            ],
        # [ 'taper_constraint'                      ,   'summary.taper_constraint'                            ],
    ]
    
    
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    # print('ANTES DO ANALISYS')
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
    # print('DEPOIS DO ANALISYS')

    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    # print('ANTES DO MISSION')
    nexus.missions = Missions.setup(nexus.analyses)
    # print('DEPOIS DO MISSION')

    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()    
    nexus.total_number_of_iterations = 0
    
    return nexus
    
def variable_sweep(problem):    
    number_of_points = 5 #valor adequado é 5
    outputs     = carpet_plot(problem, number_of_points, 0, 0)  #run carpet plot, suppressing default plots
    inputs      = outputs.inputs
    objective   = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS   = plt.contourf(inputs[0,:],inputs[1,:], objective, 20, linewidths=2)
    cbar = plt.colorbar(CS)
    
    cbar.ax.set_ylabel('fuel burn (kg)')
    CS_const = plt.contour(inputs[0,:],inputs[1,:], constraints[0,:,:])
    plt.clabel(CS_const, inline=1, fontsize=10)
    cbar = plt.colorbar(CS_const)
    cbar.ax.set_ylabel('fuel margin')
    
    plt.xlabel('Chord root (inches)')
    plt.ylabel('tapper')
    
    plt.legend(loc='upper left')  
#    plt.show(block=True)    
    
    return

if __name__ == '__main__':
    tic = time.time()
    main()
    toc = time.time()
    print('\n\n')
    print (toc-tic, ' sec Elapsed')
    print ((toc-tic)/60, 'min Elapsed')
    tik_tok = (toc-tic)
    tik_tok_m = (toc-tic)/60
    
    file = open('results.txt', 'a')
    file.write('Tik tok = ' + str(tik_tok) + ' s')
    file.write('\nTik tok = ' + str(tik_tok_m) + ' min \n\n')
    file.close()




