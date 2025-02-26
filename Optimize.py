# Optimize.py
# Created:  Feb 2016, M. Vegh
# Modified: Aug 2017, E. Botero
#           Aug 2018, T. MacDonald

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units, Data
import numpy as np
import Vehicles
import Analyses
import Missions
import Procedure
import Plot_Mission
import matplotlib.pyplot as plt
from SUAVE.Optimization import Nexus, carpet_plot
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    
    problem = setup()
    
    ## Base Input Values
    output = problem.objective()
    
    # Uncomment to view contours of the design space
    #variable_sweep(problem)
    
    # Uncomment for the first optimization
    output = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    print (output)    

    print(problem.summary)
    
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

    #   [ tag                   , initial,     lb , ub        , scaling , units ]
    problem.inputs = np.array([
    [ 'chord_root'   , 46.668395125745384 , 30. , 90. , 10. , 1*Units.inches ],
    [ 'taper'        , 0.9                , 0.3 , 0.1 , 1.  , 1*Units.less  ],
    [ 'wing_span'    , 40.6               , 30. , 45. , 10. , 1*Units.feet  ],
], dtype=object)

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([
        # [ 'fuel_burn',    100.,   Units.kg ],     
        # [ 'wing_weight',  100.,   Units.kg ],     
        [ 'Range_objective',        0.000001 ,  1*Units.less  ],     
        # [ 'L_D',        1,  Units.less ],     
        # [ 'W_0',        1000,  Units.kg ],     
        # [ 'AR',        1,  Units.kg ],     
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'Pbhp_lower_cons'                     , '>', 0.,  1.,     1*Units.less          ], # Lower boundary for Pbhp at 18k feet
        [ 'Pbhp_upper_cons'                     , '<', 0.,  1.,     1*Units.less    ], # Maximum power for Pbhp at 18k feet
        [ 'V_max_cons'                          , '<', 0.,  1.,     1*(Units.m / Units.s)   ], # Max design speed allowed
        [ 'AR_lower_cons'                       , '>', 0.,  1.,     1*Units.less          ], # Lower historical boundary for AR
        [ 'AR_upper_cons'                       , '<', 0.,  1.,     1*Units.less          ], # Upper historical boundary for AR
        [ 'S_ref_lower_cons'                    , '>', 0.,  1.,     1*Units.m**2          ], # Lower historical boundary for reference area
        [ 'S_ref_uper_cons'                     , '<', 0.,  1.,     1*Units.m**2          ], # Upper historical boundary for reference area
        # [ 'taper_constraint'                    , '<', 0., 1., Units.m], #base takeoff value = 674 m 
        # [ 'Margin_fuelburn' , '>', 0., 10., Units.kg], #fuel margin defined here as fuel 
        # [ 'takeoff_constraint'       , '<', 0., 1., Units.m], #base takeoff value = 674 m 

    ],dtype=object)
    
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
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.setup(nexus.analyses)
    
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
    number_of_points = 5
    outputs     = carpet_plot(problem, number_of_points, 0, 0)  #run carpet plot, suppressing default plots
    inputs      = outputs.inputs
    objective   = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS   = plt.contourf(inputs[0,:],inputs[1,:], objective, 20, linewidths=2,cmap='jet')
    cbar = plt.colorbar(CS)
    
    cbar.ax.set_ylabel('fuel burn (kg)')
    CS_const = plt.contour(inputs[0,:],inputs[1,:], constraints[0,:,:],cmap='jet')
    plt.clabel(CS_const, inline=1, fontsize=10)
    cbar = plt.colorbar(CS_const)
    cbar.ax.set_ylabel('fuel margin')
    
    plt.xlabel('Wing Area (m^2)')
    plt.ylabel('Cruise Altitude (km)')
    
    plt.show(block=True)    
    
    return

if __name__ == '__main__':
    main()
