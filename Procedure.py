# Procedure.py
# 
# Created:  Mar 2016, M. Vegh
# Modified: Aug 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Propulsion.compute_turbofan_geometry import compute_turbofan_geometry
#from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
#from SUAVE.Methods.Center_of_Gravity.compute_aircraft_center_of_gravity import compute_aircraft_center_of_gravity
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift.compute_max_lift_coeff import compute_max_lift_coeff
from SUAVE.Optimization.write_optimization_outputs import write_optimization_outputs

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing = simple_sizing
    
    # find the weights
    procedure.weights = weight
    # finalizes the data dependencies
    procedure.finalize = finalize
    
    # performance studies
    procedure.missions                   = Process()
    procedure.missions.design_mission    = design_mission

    # post process the results
    procedure.post_process = post_process
        
    return procedure

# ----------------------------------------------------------------------        
#   Target Range Function
# ----------------------------------------------------------------------    

# def find_target_range(nexus,mission):
    
#     segments = mission.segments
#     climb_1  = segments['climb_1']
#     climb_2  = segments['climb_2']
#     climb_3  = segments['climb_3']
#     climb_4  = segments['climb_4']
#     climb_5  = segments['climb_5']
  
#     descent_1 = segments['descent_1']
#     descent_2 = segments['descent_2']
#     descent_3 = segments['descent_3']

#     x_climb_1   = climb_1.altitude_end/np.tan(np.arcsin(climb_1.climb_rate/climb_1.air_speed))
#     x_climb_2   = (climb_2.altitude_end-climb_1.altitude_end)/np.tan(np.arcsin(climb_2.climb_rate/climb_2.air_speed))
#     x_climb_3   = (climb_3.altitude_end-climb_2.altitude_end)/np.tan(np.arcsin(climb_3.climb_rate/climb_3.air_speed))
#     x_climb_4   = (climb_4.altitude_end-climb_3.altitude_end)/np.tan(np.arcsin(climb_4.climb_rate/climb_4.air_speed))
#     x_climb_5   = (climb_5.altitude_end-climb_4.altitude_end)/np.tan(np.arcsin(climb_5.climb_rate/climb_5.air_speed))
#     x_descent_1 = (climb_5.altitude_end-descent_1.altitude_end)/np.tan(np.arcsin(descent_1.descent_rate/descent_1.air_speed))
#     x_descent_2 = (descent_1.altitude_end-descent_2.altitude_end)/np.tan(np.arcsin(descent_2.descent_rate/descent_2.air_speed))
#     x_descent_3 = (descent_2.altitude_end-descent_3.altitude_end)/np.tan(np.arcsin(descent_3.descent_rate/descent_3.air_speed))
    
#     cruise_range = mission.design_range-(x_climb_1+x_climb_2+x_climb_3+x_climb_4+x_climb_5+x_descent_1+x_descent_2+x_descent_3)
  
#     segments['cruise'].distance = cruise_range
    
#     return nexus

# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def design_mission(nexus):
    
    mission = nexus.missions.base
    # mission.design_range = 1500.*Units.nautical_miles
    # find_target_range(nexus,mission)
    results = nexus.results
    results.base = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    configs=nexus.vehicle_configurations
    base=configs.base
    
    #find conditions
    air_speed   = nexus.missions.base.segments['cruise'].air_speed 
    altitude    = nexus.missions.base.segments['cruise'].altitude_end
    atmosphere  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    
    freestream  = atmosphere.compute_values(altitude)
    freestream0 = atmosphere.compute_values(6000.*Units.ft)  #cabin altitude
    
    diff_pressure         = np.max(freestream0.pressure-freestream.pressure,0)
    fuselage              = base.fuselages['fuselage']
    fuselage.differential_pressure = diff_pressure 
    
    #now size engine
    mach_number        = air_speed/freestream.speed_of_sound
    
    #now add to freestream data object
    freestream.velocity    = air_speed
    freestream.mach_number = mach_number
    freestream.gravity     = 9.81
    
    conditions             = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()   #assign conditions in form for propulsor sizing
    conditions.freestream  = freestream
    
    for config in configs:
        config.wings.horizontal_stabilizer.areas.reference = (26.0/92.0)*config.wings.main_wing.areas.reference
            
        for wing in config.wings:
            
            wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)
            wing.areas.exposed  = 0.8 * wing.areas.wetted
            wing.areas.affected = 0.6 * wing.areas.reference
            
        fuselage              = config.fuselages['fuselage']
        fuselage.differential_pressure = diff_pressure 

    return nexus

# ----------------------------------------------------------------------        
#   Weights
# ----------------------------------------------------------------------    

def weight(nexus):
    vehicle=nexus.vehicle_configurations.base

    # weight analysis
    weights = nexus.analyses.base.weights.evaluate(method="SUAVE")
    weights = nexus.analyses.cruise.weights.evaluate(method="SUAVE")
    vehicle.mass_properties.breakdown = weights
    weights = nexus.analyses.landing.weights.evaluate(method="SUAVE")
    weights = nexus.analyses.takeoff.weights.evaluate(method="SUAVE")

    return nexus

# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
    return nexus         

# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack data
    vehicle                           = nexus.vehicle_configurations.base
    results                           = nexus.results
    summary                           = nexus.summary
    missions                          = nexus.missions.base
    analyses                          = nexus.analyses
    nexus.total_number_of_iterations +=1
    
    # Static stability calculations
    CMA = -10.
    for segment in results.base.segments.values():
        max_CMA = np.max(segment.conditions.stability.static.Cm_alpha[:,0])
        if max_CMA > CMA:
            CMA = max_CMA
            
    summary.static_stability = CMA
    
    #throttle in design mission
    max_throttle = 0
    for segment in results.base.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:,0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle
            
    summary.max_throttle = max_throttle

    eta_p = 0.8
    L_D = 11
    C_power = 0.00019
    W_2 = 1375. * Units.kilogram
    W_3 = 1175. * Units.kilogram

    RANGE_VALUE         = eta_p * L_D / C_power * np.log(W_2 / W_3) / 9.81     
    Range_objective = 1/RANGE_VALUE      


                                 # Optimization objective
    print('----------------------------------------------')
    print('RANGE VALUE', RANGE_VALUE/Units.km)
    print('\n') 

    ##########################################################################
                                # CONSTRAINTS
    ##########################################################################
    
    # taper_constraint = taper - 1.0
    
    # Margin_fuelburn             = final_weight_real - zero_fuel_weight         # Minimum fuel remaining
    Pbhp = 180. *Units.hp
    Pbhp_lower_cons             = Pbhp - 88                                    # Lower boundary for Pbhp at 18k feet
    Pbhp_upper_cons             = 200 - Pbhp                                    # Maximum power for Pbhp at 18k feet
    
    V_ideal = 160 *Units.knot
    V_max_cons                  = V_ideal - 145*Units.knot                     # Max design speed allowed

    AR = 7.32
    AR_lower_cons               = AR - 6                                       # Lower historical boundary for AR
    AR_upper_cons               = 11 - AR                                      # Upper historical boundary for AR

    S_ref = 174.4 * Units.feet**2
    S_ref_lower_cons            = S_ref - 150 * Units.feet**2                  # Lower historical boundary for reference area
    S_ref_uper_cons            = 400 - S_ref  * Units.feet**2                   # Upper historical boundary for reference area
    
    
    ##########################################################################
                                # SUMMARY
    ##########################################################################
    summary.Range_objective         = Range_objective

    summary.Pbhp_lower_cons             = Pbhp_lower_cons
    summary.Pbhp_upper_cons             = Pbhp_upper_cons

    summary.V_max_cons                  = V_max_cons

    summary.AR_lower_cons               = AR_lower_cons
    summary.AR_upper_cons               = AR_upper_cons

    summary.S_ref_lower_cons            = S_ref_lower_cons
    summary.S_ref_uper_cons             = S_ref_uper_cons
   
    return nexus    
