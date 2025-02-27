# Procedure.py

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length
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

    ##########################################################################
                                # RANGE EVALUATION 
    ##########################################################################
    # Geometry unpacking
    MAC                      = vehicle.wings.main_wing.chords.mean_aerodynamic 
    AR                       = vehicle.wings.main_wing.aspect_ratio
    S_ref                    = vehicle.reference_area
    chord_root               = vehicle.wings.main_wing.chords.root
    span                     = vehicle.wings.main_wing.spans.projected
    taper                    = vehicle.wings.main_wing.taper


    # Mass unpacking
    GTOW                     = vehicle.mass_properties.takeoff
    W_0                      = GTOW
    Empty_weight             = vehicle.mass_properties.operating_empty #empty weight  
    design_takeoff_weight    = GTOW
    # Fuel_mass_total          = vehicle.fuel.mass_properties.mass 
    Fuel_mass_total          = 237 * Units.kg
    

    # payload                  = vehicle.passenger_weights.mass_properties.mass # number of passenger \times 225pounds = nmber*102kg
    zero_fuel_weight         = GTOW - Fuel_mass_total
    wing_weight              = vehicle.weight_breakdown.wing
    final_weight_real        = results.base.segments[-1].conditions.weights.total_mass[-1]
    W_4                      = zero_fuel_weight                                #Weight at the final of mission; It must be equal to design_landing_weight
    
    
    Massa_breakdown          = vehicle.weight_breakdown
    
    # Weight ratios over the mission
    W1_W0                    = 0.97                                            # Warmup and takeoff fuel consumption
    W2_W1                    = 0.985                                           # Climb fuel consumption
    W4_W3                    = 0.995                                           # Descent fuel consumption
    
    
    # Weights evaluation
    # W_0 = 1406.136347
    # W_3                       = W_4 / W4_W3                                  # Final cruise weight
    W_2                       = W1_W0 * W2_W1 * W_0                            # Initial cruise weight
    Fuel_mass_cruise          = 168. * Units.kg                                # Fuel during cruise = Cessna 182 original
    W_3                       = W_2 - Fuel_mass_cruise
    # W_3                       = 1175.011
    


    #Atmosphere evaluation at 5500 m
    altitude_5500 = 5500 * Units.m  
    # altitude_5500 = 0 * Units.m  
    atmosphere  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream_5500  = atmosphere.compute_values(altitude_5500)
    
    
    
    #now add to freestream data object
    freestream_5500.gravity     = 9.81
    rho_5500 = freestream_5500.density[0,0]
    mu_5500 = freestream_5500.dynamic_viscosity[0,0]
    a_5500 = freestream_5500.speed_of_sound[0,0]
    
    
    
    # CD0 evaluation
    V_estimated = 110 * Units.knot #An estimation for a first iteration of CD0
    Rey_estimated = rho_5500*MAC*V_estimated/mu_5500
    Mach_number = V_estimated/a_5500

    
    for segment in results.base.segments.values():
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        e_factor =  drag_breakdown.induced.efficiency_factor[0,0] #updated oswald efficiency factor
        
    K = 1/(np.pi*AR*e_factor)

    ref_condition                   = Data()
    ref_condition.mach_number       = Mach_number
    ref_condition.reynolds_number   = Rey_estimated 

    # cruise_configs  = vehicle.cruise
    cruise_configs  = nexus.vehicle_configurations.cruise
    
    analyses_CD0 = Data()
    analyses_CD0.configs = Data()
    analyses_CD0.configs.cruise = Data()
    analyses_CD0.configs.cruise = analyses.cruise
    CD0             = print_parasite_drag(ref_condition, cruise_configs, analyses_CD0,'C182_drag.dat' )


    #CL_ideal and Vel_ideal evaluation    
    CL_ideal    = np.sqrt(CD0/K)                                               # CL for best L/D
    V_ideal     = np.sqrt(2*W_2*9.81/(rho_5500*S_ref*CL_ideal))                     # Vel for best L/D
    
    
    # print('\n\n CD0 LOOP \n\n')
    for i in range(5):
        Rey_estimated = rho_5500*MAC*V_ideal/mu_5500
        Mach_number = V_ideal/a_5500
        ref_condition.mach_number       = Mach_number
        ref_condition.reynolds_number   = Rey_estimated 
        CD0 = print_parasite_drag(ref_condition, cruise_configs, analyses_CD0,'C182_drag.dat' )
        CL_ideal    = np.sqrt(CD0/K)                                               # CL for best L/D
        V_ideal     = np.sqrt(2*W_2*9.81/(rho_5500*S_ref*CL_ideal))                     # Vel for best L/D
        print('CD0', CD0)
        # print('CL_ideal', CL_ideal)
        # print('V_ideal', V_ideal)
        
    
    
    
    print('Rey_estimated', Rey_estimated)
    
    # Total drag evaluation
    CD_i        = K*CL_ideal**2
    CD_total    = CD_i + CD0
    L_D         = CL_ideal / CD_total
    
    
    D           = 1/2 * rho_5500 * V_ideal**2 * S_ref * CD_total               # Drag during the cruise
    
    
    # Power required and provided
    D_V         = D * V_ideal                                                  # Thrust power produced
    
    eta_p       = 0.80                                                         # Propeller efficiency (Raymer)
    P_provided  = D_V / eta_p                                                  # Power provided to the propeller
    Pbhp        = P_provided * 0.00134102                                      # Power provided, break horse power unit
    
    
    # Fuel consumption value (from Lycoming chart)
    Fuel_density        = 6.01                                                 # Avgas density [lb / u.s. gal]
    Fuel_consumption    = -0.00002 * Pbhp**2 + 0.0656 * Pbhp  + 3.8099         # Fuel consumption [u.s. gal / hr]
    C_bhp               = Fuel_consumption * Fuel_density / Pbhp               # Cbhp [lb/hr/bhp]
    C_power             = C_bhp * 0.453592 / 3600 / 745.7                      # Conversion from Cbhp to SI [kg/W-s]
    
    
    
    RANGE_VALUE         = eta_p * L_D / C_power * np.log(W_2 / W_3) / 9.81     # Range with 168 kg mass of fuel
    Range_objective = 1/RANGE_VALUE      


                                 # Optimization objective
    print('----------------------------------------------')
    print('RANGE VALUE', RANGE_VALUE/Units.km)
    print('\n') 

    print('span', span)
    print('chord_root', chord_root)
    print('taper', taper)
    print('Empty_weight', Empty_weight)
    print('CD0', CD0)
    print('Oswald factor e', e_factor)
    
    
    print('\n')
    
    
    
    print('Sref', S_ref)
    print('AR', AR)

    print('\n')


    print('W_0', W_0)
    print('W_2', W_2)
    print('W_3', W_3)
    print('W_2 / W_3',  W_2 / W_3 )

    print('\n')



    print('CL_ideal', CL_ideal)
    print('CD_total', CD_total)
    print('L_D', L_D)

    print('\n')


    print('C_bhp', C_bhp)
    print('C_power', C_power)

    # print('k', K)
    print('V_ideal', V_ideal / Units.knot)
    print('D ', D )
    print('D_V', D_V )
    print('P_provided', P_provided )
    print('Pbhp', Pbhp )
    

    # print('rho', rho_5500 )
    # print('Fuel_consumption', Fuel_consumption )
    print('Mach_number', Mach_number )
    # print('Sound speed', a_5500 / Units.knot )
    print('\n\n')
    
    
    ##########################################################################
                                # FULL MISSION REAL VALUE
    ##########################################################################
    Real_fuelburn                = design_takeoff_weight - results.base.segments['cruise'].conditions.weights.total_mass[-1] #Fuel burn

     
    final_weight = results.base.segments['cruise'].conditions.weights.total_mass[-1]
    # print('final_weight Real = ', final_weight)


    # TAKEOFF DISTANCE SETTING:
    Analyses_opt = SUAVE.Analyses.Analysis.Container()
    Analyses_opt.base = nexus.analyses.base
    Analyses_opt.missions = nexus.missions              #Esse esta OK


    # print('\n')
    engine_type = 'propeller'
    takeoff_dist =  estimate_take_off_field_length(vehicle, Analyses_opt, missions.airport, engine_type)
    # print('TAKEOFF dist = ', takeoff_dist)
    
    #parametrizacao da constraint:
    length_desired = 600. *Units.m
    takeoff_constraint = takeoff_dist - length_desired



    base_mission_fuelburn = Real_fuelburn

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
