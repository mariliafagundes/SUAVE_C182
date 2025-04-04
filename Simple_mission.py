import numpy as np
import matplotlib.pyplot as plt

import SUAVE
from SUAVE.Core import Units, Data

from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_segmented_planform import wing_segmented_planform
from SUAVE.Plots.Performance import *
from SUAVE.Components import Lofted_Body_Segment
from SUAVE.Analyses.Weights import Weights_Transport

#from SUAVE.Input_Output.OpenVSP import write

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
     
    # Define internal combustion engine from Cessna Regression Aircraft 
    vehicle    = vehicle_setup()

    #write(vehicle,'C172')

    # Setup analyses and mission
    analyses = base_analysis(vehicle)
    analyses.finalize()
    mission  = mission_setup(analyses,vehicle)
    
    # evaluate
    results = mission.evaluate()
    
    plot_mission(results)

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------        
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Cessna_182'
                                                
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    Fuel_masss  = 282.    * Units.kg
    Payload     = 256.    * Units.kg
    Empty_original = 873 * Units.kg
    GTOW        =    Fuel_masss +  Payload + Empty_original 
    # GTOW        =    1411                   
    

    vehicle.mass_properties.max_takeoff               = GTOW
    vehicle.mass_properties.operating_empty           = Empty_original  
    vehicle.mass_properties.takeoff                   = GTOW                 
    vehicle.mass_properties.max_zero_fuel             = GTOW - Fuel_masss
    vehicle.mass_properties.cargo                     =  0.
    vehicle.mass_properties.landing                   = Empty_original + Payload  
    vehicle.mass_properties.payload                   = Payload
                                               
    # envelope properties                       
    vehicle.envelope.ultimate_load              = 5.7
    vehicle.envelope.limit_load                 = 3.8
                                                
    # basic parameters                          
    vehicle.reference_area                      = 174. * Units.feet**2       
    vehicle.passengers                          = 4

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'    
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 174. * Units.feet**2
    wing.spans.projected                        = 36.  * Units.feet + 1. * Units.inches
    wing.chords.root                            = 66. * Units.inches
    wing.chords.tip                             = 45. * Units.inches
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[80.* Units.inches,0,36.75* Units.inches]]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = False
    wing.dynamic_pressure_ratio                 = 1.0 

    # # Wing Segments
    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'Root'
    # segment.percent_span_location         = 0.0
    # segment.twist                         = 3. * Units.deg
    # segment.root_chord_percent            = 1.
    # segment.thickness_to_chord            = 0.12
    # segment.dihedral_outboard             = 2.5 * Units.degrees
    # segment.sweeps.quarter_chord          = 0
    # wing.append_segment(segment)

    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'Break'
    # segment.percent_span_location         = 0.531177829
    # segment.twist                         = 2. * Units.deg
    # segment.root_chord_percent            = 1.0
    # segment.thickness_to_chord            = 0.12
    # segment.dihedral_outboard             = 5 * Units.degrees
    # segment.sweeps.quarter_chord          = -3. * Units.degrees
    # wing.append_segment(segment)

    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'Tip'
    # segment.percent_span_location         = 1.
    # segment.twist                         = 1. * Units.degrees
    # segment.root_chord_percent            = 0.67
    # segment.thickness_to_chord            = 0.12
    # segment.dihedral_outboard             = 0.
    # segment.sweeps.quarter_chord          = 0.
    # wing.append_segment(segment)

    # wing = wing_segmented_planform(wing)


      
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------                                
    wing                                        = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag                                    = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord                   = 19.5 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.spans.projected                        = 135.  * Units.inches
    wing.areas.reference                        = 5500  * Units.inches**2
    wing.chords.root                            = 55. * Units.inches
    wing.chords.tip                             = 28. * Units.inches
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = (wing.spans.projected**2)/ wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[253.* Units.inches,0,0]]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = False 
    wing.dynamic_pressure_ratio                 = 0.9
    
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 
    
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    wing                                        = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag                                    = 'vertical_stabilizer' 
    wing.sweeps.quarter_chord                   = 48. * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 3500. * Units.inches**2
    wing.spans.projected                        = 56.   * Units.inches
    wing.chords.root                            = 64. * Units.inches
    wing.chords.tip                             = 30. * Units.inches
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[240.* Units.inches,0,0]]
    wing.vertical                               = True 
    wing.symmetric                              = False
    wing.t_tail                                 = False 
    wing.dynamic_pressure_ratio                 = 1.0
    
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Strut
    # ------------------------------------------------------------------

    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'strut' 
    wing.sweeps.quarter_chord                   = 0. * Units.deg
    wing.thickness_to_chord                     = 0.4
    wing.areas.reference                        = 660. * Units.inches**2
    wing.spans.projected                        = 200.   * Units.inches
    wing.chords.root                            = 6. * Units.inches
    wing.chords.tip                             = 6. * Units.inches
    wing.chords.mean_aerodynamic                = 6. * Units.inches 
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[80.* Units.inches,20.* Units.inches,-17.* Units.inches]]
    wing.dihedral                               = 30.0 * Units.degrees
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.t_tail                                 = False 
    wing.dynamic_pressure_ratio                 = 1.0

    # add to vehicle
    vehicle.append_component(wing)
    


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.number_coach_seats                 = 4.       
    fuselage.tag                                = 'fuselage'    
    fuselage.differential_pressure              = 8*Units.psi                   # Maximum differential pressure
    fuselage.width                              = 42.         * Units.inches     # Width of the fuselage
    fuselage.heights.maximum                    = 62. * Units.inches    # Height of the fuselage
    fuselage.lengths.total                      = 329.37 * Units.inches            # Length of the fuselage
    fuselage.lengths.empennage                  = 161. * Units.inches
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.structure                  = fuselage.lengths.total-fuselage.lengths.empennage 
    fuselage.mass_properties.volume             = .4*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.) #try this as approximation
    fuselage.mass_properties.internal_volume    = .3*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.)
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches
    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.effective_diameter                 = 50. * Units.inches
    fuselage.areas.side_projected = fuselage.effective_diameter*fuselage.lengths.total

    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  
    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = SUAVE.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = SUAVE.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches 
    nose_gear.strut_length                      = 6. * Units.inches 
                                                
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear
                                                
    # add to vehicle                             
    vehicle.landing_gear                        = landing_gear


    # ------------------------------------------------------------------
    #   Fuel
    # ------------------------------------------------------------------    
    # define fuel weight needed to size fuel system
    fuel                                        = SUAVE.Attributes.Propellants.Aviation_Gasoline()
    fuel.mass_properties                        = SUAVE.Components.Mass_Properties() 
    fuel.number_of_tanks                        = 1.
    fuel.origin                                 = wing.origin
    fuel.internal_volume                        = fuel.mass_properties.mass/fuel.density #all of the fuel volume is internal
    fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    fuel.mass_properties.mass                   = 621 *Units.lbs
    vehicle.fuel                                = fuel

    # ------------------------------------------------------------------
    #   Piston Propeller Network
    # ------------------------------------------------------------------    
    
    # build network
    net                                         = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    net.tag                                     = 'internal_combustion'
    net.number_of_engines                       = 1.
    net.nacelle_diameter                        = 42 * Units.inches
    net.engine_length                           = 0.01 * Units.inches
    net.areas                                   = Data()
    net.rated_speed                             = 2400. * Units.rpm
    net.rated_power                             = 230.  * Units.hp
    net.areas.wetted                            = 0.01
    # net = propeller_design(net)
    
    # Component 2 the PROPELLER
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller() 


    prop.number_blades       = 3.0
    prop.freestream_velocity = 135.*Units['mph']    
    prop.angular_velocity    = 1300.  * Units.rpm 
    prop.tip_radius          = 82./2. * Units.inches
    prop.hub_radius          = 8.     * Units.inches
    prop.design_Cl           = 0.8
    prop.design_altitude     = 12000. * Units.feet
    prop.design_thrust       = 800.  
    prop.origin              = [[2.,2.5,0.784]]  #  prop influence               
    prop.symmetry            = True
    prop                     = propeller_design(prop)    
    net.propeller            = prop    

                                   
    # Component 1 the engine                    
    net.engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    net.engine.sea_level_power                  = 230. * Units.horsepower   
    net.engine.flat_rate_altitude               = 0.0
    net.engine.speed                            = 2400. * Units.rpm       
    net.engine.power_specific_fuel_consumption  = 0.65 #lb/h-hp (inside Internal Combustion Engine, it will be converted)
    
    
    # add the network to the vehicle
    vehicle.append_component(net)  


    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere =  SUAVE.Analyses.Atmospheric.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    atmosphere=SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    planet = SUAVE.Attributes.Planets.Earth()
    
    #------------------------------------------------------------------
    #  First Climb Segment: Constant Speed, Constant Throttle
    #------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_1"

    # connect vehicle configuration
    segment.analyses.extend( analyses )

    # define segment attributes
    segment.atmosphere     = atmosphere
    segment.planet         = planet

    segment.altitude_start = 0.0
    segment.altitude_end   = 800.0 * Units.ft
    segment.air_speed      = 70.0 * Units.knots
    segment.climb_rate = 6.0 *Units.knots
    print('first climb segment',segment.climb_rate)

    # add to misison
    mission.append_segment(segment)
    
    
    #------------------------------------------------------------------
    #  Second Climb Segment: Constant Speed, Constant Throttle
    #------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_2"

    # connect vehicle configuration
    segment.analyses.extend( analyses )

    # define segment attributes
    segment.atmosphere     = atmosphere
    segment.planet         = planet

    segment.altitude_start = 800.0 * Units.ft
    segment.altitude_end   = 3000. * Units.ft
    segment.air_speed      = 80.0 * Units.knots
    segment.climb_rate = 5.5 *Units.knots
    print('second climb segment',segment.climb_rate)


    # add to misison
    mission.append_segment(segment)
    
    #------------------------------------------------------------------
    #  Third Climb Segment: Constant Speed, Constant Throttle
    #------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_3"

    # connect vehicle configuration
    segment.analyses.extend( analyses )

    # define segment attributes
    segment.atmosphere     = atmosphere
    segment.planet         = planet

    segment.throttle       = 1.0
    segment.altitude_start = 3000.0 * Units.ft
    segment.altitude_end   = 8000.0 * Units.ft    
    segment.air_speed      = 100.0 * Units.knots
    segment.climb_rate = 4.0 *Units.knots
    print('third climb segment',segment.climb_rate)
    #segment.climb_rate =  9.0 * Units.knots


    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses )

    segment.altitude   = 8000. * Units.ft 
    segment.air_speed  = 130 *Units.knot    
    segment.distance   = 1500 * Units.km	
    segment.rpm = 2400 * Units.rpm
    
    ones_row                                        = segment.state.ones_row   
    segment.state.numerics.number_control_points    = 16
    segment.state.unknowns.throttle                 = 1 * ones_row(1) 


    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip 

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  First Descent Segment: Constant Speed, Constant Rate
    #------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_1"

    # connect vehicle configuration
    segment.analyses.extend( analyses )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet
    segment.altitude_start = 8000.0 * Units.ft
    segment.altitude_end = 4000.  * Units.ft
    segment.air_speed    = 100.0 * Units.knots
    segment.descent_rate =  4.0 *Units.knots

    
    # add to mission
    mission.append_segment(segment)
    
    
    #------------------------------------------------------------------
    #  Second Descent Segment: Constant Speed, Constant Rate
    #------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_2"

    # connect vehicle configuration
    segment.analyses.extend( analyses )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet
    segment.altitude_start = 4000.0 * Units.ft
    segment.altitude_end = 1000.  * Units.ft
    segment.air_speed    = 90.0 * Units.knots
    segment.descent_rate =  5.0 *Units.knots

    # add to mission
    mission.append_segment(segment)
    
    
    #------------------------------------------------------------------
    #  Third Descent Segment: Constant Speed, Constant Rate
    #------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_3"

    # connect vehicle configuration
    segment.analyses.extend( analyses )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet
    segment.altitude_start = 1000.0 * Units.ft
    segment.altitude_end = 0.  * Units.ft
    segment.air_speed    = 80.0 * Units.knots
    segment.descent_rate =  5.2 *Units.knots

    # add to mission
    mission.append_segment(segment)


    return mission


def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    ## ------------------------------------------------------------------
    ##  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    
    # Calculate extra drag from landing gear:
    
    main_wheel_width  = 4. * Units.inches
    main_wheel_height = 12. * Units.inches
    nose_gear_height  = 10. * Units.inches
    nose_gear_width   = 4. * Units.inches
    
    total_wheel       = 2*main_wheel_width*main_wheel_height + nose_gear_width*nose_gear_height
    
    main_gear_strut_height = 2. * Units.inches
    main_gear_strut_length = 24. * Units.inches
    nose_gear_strut_height = 12. * Units.inches
    nose_gear_strut_width  = 2. * Units.inches
    
    total_strut = 2*main_gear_strut_height*main_gear_strut_length + nose_gear_strut_height*nose_gear_strut_width
    
    # total drag increment area
    drag_area = 1.4*( total_wheel + total_strut)
    
    
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero() 
    aerodynamics.geometry                            = vehicle
    aerodynamics.settings.drag_coefficient_increment = 1.0*drag_area/vehicle.reference_area
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses


# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)  

    return



# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    
    main()
    plt.show()

