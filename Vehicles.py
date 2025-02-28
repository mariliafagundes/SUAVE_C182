# Vehicles.py

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import SUAVE
from SUAVE.Core import Units
from SUAVE.Core import (
    Data, Container,
)
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import compute_span_location_from_chord_length
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.datcom import datcom
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions.trapezoid_ac_x import trapezoid_ac_x
from SUAVE.Methods.Propulsion import propeller_design
#from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

def base_setup():

    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Cessna_182'    
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

#     mass properties 
    Fuel_masss  = 237.    * Units.kg
    Payload     = 262.    * Units.kg
    # Empty_original = 907. * Units.kg
    Empty_original = 907 * Units.kg
    GTOW        =    Fuel_masss +  Payload + Empty_original # = 907 (empty) + 237 (fuel) + 262 (payload) = 1406. kg
    # GTOW        =    1351.92681156901                      # = 907 (empty) + 237 (fuel) + 262 (payload) = 1406. kg
    

    vehicle.mass_properties.max_takeoff               = GTOW
    vehicle.mass_properties.operating_empty           = Empty_original  #907,1847kg
    vehicle.mass_properties.takeoff                   = GTOW                   #1406.136 kg
    vehicle.mass_properties.max_zero_fuel             = GTOW - Fuel_masss # 1169 kg
    vehicle.mass_properties.cargo                     =  0.
    vehicle.mass_properties.landing                   = Empty_original + Payload  #1338,097
    vehicle.mass_properties.payload                   = Payload

    # envelope properties                       
    vehicle.envelope.ultimate_load = 2.5
    vehicle.envelope.limit_load    = 1.0
    
    vehicle.systems.control = "not powered"
    vehicle.systems.accessories       = "business"
	
    
    cruise_speed                    = 114.206055161447 * Units.knot     #145 knot/s? = 67.08648 ,/s
    altitude                        = 5500.0 * Units.m	# Roskan
    atmo                            = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream                      = atmo.compute_values (0.)
    freestream0                     = atmo.compute_values (altitude)
    # mach_number                     = (cruise_speed/freestream.speed_of_sound)[0][0]
    mach_number = 0.201     # real 
    vehicle.design_dynamic_pressure             = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number                  =  mach_number    
	
	
    # basic parameters
    vehicle.passengers              = 2.  #including pilot                           
    vehicle.reference_area          = 174. * Units.feet**2 # Wing gross area in square meters
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12 #NACA 2412
    wing.chords.root             = 66. * Units.inches
    cr = wing.chords.root
    wing.chords.tip              = 50. * Units.inches
    wing.taper                   = wing.chords.tip/wing.chords.root # I inverted to fix it
    taper = wing.taper
    wing.span_efficiency         = 0.9     #CHANGE IT IF NECESSARY
    wing.spans.projected         = 36. * Units.feet
    wing.chords.mean_aerodynamic = 2/3*cr*(1+taper+taper**2)/(1+taper) #pressure acting point; Gudmundsson pg 310
    wing.areas.reference         = vehicle.reference_area
    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root             = 3.0 * Units.degrees	# Copied of Cessna_172
    wing.twists.tip              = 1.5 * Units.degrees	# Copied of Cessna_172
    wing.origin                    = [80.* Units.inches,0,35.*Units.inches]
    wing.aerodynamic_center        = [22.* Units.inches,0,0]
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.dynamic_pressure_ratio  = 1.0 #ERASE THIS AFTER
    
    # ------------------------------------------------------------------
    #   Flaps
    # ------------------------------------------------------------------
    flap                                        = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                                    = 'flap' 
    flap.span_fraction_start                    = 0.05 
    flap.span_fraction_end                      = 0.45    
    flap.deflection                             = 1.0 * Units.deg
    flap.chord_fraction                         = 0.20    
    wing.append_control_surface(flap) 
    # wing.flaps.type       = 'single_slotted'
    
    # add to vehicle
    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.08
    wing.span_efficiency         = 0.9
    wing.areas.reference         = 5800 * Units.inches**2 #table says 3182.4 in^2
    wing.spans.projected         = 11.6667 * Units.feet
    wing.chords.root             = 53.  * Units.inches
    wing.chords.tip              = 29.9 * Units.inches
    cr = wing.chords.root
    wing.taper                   = wing.chords.tip/wing.chords.root
    taper = wing.taper
    wing.chords.mean_aerodynamic = 2/3*cr*(1+taper+taper**2)/(1+taper) #pressure acting point; Gudmundsson pg 310
    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [246.* Units.inches,0,0 ] 
    wing.aerodynamic_center      = [20.* Units.inches,0,0] 
    wing.vertical                = False 
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9  
    
    # add to vehicle
    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    
    
    wing.sweeps.quarter_chord    = 30. * Units.deg
    wing.thickness_to_chord      = 0.08
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 73 * Units.inches
    wing.chords.root             = 66. * Units.inches
    wing.chords.tip              = 27. * Units.inches
    cr = wing.chords.root
    wing.taper                   = wing.chords.root/wing.chords.tip
    taper = wing.taper
    wing.chords.mean_aerodynamic = 2/3*cr*(1+taper+taper**2)/(1+taper) #pressure acting point; Gudmundsson pg 310
    wing.areas.reference         = (wing.chords.root + wing.chords.tip)*wing.spans.projected/2
    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [237.* Units.inches,0,0]
    wing.aerodynamic_center      = [20.* Units.inches,0,0]
    
    wing.vertical                = True 
    wing.symmetric               = False
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
        
    # add to vehicle
    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    # CESSNA_172.py CODE:
    
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
    #   Landing gear: check it up later, it is related to the parasite drag
    # ------------------------------------------------------------------  
    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = SUAVE.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = SUAVE.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches #guess based on picture
    nose_gear.strut_length                      = 6. * Units.inches 
                                                
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear
                                                
    #add to vehicle                             
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
    fuel.mass_properties.mass                   = 522 *Units.lbs
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
    net.rated_speed                             = 2000. * Units.rpm
    net.rated_power                             = 230.  * Units.hp
    net.areas.wetted                            = 0.01
    # net = propeller_design(net)
    
    # Component 2 the PROPELLER
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller() 


    prop.number_blades       = 3.0
    prop.freestream_velocity = 135.*Units['mph']    
    prop.angular_velocity    = 1300.  * Units.rpm # 2400      !!!!!!!
    prop.tip_radius          = 82./2. * Units.inches
    prop.hub_radius          = 8.     * Units.inches
    prop.design_Cl           = 0.8
    prop.design_altitude     = 12000. * Units.feet
    prop.design_thrust       = 800.  
    prop.origin              = [[2.,2.5,0.784]]  #  prop influence               
    prop.symmetry            = True
    prop                     = propeller_design(prop)    
    net.propeller            = prop    
    # motor.propeller_Cp       = 0.55  #remove if necessary (copied by some website)

                                   
    # Component 1 the engine                    
    net.engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    net.engine.sea_level_power                  = 180. * Units.horsepower   #!!!!!!
    net.engine.flat_rate_altitude               = 0.0
    net.engine.speed                            = 2000. * Units.rpm         #!!!!
    net.engine.power_specific_fuel_consumption  = 0.620166 #lb/h-hp (inside Internal Combustion Engine, it will be converted)
    
    
    # add the network to the vehicle
    vehicle.append_component(net) 

    # #find uninstalled avionics weight: TEST TO TAKE IT OFF
    # Wuav                                        = 2. * Units.lbs
    # avionics                                    = SUAVE.Components.Energy.Peripherals.Avionics()
    # avionics.mass_properties.uninstalled        = Wuav
    # vehicle.avionics                            = avionics     

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
   
    # Component 9 Miscellaneous Systems 
    sys = SUAVE.Components.Systems.System()
    sys.mass_properties.mass = 5 # kg
  
    return vehicle
  

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                                    = SUAVE.Components.Configs.Config.Container() 
    base_config                                                = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag                                            = 'base'
    configs.append(base_config)
    
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------ 
    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'takeoff' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.V2_VS_ratio                                         = 1.21
    config.maximum_lift_coefficient                            = 2.
    
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Cruise Configuration UNPRINT THIS:
    # ------------------------------------------------------------------ 
    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'cruise' 
    configs.append(config)
    
    
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'landing' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.Vref_VS_ratio                                       = 1.23
    config.maximum_lift_coefficient                            = 2.
                                                               
    configs.append(config)
    
    
    # done!
    return configs
