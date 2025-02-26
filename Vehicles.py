# Vehicles.py
# 
# Created:  Feb. 2016, M. Vegh
# Modified: Aug. 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import SUAVE
from SUAVE.Core import Units
from SUAVE.Core import (
    Data, Container,
)
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform
from SUAVE.Methods.Propulsion import propeller_design
from copy import deepcopy

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

    Fuel_mass  = 250.    * Units.kg
    Payload     = 537.    * Units.kg
    Empty_original = 872 * Units.kg
    GTOW        =    Fuel_mass +  Payload + Empty_original 

    vehicle.mass_properties.max_takeoff               = GTOW   # kg
    vehicle.mass_properties.operating_empty           = Empty_original   # kg
    vehicle.mass_properties.takeoff                   = GTOW   # kg
    vehicle.mass_properties.max_zero_fuel             = GTOW - Fuel_mass   # kg
    vehicle.mass_properties.max_payload               = Payload   # kg
    vehicle.mass_properties.max_fuel                  = Fuel_mass   # kg
    vehicle.mass_properties.cargo                     =     0.0  # kg
    vehicle.mass_properties.landing                   = Empty_original + Payload # kg

    # envelope properties
    vehicle.envelope.ultimate_load = 2.5
    vehicle.envelope.limit_load    = 1.5

    # basic parameters
    vehicle.reference_area         = 174. *Units.feet**2
    vehicle.passengers             = 2
    vehicle.systems.control        = "not powered"
    vehicle.systems.accessories    = "business"


    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                         = SUAVE.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = 174. * Units.feet**2
    wing.aspect_ratio            = 7.32
    wing.chords.root             = 66 * Units.inches
    wing.chords.tip              = 20 * Units.inches
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.303
    wing.dihedral                = 5.00 * Units.deg
    wing.spans.projected         = 36.089 * Units.feet
    wing.origin                  = [[80.* Units.inches,0,35.*Units.inches]]
    wing.vertical                = False
    wing.symmetric               = True       
    wing.high_lift               = True
    wing.areas.exposed           = 0.80 * wing.areas.wetted        
    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 1.5 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.0
    
    # control surfaces -------------------------------------------
    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span_fraction_start   = 0.05
    flap.span_fraction_end     = 0.45
    flap.deflection            = 1.0 * Units.deg 
    flap.chord_fraction        = 0.20   
    flap.configuration_type    = 'single_slotted'
    wing.append_control_surface(flap)   
    
    wing                         = wing_planform(wing)

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    wing.areas.reference         = 3768.6775 * Units.inches**2
    wing.aspect_ratio            = 5.5
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.564
    wing.dihedral                = 2 * Units.degrees
    wing.origin                  = [[246.* Units.inches,0,0 ]]
    wing.vertical                = False
    wing.symmetric               = True       
    wing.high_lift               = False  
    wing                         = wing_planform(wing)
    wing.areas.exposed           = 0.9 * wing.areas.wetted 
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 0.90

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'
    wing.areas.reference         = 3394.5 * Units.inches**2
    wing.aspect_ratio            =  3.7
    wing.sweeps.quarter_chord    = 35. * Units.deg
    wing.thickness_to_chord      = 0.10
    wing.taper                   = 0.41
    wing.dihedral                = 0.00
    wing.origin                  = [[237.* Units.inches,0,0]]
    wing.vertical                = True
    wing.symmetric               = False       
    wing.high_lift               = False
    wing                         = wing_planform(wing)
    wing.areas.exposed           = 0.9 * wing.areas.wetted
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.00
    
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag    = 'fuselage'
    fuselage.origin = [[0,0,0]]
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 0
    fuselage.seat_pitch            = 30. * Units.inches

    fuselage.fineness.nose         = 1.6
    fuselage.fineness.tail         = 2.

    fuselage.lengths.nose          = 67.5 * Units.inches
    fuselage.lengths.tail          = 167.5 * Units.inches
    fuselage.lengths.cabin         = 114 * Units.inches
    fuselage.lengths.total         = 349 * Units.inches
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.

    fuselage.width                 = 42. * Units.inches

    fuselage.heights.maximum       = 62. * Units.inches    
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches

    fuselage.areas.side_projected  = 17450. * Units.inches**2
    fuselage.areas.wetted          = 30000. * Units.inches**2.
    fuselage.areas.front_projected = fuselage.width* fuselage.heights.maximum

    fuselage.effective_diameter    = 50. * Units.inches

    fuselage.differential_pressure = 10**5 * Units.pascal    # Maximum differential pressure

    # add to vehicle
    vehicle.append_component(fuselage)  
    

    # ------------------------------------------------------------------
    #   Piston Propeller Network
    # ------------------------------------------------------------------    
    
    # build network
    net                                         = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    net.tag                                     = 'internal_combustion'
    net.number_of_engines                       = 1.
    net.identical_propellers                    = True
    net.nacelle_diameter                        = 42 * Units.inches
    net.engine_length                           = 0.01 * Units.inches
    net.areas                                   = Data()
    net.rated_speed                             = 2000. * Units.rpm
    net.rated_power                             = 230.  * Units.hp
    net.areas.wetted                            = 0.01

    # Component 1 the engine                    
    engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    engine.sea_level_power                  = 180. * Units.horsepower   #!!!!!!
    engine.flat_rate_altitude               = 0.0
    engine.speed                            = 2000. * Units.rpm         #!!!!
    engine.power_specific_fuel_consumption  = 0.620166 #lb/h-hp (inside Internal Combustion Engine, it will be converted)
    
    
    # add the network to the vehicle
    net.engines.append(engine)

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
    prop.origin              = [[2.,2.5,0.784]]  #  prop influence               
    prop.symmetry            = True
    prop.design_power            = .64 * 180. * Units.horsepower
    prop.variable_pitch          = False
    prop                     = propeller_design(prop)    
    net.propellers.append(prop)   

    # add the network to the vehicle
    vehicle.append_component(net)       
    
    fuel                                  = SUAVE.Components.Physical_Component()
    vehicle.fuel                          = fuel
    fuel.mass_properties.mass             = vehicle.mass_properties.max_takeoff-vehicle.mass_properties.max_fuel
    fuel.origin                           = vehicle.wings.main_wing.mass_properties.center_of_gravity     
    fuel.mass_properties.center_of_gravity= vehicle.wings.main_wing.aerodynamic_center
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    configs.append(config)
    
    config.maximum_lift_coefficient = 1.2
    
    # ------------------------------------------------------------------
    #   Cruise with Spoilers Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise_spoilers'

    configs.append(config)
    
    config.maximum_lift_coefficient = 1.2


    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.V2_VS_ratio = 1.21
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.Vref_VS_ratio = 1.23
    configs.append(config)   

    return configs
