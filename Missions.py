# Missions.py
# 
# Created:  Mar 2016, M. Vegh
# Modified: Aug 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units

import numpy as np

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
    
def setup(analyses):
    
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------
    base_mission = base(analyses)
    missions.base = base_mission 
 
    return missions  
    
def base(analyses):
    
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
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Speed_Constant_Rate()
    # segment.tag = "climb_1"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.base )

    # # define segment attributes
    # segment.atmosphere     = atmosphere
    # segment.planet         = planet

    # segment.altitude_start = 0.0   * Units.ft
    # segment.altitude_end   = 1000 * Units.ft
    # segment.air_speed      = 75. * Units.knots
    # segment.climb_rate     = 1340. * Units['ft/min']

    # # add to misison
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Speed_Constant_Rate()
    # segment.tag = "climb_2"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 3000 * Units.ft
    # segment.air_speed    = 100. * Units.knots
    # segment.climb_rate   = 1000. * Units['ft/min']

    # # add to mission
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed, Constant Climb Rate
    # ------------------------------------------------------------------

    # segment = Segments.Climb.Constant_Speed_Constant_Rate()
    # segment.tag = "climb_3"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 18000. * Units.ft
    # segment.air_speed    = 100.0  * Units.knots
    # segment.climb_rate   = 800. * Units['ft/min']

    # # add to mission
    # mission.append_segment(segment) 
    
    
    # ------------------------------------------------------------------
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude()
    segment.tag = "cruise"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere = atmosphere
    segment.planet     = planet

    segment.altitude_start = 18000. * Units.ft
    segment.altitude_end = 18000. * Units.ft
    segment.altitude = 18000. * Units.ft
    segment.air_speed  = 114. * Units.knots
    segment.distance   = 1477.24412425965 * Units.km

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate()
    # segment.tag = "descent_1"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 3000 * Units.ft
    # segment.air_speed    = 100. * Units.knots
    # segment.descent_rate   = 1000. * Units['ft/min']

    # # add to mission
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate()
    # segment.tag = "descent_2"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise_spoilers )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end   = 1000 * Units.ft
    # segment.air_speed      = 75. * Units.knots
    # segment.descent_rate     = 1340. * Units['ft/min']

    # # append to mission
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate()
    # segment.tag = "descent_3"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 0.0   * Units.km
    # segment.air_speed    = 75.0 * Units.knots
    # segment.descent_rate = 1200. * Units['ft/min']

    # # append to mission
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
    return mission

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    import vehicles
    import analyses
    
    vehicles = vehicles.setup()
    analyses = analyses.setup(vehicles)
    missions = setup(analyses)
    
    vehicles.finalize()
    analyses.finalize()
    missions.finalize()
    
    missions.base.evaluate()