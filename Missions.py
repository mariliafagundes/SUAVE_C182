# Missions.py

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
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    planet = SUAVE.Attributes.Planets.Earth()

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row


    
#        THE FOLLOWING SEGMENTS GROUND AND TAKEOFF ARE NOT NECESSARY TO THE REST OF THE CODY
###    # ------------------------------------------------------------------
###    #  Ground
###    # ------------------------------------------------------------------
###    #   Based in:   SUAVE/regression/scripts/segments/segment_test.py 
###
###    segment = Segments.Ground.Ground(base_segment)
###    segment.tag = "Ground"
###
###    segment.analyses.extend( analyses.takeoff )
###    segment.velocity_start           = 0.* Units.ft/Units.s
###    segment.velocity_end             = 50*1.2 * Units.knots   #Lecture_22_Flight_Testing_takeoff
###    segment.friction_coefficient     = 0.4
###    segment.state.unknowns.throttle  = 0.3 * ones_row(1)  
###    
###    # add to misison
###    mission.append_segment(segment)    
###    
###    
###    # # ------------------------------------------------------------------    
###    # #   Takeoff Segment
###    # # ------------------------------------------------------------------  
###    #   Based in:   SUAVE/regression/scripts/segments/segment_test.py 
###        
###    segment = Segments.Ground.Takeoff(base_segment)
###    segment.tag = "Takeoff"
###
###    segment.analyses.extend( analyses.takeoff )   #Lecture_22_Flight_Testing_takeoff
###    segment.velocity_start           = 0.* Units.ft/Units.s
###    segment.velocity_end             = 50*1.2 * Units.knots   #Lecture_22_Flight_Testing_takeoff
###    segment.friction_coefficient     = 0.04
####    segment.time                     = 25  # Lecture_22_Flight_Testing_takeoff
###    segment.state.unknowns.throttle  = 1.0 * ones_row(1)  
###    
###    
###    # add to misison
###    mission.append_segment(segment)
##    
    

#### EU COMENTEI OS CLIMBS E OS DESCENTS PARA VERIFICAR A CONVERGENCIA




#     # ------------------------------------------------------------------
#     #   First Climb Segment: Constant Speed, Constant Throttle
#     # ------------------------------------------------------------------

#     segment = Segments.Climb.Constant_Throttle_Constant_Speed()
#     segment.tag = "climb_1"

#     # connect vehicle configuration
#     segment.analyses.extend( analyses.takeoff )

#     # define segment attributes
#     segment.atmosphere     = atmosphere
#     segment.planet         = planet

#     segment.altitude_start = 0.1
#     segment.altitude_end   = 100 * Units.m
#     segment.air_speed      = 75.0 * Units.knots
#     segment.throttle       = 1.0
#     aux_1 = np.arctan(1/9) #Descent ramp 9x/1z; 6.34 deg.I'm suposing climb rate at the same value
#     segment.climb_rate =  segment.air_speed*np.sin(aux_1)#Descending from 1000 to 100, 22.367 ft/s
#     # print('climb_1_rate', segment.climb_rate/(Units.feets/Units.min), '[m/s]')

#     # add to misison
#     mission.append_segment(segment)
    
       
    
    
    
    
#     # ------------------------------------------------------------------
#     #   Second Climb Segment: Constant Speed, Constant Throttle
#     # ------------------------------------------------------------------

#     segment = Segments.Climb.Constant_Throttle_Constant_Speed()
#     segment.tag = "climb_2"

#     # connect vehicle configuration
#     segment.analyses.extend( analyses.takeoff )

#     # define segment attributes
#     segment.atmosphere     = atmosphere
#     segment.planet         = planet

#     segment.altitude_start = 100.0 * Units.m
#     segment.altitude_end   = 1000. * Units.ft
#     segment.air_speed      = 100.0 * Units.knots
#     segment.throttle       = 1.0
#     aux_1 = np.arctan(1/10) #Descent ramp 9x/1z; 6.34 deg.I'm suposing climb rate at the same value
#     segment.climb_rate =  segment.air_speed*np.sin(aux_1)#Descending from 1000 to 100, 22.367 ft/s
#     # print('climb_2_rate', segment.climb_rate/(Units.feets/Units.min), '[m/s]')


#     # add to misison
#     mission.append_segment(segment)
    
    
    
    
#     # ------------------------------------------------------------------
#     #   Third Climb Segment: Constant Speed, Constant Throttle
#     # ------------------------------------------------------------------

#     segment = Segments.Climb.Constant_Throttle_Constant_Speed()
#     segment.tag = "climb_3"

#     # connect vehicle configuration
#     segment.analyses.extend( analyses.takeoff )

#     # define segment attributes
#     segment.atmosphere     = atmosphere
#     segment.planet         = planet

#     segment.altitude_start = 1000.0 * Units.ft
# #    segment.altitude_end   = 5000 * Units.ft    # Service ceiling: 18100ft, 5517m
#     segment.altitude_end   = 18100. * Units.ft    # Service ceiling: 18100ft, 5517m
#     segment.air_speed      = 120.0 * Units.knots
#     segment.throttle       = 1.0
#     aux_1 = np.arctan(1/10) #Descent ramp 9x/1z; 6.34 deg.I'm suposing climb rate at the same value
#     segment.climb_rate =  segment.air_speed*np.sin(aux_1)#Descending from 1000 to 100, 22.367 ft/s
#     # print('climb_3_rate', segment.climb_rate/(Units.feets/Units.min), '[m/s]')


#     # add to misison
#     mission.append_segment(segment)
    
    
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )
    
    segment.altitude   = 5500 * Units.m  #This line was commented, so it is equal as the older one
    # segment.air_speed  = 114.206055161447 *Units.knot    #145 KCAS (Cessna_comparison_between_versions)
    segment.air_speed  = 113.5459778197371 *Units.knot    #145 KCAS (Cessna_comparison_between_versions)
    # segment.distance   = 1469.88537467077* Units.km	# Max range is 1695 km
    segment.distance   = 1477.244124259659 * Units.km	# Max range is 1695 km
    
    # add to mission
    mission.append_segment(segment)



    # # ------------------------------------------------------------------
    # #   First Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate()
    # segment.tag = "descent_1"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 1000.  * Units.ft
    # segment.air_speed    = 120.0 * Units.knots
    # aux_1 = np.arctan(1/10) #Descent ramp 9x/1z; 6.34 deg
    # segment.descent_rate =  segment.air_speed*np.sin(aux_1)#Descending from 1000 to 100, 22.367 ft/s
    # # print('desc_1_rate', segment.descent_rate/(Units.feets/Units.min), '[m/s]')

    
    # # add to mission
    # mission.append_segment(segment)
    
    
    # # ------------------------------------------------------------------
    # #   Second Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate()
    # segment.tag = "descent_2"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 100.  * Units.ft
    # segment.air_speed    = 100.0 * Units.knots
    # aux_1 = np.arctan(1/10) #Descent ramp 9x/1z; 6.34 deg
    # segment.descent_rate =  segment.air_speed*np.sin(aux_1)#Descending from 1000 to 100, 22.367 ft/s
    # # print('desc_2_rate', segment.descent_rate/(Units.feets/Units.min), '[m/s]')

    # # add to mission
    # mission.append_segment(segment)
    
    
    # # ------------------------------------------------------------------
    # #   Third Descent Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------

    # segment = Segments.Descent.Constant_Speed_Constant_Rate()
    # segment.tag = "descent_3"

    # # connect vehicle configuration
    # segment.analyses.extend( analyses.cruise )

    # # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    # segment.altitude_end = 0.  * Units.ft
    # segment.air_speed    = 75.0 * Units.knots
    # aux_1 = np.arctan(1/9) #Descent ramp 9x/1z; 6.34 deg
    # segment.descent_rate =  segment.air_speed*np.sin(aux_1)#Descending from 1000 to 100, 22.367 ft/s
    # # print('desc_3_rate', segment.descent_rate/(Units.feets/Units.min), '[m/s]')

    # # add to mission
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
