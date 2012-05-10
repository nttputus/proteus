from morse.builder import *

# Helicopter robot
RMax = Robot('rmax')
RMax.name = "RMAX"

GPS_001 = Sensor('gps')
# GPS_001.translate(z=0.6)
RMax.append(GPS_001)

CameraMain_002 = Sensor('video_camera')
CameraMain_002.translate(x=0.6, z=0.38)
CameraMain_002.rotate(x=-1.5708)
RMax.append(CameraMain_002)
CameraMain_002.properties(capturing = True,
                          cam_width = 128,
                          cam_height = 128,
                          cam_focal = 35.0000)

Gyroscope_001 = Sensor('gyroscope')
Gyroscope_001.translate(x=0.4, z=0.82)
RMax.append(Gyroscope_001)

Motion_Controller = Actuator('destination')
# Motion_Controller.translate(z=0.6)
RMax.append(Motion_Controller)

RMax.translate(x=5.0, y=0.0, z=9.0)

# Scene configuration
GPS_001.configure_mw('ros')
CameraMain_002.configure_mw('ros')
Motion_Controller.configure_mw('ros')
Gyroscope_001.configure_mw('ros')

# GPS_001.configure_modifier('NED')
# Motion_Controller.configure_modifier('NED')
# Gyroscope_001.configure_modifier('NED')

#env = Environment('land-1/buildings_2')
env = Environment('land-1/trees')
env.aim_camera([1.0470, 0, 0.7854])
env.place_camera([10, -10, 15])
