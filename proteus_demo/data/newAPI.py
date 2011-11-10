from morse.builder.sensors import *
from morse.builder.actuators import *
from morse.builder.morsebuilder import *

"""
The purpose of this new API is to build components 100% in Python. To avoid to 
access `.blend` files. At the moment, robots, camera, and laser sensor are still 
read from `.blend` files due to their complexity (mesh or/and material).
"""

#environment = 'land-1/trees' # outdoor
environment = 'indoors-1/indoor-1' # indoor

# in case you launche this script with a .blend file
if len(bpy.data.objects) > 0:
    environment = None

# Append ATRV robot to the scene
atrv = Robot('atrv')
atrv.name = 'ATRV'

# Append an actuator
motion = MotionController()
motion.name = 'Motion_Controller'
motion.translate(z = 0.3)
atrv.append(motion)

# Append an odometry sensor
odometry = Odometry()
odometry.name = 'Odometry'
odometry.translate(x = -0.1, z = 0.83)
atrv.append(odometry)

# Append a Pose sensor (GPS + Gyroscope)
pose = Pose()
pose.name = 'Pose_sensor'
pose.translate(x = -0.25, z = 0.83)
atrv.append(pose)

# Append a sick laser sensor
sick = Sick()
sick.name = 'Sick'
sick.translate(x = 0.18, z = 0.94)
atrv.append(sick)
sick.properties(resolution = 1)
# sick.properties(scan_window = 270, resolution = .25)
# sick.properties(scan_window = 180, resolution = 1)

# Append a camera
camera = Camera()
camera.name = 'CameraMain'
camera.translate(x = 0.3, z = 0.96)
atrv.append(camera)
camera.properties(cam_width = 128, cam_height = 128, Vertical_Flip = True)

# Append infrared sensors (left and right)
infrared_r = Infrared()
infrared_r.name = 'InfraredR'
infrared_r.translate(x = 0.6, y = -0.15,z = 0.2)
atrv.append(infrared_r)
infrared_l = Infrared()
infrared_l.name = 'InfraredL'
infrared_l.translate(x = 0.6, y = 0.15,z = 0.2)
atrv.append(infrared_l)

# Append a light actuator
light = Light()
light.translate(x = 0.4, z = 0.9)
atrv.append(light)

# Append a battery sensor
battery = Battery()
atrv.append(battery)

# Configure the middlewares
motion.configure_mw('ros')
odometry.configure_mw('ros')
pose.configure_mw('ros')
sick.configure_mw('ros')
camera.configure_mw('ros')
infrared_r.configure_mw('ros')
infrared_l.configure_mw('ros')
light.configure_mw('ros')
battery.configure_mw('ros')

# Select the environement
env = Environment(environment)
env.aim_camera([1.0470, 0, 0.7854])

