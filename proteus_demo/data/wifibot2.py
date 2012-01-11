from morse.builder.morsebuilder import *

environment = 'land-1/trees' # outdoor
#environment = 'indoors-1/indoor-1' # indoor

# in case you launche this script with a .blend file
if len(bpy.data.objects) > 0:
    environment = None

# Append ATRV robot to the scene
atrv = Robot('atrv')
atrv.name = 'ATRV'

# Append an actuator
motion = Actuator('v_omega')
motion.name = 'Motion_Controller'
motion.translate(z = 0.3)
atrv.append(motion)

# Append an odometry sensor
odometry = Sensor('odometry')
odometry.name = 'Odometry'
odometry.translate(x = -0.1, z = 0.83)
atrv.append(odometry)

# Append a Pose sensor (GPS + Gyroscope)
pose = Sensor('pose')
pose.name = 'Pose_sensor'
pose.translate(x = -0.25, z = 0.83)
atrv.append(pose)

# Append a sick laser
sick = Sensor('sick')
sick.name = 'Sick'
sick.translate(x = 0.18, z = 0.94)
atrv.append(sick)
# sick.properties(scan_window = 270, resolution = .25)
# sick.properties(scan_window = 180, resolution = 1)

# Append a camera
camera = Sensor('video_camera')
camera.name = 'CameraMain'
camera.translate(x = 0.3, z = 0.96)
atrv.append(camera)
camera.properties(cam_width = 128, cam_height = 128, Vertical_Flip = True)

# Append infrared sensors (left and right)
infrared_r = Sensor('infrared')
infrared_r.name = 'InfraredR'
infrared_r.translate(x = 0.6, y = -0.15,z = 0.2)
atrv.append(infrared_r)
infrared_l = Sensor('infrared')
infrared_l.name = 'InfraredL'
infrared_l.translate(x = 0.6, y = 0.15,z = 0.2)
atrv.append(infrared_l)

# Configure the middlewares
motion.configure_mw('ros')
odometry.configure_mw('ros')
pose.configure_mw('ros')
sick.configure_mw('ros')
camera.configure_mw('ros')
infrared_r.configure_mw('ros')
infrared_l.configure_mw('ros')

# Select the environement
env = Environment(environment)
env.aim_camera([1.0470, 0, 0.7854])

