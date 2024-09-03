import math, rospy
from utilities import set_model_state, get_model_state, \
                      spawn_coke_can
from geometry_msgs.msg import Pose, Point, Quaternion

if get_model_state('coke_can').success == False:
    spawn_coke_can('coke_can', Pose(position=Point(0,1,0.22)))

model_state = get_model_state('coke_can')
print (model_state.pose.position)

new_pose = Pose(position=Point(1,0,0.22))
set_model_state('coke_can', new_pose)

x, y, z = 1, 0, 0.22
for angle in range(0,360,10):
    theta = math.radians(angle)
    xp = x * math.cos(theta) - y * math.sin(theta)
    yp = x * math.sin(theta) + y * math.cos(theta)
    set_model_state('coke_can', Pose(position=Point(xp,yp,z)))
    rospy.sleep(0.1)

position = Point(x=0, y=0, z=0)
for angle in range(0,360,10):
    theta = math.radians(angle)
    orientation = Quaternion(x=0, y=0, z=math.sin(theta/2), w=math.cos(theta/2))
    set_model_state('mobile_base', Pose(position, orientation))
    rospy.sleep(0.1)