import math, rospy
from utilities import spawn_coke_can, spawn_table, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion

unpause_physics()
spawn_table('table', Pose(position=Point(1,0,0)))

for i in range(1):
    spawn_coke_can('coke_can_'+str(i), Pose(position=Point(1,0,4)))
    rospy.sleep(1.0)