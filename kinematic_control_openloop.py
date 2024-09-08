import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from plot_functions import plot_trajectory
import math

rospy.init_node('kinematic_controller', anonymous=True)

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)

class OdometryReader():
    def __init__(self, topic):
        self.odom_pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.odom_pose['x'] = msg.pose.pose.position.x
        self.odom_pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.odom_pose['x'], self.odom_pose['y']))
        (_, _, self.odom_pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.odom_subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.odom_subscriber.unregister()

velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')
rospy.sleep(1)

##### YOUR CODE STARTS HERE ##### 
v=0.65
#waypoint (x,y,R)
#waypoints = [(1,1,1), (1,1,-1),(1,0,100),(1,-1,-1),(-0.5,-0.5,-0.5),(-0.5,-0.5,0.5),(0,-1,100), (-0.5,-0.5,-0.5), (-0.5,0.5,-0.5),(-1,1,1),(-1,0,100)]
waypoints = [(1,1,-1), (1,1,1),(1,0,100),(1,-1,1),(-0.5,-0.5,0.5),(-0.5,-0.5,-0.5)
                ,(0,-1,100),(-0.5,-0.5,0.5),(-0.5,0.5,0.5),(-1,1,-1),(-1,0,100)]
ref_points = [(0,0),(1,-1),(2,-2),(3,-2),(4,-1),(3.5,-0.5),(3,0),(3,1),(2.5,1.5),(2,1),(1,0),(0,0)]
#positive w is ccw ,negative w is cw
visited_points = []
visited_points.append((odometry.odom_pose['x'], odometry.odom_pose['y']))
for it in waypoints:
    (dx, dy , R)= it
    print(dx, dy, R)
    w = v/R if R<10 else 0
    if R < 10:
        distance_to_travel = 3.14159*R/2 
    else:
        distance_to_travel = dx if abs(dx) > abs(dy) else dy
    time_to_travel = abs(distance_to_travel/v)
    print(time_to_travel)
    velocity.move(v,w)
    rospy.sleep(time_to_travel)
    #record the visited_points
    visited_points.append((odometry.odom_pose['x'], odometry.odom_pose['y']))

    

##### YOUR CODE ENDS HERE ##### 
velocity.move(0,0)
#velocity.move(0,0)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
print('Final positioning error is %.2fm' % error)

print('visited_points {} :'.format(len(visited_points)), visited_points)
print('ref_points {} :'.format(len(ref_points)), ref_points)
#plot_trajectory(visited_points)
positions_error_list = [ (lambda a,b : math.sqrt((a[1] - b[1])**2 + (a[0] - b[0])**2))(ref_points[i], visited_points[i])  for i in range(len(ref_points))]
print('all positions error is ', sum(positions_error_list)/len(positions_error_list))
