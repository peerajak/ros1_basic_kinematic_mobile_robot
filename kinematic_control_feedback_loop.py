import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import random

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

def normalize(angle):
    ############
    ### YOUR ###
    ### CODE ###
    if angle >= -math.pi and angle <= math.pi:
        return angle
    
    if angle < -math.pi:
        sign_multiplier = 1
    
    if angle > math.pi:
        sign_multiplier = -1
    
    normalized_angle = angle
    for i in range(20):
        normalized_angle += sign_multiplier*2*math.pi 
        if normalized_angle >= -math.pi and normalized_angle <= math.pi:
            return normalized_angle
    return None
    
    ############


# for _ in range(10):
#     angle = (random.random()-0.5)*2*math.pi
#     new_angle = angle + (random.randint(0,10)-5) * 2*math.pi
#     norm_angle = normalize(new_angle)
#     print('%9.4f %9.4f %9.4f' %(angle, new_angle, norm_angle))

def go_to(xg, yg, thetag_degrees):
    global k_rho
    global k_alpha
    global k_beta
    improvement_version = False
    rho = float("inf")
    thetag = normalize(math.radians(thetag_degrees))
    while rho>0.01:
        ############
        ### YOUR ###
        ### CODE ###
        x_pos = odometry.odom_pose['x']
        y_pos = odometry.odom_pose['y']
        theta_pos = odometry.odom_pose['theta']
        delta_x = xg - x_pos
        delta_y = yg - y_pos
        rho = math.sqrt(delta_x**2 + delta_y**2)
        alpha = normalize(-theta_pos + math.atan2(delta_y , delta_x))
        beta  = normalize(thetag - (alpha + theta_pos))
        
        v = k_rho*rho
        w = k_alpha*alpha + k_beta*beta
        print('rho {}'.format(rho))
        if improvement_version:
            norm_vw = math.sqrt(v**2 + w**2)
            v_constant = 0.65
            s = v_constant/norm_vw
        else:
            s = 1
        ############
        velocity.move(v/s, w/s)
        rospy.sleep(0.01)
        
k_rho = 0.3
k_alpha = 0.8
k_beta = -0.15


velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

v=0.65
waypoints = [(1,-1,-90),(2,-2,0),(3,-2,0),(4,-1,90),(3.5,-0.5,180),
             (3,0,90),(3,1,90),(2,1,-90),(1,0,180),(0,0,180)]

for xg, yg, thetag in waypoints:
    go_to(xg, yg, thetag)

velocity.move(0,0)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
print('Final positioning error is %.2fm' % error)