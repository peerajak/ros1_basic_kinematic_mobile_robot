import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
import random

rospy.init_node('kinematic_controller_holonomic', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
rospy.sleep(1)



class OdometryReader():
    def __init__(self, topic):
        self.odom_pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        global phi
        self.odom_pose['x'] = msg.pose.pose.position.x
        self.odom_pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.odom_pose['x'], self.odom_pose['y']))
        (_, _, self.odom_pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
        phi = self.odom_pose['theta']
    
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
def velocity2twist(dphi, dx, dy):
    global phi
    R = np.array([[1, 0, 0],
                  [0,  np.cos(phi), np.sin(phi)],
                  [0, -np.sin(phi), np.cos(phi)]])
    v = np.array([dphi, dx, dy])
    v.shape = (3,1)
    twist = np.dot(R, v)
    wz, vx, vy = twist.flatten().tolist()
    return wz, vx, vy

def twist2wheels(wz, vx, vy):
    
    # half of the wheel base distance
    l = 0.500/2
    # the radius of the wheels
    r = 0.254/2
    # half of track width
    w = 0.548/2
    
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r
    twist = np.array([wz, vx, vy])
    twist.shape = (3,1)
    u = np.dot(H, twist)
    return u.flatten().tolist()

def go_to(xg, yg, thetag_degrees):
    global k_rho
    global k_alpha
    global k_beta
    global phi
    rho = float("inf")
    thetag = normalize(math.radians(thetag_degrees))
    while rho>0.01:
        ############
        ### YOUR ###
        ### CODE ###
        x_pos = odometry.odom_pose['x']
        y_pos = odometry.odom_pose['y']
        theta_pos = phi#odometry.odom_pose['theta']
        delta_x = xg - x_pos
        delta_y = yg - y_pos
        rho = math.sqrt(delta_x**2 + delta_y**2)
        alpha = 0#normalize(-theta_pos + math.atan2(delta_y , delta_x))
        #positive w is ccw ,negative w is cw
        beta  = normalize(-thetag - (alpha + theta_pos))
        w = k_alpha*alpha + k_beta*beta
        wz, vx, vy = velocity2twist(dphi=w, dx=delta_x , dy=delta_y )

        #v = k_rho*rho
        #vx = v*math.cos(theta_pos)
        #vy = v*math.sin(theta_pos)
        #w = k_alpha*alpha + k_beta*beta
        print('rho {} thetag {} beta {}'.format(rho,thetag,beta))
        ############
        u = twist2wheels(wz, vx, vy)
        msg = Float32MultiArray(data=u)
        pub.publish(msg)
        rospy.sleep(0.01)
        
k_rho = 0.3
k_alpha = 0.8
k_beta = 1#-0.15



odometry = OdometryReader('/odom')
ref_positions= [(1,-1,90)]#, (2,0,0), (3,1,0), (4,0,90),(3.5,-0.5,0),(3,-1,-90),(2,0,0),(1,1,0),(0,0,0)]
#ref_positions= [(1,-1,0), (2,0,0), (3,1,0), (4,0,90),(3.5,-0.5,0),(3,-1,-90),(2,0,0),(1,1,0),(0,0,0)]

for (mx, my, mphi) in ref_positions:
    dx = mx
    dy = my
    dphi = mphi
    print(dphi)
    go_to(dx,dy, dphi)

stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
print('Final positioning error is %.2fm' % error)