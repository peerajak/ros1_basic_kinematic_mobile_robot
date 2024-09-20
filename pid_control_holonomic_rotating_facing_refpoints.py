import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
import math
import time

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


class PID_controller():
    def __init__(self,KP,KI,KD, KP_angle, KI_angle, KD_angle):
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.Kp_angle = KP_angle
        self.Ki_angle = KI_angle
        self.Kd_angle = KD_angle
        self.integral_x_pos = 0
        self.integral_y_pos = 0
        self.integral_theta = 0
        self.Hz = 10.0
        self.dt = 1/self.Hz

    def reset(self):
        self.integral_x_pos = 0
        self.integral_y_pos = 0
        self.integral_theta = 0       

    def pid_controller(self, error_signal):
        #Todo : unit test against normalized angle
        dtheta_pos,dx_pos,dy_pos = error_signal
        # Todo Is it the case delta_x or delta_x/self.dt?
        omega = dtheta_pos/self.dt
        vx = dx_pos/self.dt
        vy = dy_pos/self.dt
        self.integral_x_pos += dx_pos
        self.integral_y_pos += dy_pos
        self.integral_theta += dtheta_pos
        proportion_signal_x = self.Kp*dx_pos
        proportion_signal_y = self.Kp*dy_pos
        proportion_signal_theta = self.Kp_angle*dtheta_pos
        integral_signal_x = self.Ki*self.integral_x_pos
        integral_signal_y = self.Ki*self.integral_y_pos
        integral_signal_theta = self.Ki_angle*self.integral_theta
        derivative_signal_x = self.Kd * vx
        derivative_signal_y = self.Kd * vy
        derivative_signal_theta = self.Kd_angle * omega
        omega = normalize(proportion_signal_theta + integral_signal_theta + derivative_signal_theta)
        vx = proportion_signal_x + integral_signal_x + derivative_signal_x
        vy = proportion_signal_y + integral_signal_y + derivative_signal_y
        controller_signal = (omega, vx, vy)
        return controller_signal

    def error(self, output_signal,xg, yg, thetag): 
        #Todo : unit test against normalized angle
        theta_pos, x_pos, y_pos = output_signal
        error_signal = (thetag - theta_pos, xg - x_pos, yg - y_pos)
        print('output signal position theta:{} x:{} y:{}'.format(theta_pos, x_pos, y_pos))
        print('error_signal d_theta:{} dx:{} dy:{}'.format(error_signal[0],error_signal[1],error_signal[2]))
        return error_signal

    def plant_process(self, controller_signal):
        global odometry
        w, delta_x, delta_y = controller_signal
        wz, vx, vy = velocity2twist(dphi=w, dx=delta_x , dy=delta_y )
        u = twist2wheels(wz, vx, vy)
        msg = Float32MultiArray(data=u)
        pub.publish(msg)
        x_pos =  odometry.odom_pose['x']
        y_pos =  odometry.odom_pose['y']
        theta_pos = odometry.odom_pose['theta']
        # Todo Is it the case delta_x or delta_x/self.dt?
        output_signal = (theta_pos,x_pos,y_pos)
        return output_signal

#positive w is ccw ,negative w is cw
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


def simulate_pid(duration, pid_controller, xg, yg, thetag ,threshold, threshold_angle):
    global odometry
    dt = 0.1   
    x_pos =  odometry.odom_pose['x']
    y_pos = odometry.odom_pose['y']
    theta_pos = odometry.odom_pose['theta']
    output_signal =  (theta_pos,x_pos,y_pos)
    distance_error_norm = 100000 # some large number
    error_angle = 100000 #some large number

    #for i in range(1, len(times)):
    while(distance_error_norm > threshold or error_angle > threshold_angle):
        error = pid_controller.error(output_signal, xg, yg, thetag)
        res = pid_controller.pid_controller(error)
        output_signal = pid_controller.plant_process(res)
        distance_error_norm = math.sqrt(error[1]**2+error[2]**2)
        error_angle = abs(error[0])
        print('distance_error_norm  {} angle_error {}'.format(distance_error_norm, error_angle ))
        rospy.sleep(dt)
        # Keep the statistics.
    pid_controller.reset()

pid_controller = PID_controller(1,0.2,0.2,0.7,0.2,0.2)
odometry = OdometryReader('/odom')

v=0.65
ref_ponits = [(3,2),(1,0),(3,-3),[1,0]]
threshold = 0.01
threshold_angle = math.radians(0.1)
xg = 0
yg = 0
for xg_angle, yg_angle in ref_ponits:
    thetag = math.atan2(yg_angle,xg_angle)
    #thetag = math.radians(thetag_degree)
    simulate_pid(10,pid_controller,xg,yg,thetag, threshold, threshold_angle)
    print("rotating facing ({},{})".format(xg_angle,yg_angle))
    stop = [0,0,0,0]
    msg = Float32MultiArray(data=stop)
    pub.publish(msg)
    time.sleep(4)


stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
print('Final positioning error is %.2fm' % error)
