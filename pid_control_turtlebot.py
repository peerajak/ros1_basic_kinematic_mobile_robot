import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
import math
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
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)

    def stop_moving(self):
        self.move()


    def pid_controller(self, error_signal):
        #Todo : unit test against normalized angle
        d_distance,d_angle_to_goal = error_signal
        # Todo Is it the case delta_x or delta_x/self.dt?
        dtheta_pos_and_to_goal = d_angle_to_goal
        omega = dtheta_pos_and_to_goal/self.dt
        vx = d_distance/self.dt
        self.integral_x_pos += self.dt *d_distance
        self.integral_theta += self.dt *dtheta_pos_and_to_goal 
        self.integral_theta = self.integral_theta
        proportion_signal_x = self.Kp*d_distance
        proportion_signal_theta = self.Kp_angle*dtheta_pos_and_to_goal 
        integral_signal_x = self.Ki*self.integral_x_pos
        integral_signal_theta = self.Ki_angle*self.integral_theta
        derivative_signal_x = self.Kd * vx
        derivative_signal_theta = self.Kd_angle * omega
        omega = normalize(proportion_signal_theta + integral_signal_theta + derivative_signal_theta)
        vx = proportion_signal_x + integral_signal_x + derivative_signal_x
        controller_signal = (omega, vx)
        return controller_signal

    def error(self, output_signal,xg, yg, thetag): 
        #Todo : unit test against normalized angle
        theta_pos, x_pos, y_pos = output_signal
        distance_to_goal = math.sqrt((xg-x_pos)**2 + (yg-y_pos)**2)
        angle_to_goal = math.atan2(yg - y_pos,xg-x_pos)
        error_signal = (distance_to_goal, normalize(angle_to_goal - theta_pos))
        print('output signal position theta:{} x:{} y:{}'.format(theta_pos, x_pos, y_pos))
        print('error_signal distance_to_goal:{} error_angle_to_goal:{}'.format(error_signal[0],error_signal[1]))
        return error_signal

    def plant_process(self, controller_signal):
        global odometry
        w, delta_x = controller_signal
        # TODO 
        #wz, vx, vy = velocity2twist(dphi=w, dx=delta_x , dy=delta_y )
        #u = twist2wheels(wz, vx, vy)
        #msg = Float32MultiArray(data=u)
        #pub.publish(msg)
        self.move(delta_x, w)
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
        distance_error_norm =error[0]
        error_angle = abs(error[1])
        print('distance_error_norm  {} angle_error {}'.format(distance_error_norm, error_angle ))
        rospy.sleep(dt)
        # Keep the statistics.


pid_controller = PID_controller(0.1,0.0,0.0,0.1,0.1,0.0)
odometry = OdometryReader('/odom')

v=0.65
ref_ponits = [(2,-2,0)]
threshold = 0.01
threshold_angle = 0.1# math.radians(0.1)

for xg, yg, thetag_degree in ref_ponits:
    thetag = math.radians(thetag_degree)
    simulate_pid(10,pid_controller,xg,yg,thetag, threshold, threshold_angle)


pid_controller.stop_moving()
