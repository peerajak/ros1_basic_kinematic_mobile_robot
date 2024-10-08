#! /usr/bin/env python

import rospy, math, numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# half of the wheel base distance
l = 0.500/2
# the radius of the wheels
r = 0.254/2
# half of track width
w = 0.548/2


def odom_callback(msg):
    global phi   
    position = msg.pose.pose.position
    (_, _, phi) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                         msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z, 
                                         msg.pose.pose.orientation.w])
    print('odom callback phi ',phi)

    
rospy.init_node('absolute_motion', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
position_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.sleep(1)

def u_to_twist3(u):
    global l
    global r
    global w
    u_array = np.array(u).reshape((4, 1))
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r   
    print('H',H)
    Hinv_least_square = np.dot(np.linalg.inv(np.dot(H.T, H)), H.T)
    twist = np.dot(Hinv_least_square, u_array)
    return twist.flatten().tolist()


def velocity2twist(dphi, dx, dy):
    print('velocity2twist',phi)
    R = np.array([[1, 0, 0],
                  [0,  np.cos(phi), np.sin(phi)],
                  [0, -np.sin(phi), np.cos(phi)]])
    v = np.array([dphi, dx, dy])
    v.shape = (3,1)
    twist = np.dot(R, v)
    wz, vx, vy = twist.flatten().tolist()
    return wz, vx, vy


def twist2wheels(wz, vx, vy):
    global l
    global r
    global w    
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r
    twist = np.array([wz, vx, vy])
    twist.shape = (3,1)
    u = np.dot(H, twist)
    return u.flatten().tolist()

waypoints = [(0,0),(1,1),(2,0),(3,-1),(4,0),(3,1),(2,0),(1,-1),(0,0)] 

motions = [ (lambda a,b: (a[0] - b[0], a[1] - b[1],math.atan((a[1]-b[1])/(a[0]-b[0]))))(waypoints[1:][i] , waypoints[:-1][i])  for i in range(len(waypoints[:-1])) ]#a is destination,b is current position
print(motions)
# motion_angles =  [ (lambda a,b: math.atan((a[1]-b[1])/(a[0]-b[0])))(waypoints[1:][i] , waypoints[:-1][i])  for i in range(len(waypoints[:-1])) ]#a is destination,b is current position
# print(motion_angles)

for (mx, my, delta_phi) in motions:
    dx = mx
    dy = my
    dphi = delta_phi
    # constant angular velocity for turning at 30 degrees/second
    #dphi = math.radians(45)
    # publish this motion for about 3 seconds = iterations (300) x sleep value of 0.01 
    iterations = 100
    avg_wz = 0
    avg_vx = 0
    avg_vy = 0
    for _ in range(iterations):
        wz, vx, vy = velocity2twist(dphi, dx, dy)
        print(wz, vx, vy)
        avg_wz += wz
        avg_vx += vx
        avg_vy += vy        
        u = twist2wheels(wz, vx, vy)
        msg = Float32MultiArray(data=u)
        pub.publish(msg)
        twist_inv3 = u_to_twist3(u)
        print('twist msg inversed 3 from u',twist_inv3)
        rospy.sleep(0.01)
    avg_wz /= iterations
    avg_vx /= iterations
    avg_vy /= iterations
    print('average wz {} vx {} vy {}'.format(avg_wz, avg_vx, avg_vy))
stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)