#! /usr/bin/env python

import rospy, numpy as np
from std_msgs.msg import Float32MultiArray
# half of the wheel base distance
l = 0.500/2
# the radius of the wheels
r = 0.254/2
# half of track width
w = 0.548/2

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

def u_to_twist(u):
    global l
    global r
    global w
    u_array = np.array(u).reshape((4, 1))
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r   
    Hinv = np.linalg.pinv(H) 
    twist = np.dot(Hinv, u_array)
    return twist.flatten().tolist()


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

rospy.init_node('make_turn', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
rospy.sleep(1)

u = twist2wheels(wz=1.5, vx=1, vy=0)
twist_inv = u_to_twist(u)
print('twist msg inversed from u',twist_inv)
twist_inv3 = u_to_twist3(u)
print('twist msg inversed 3 from u',twist_inv3)
msg = Float32MultiArray(data=u)
pub.publish(msg)
rospy.sleep(1)
stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)