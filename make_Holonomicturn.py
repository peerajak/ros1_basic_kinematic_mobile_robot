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

def u_to_twist2(u):
    global l
    global r
    global w
    u_array = np.array(u).reshape((4, 1))
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r   
    print('H',H)
    U, S, Vh = np.linalg.svd(H, False)
    print('USVh',U,S,Vh)
    S_plus_diag = np.array([1/element if element != 0 else 0 for element in S.tolist()])
    S_plus = np.zeros((4,3),dtype='float64')
    np.fill_diagonal(S_plus, S_plus_diag)
    
    print('US+Vh',U,S_plus,Vh)
    Hinv = np.dot(np.dot(np.conjugate(Vh).T,S_plus),np.conjugate(U).T)
    print('Hinv', Hinv)
    twist = np.dot(Hinv, u_array)
    return twist.flatten().tolist()


rospy.init_node('make_turn', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
rospy.sleep(1)

u = twist2wheels(wz=1.5, vx=1, vy=0)
twist_inv = u_to_twist(u)
print('twist msg inversed from u',twist_inv)
twist_inv2 = u_to_twist2(u)
print('twist msg inversed 2 from u',twist_inv2)
msg = Float32MultiArray(data=u)
pub.publish(msg)
rospy.sleep(1)
stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)