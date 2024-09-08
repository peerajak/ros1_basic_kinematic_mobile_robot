import rospy, math, numpy as np
from plot_functions import plot_trajectory

ref_points = [(0,0),(1,-1),(2,-2),(3,-2),(4,-1),(3.5,-0.5),(3,0),(3,1),(2.5,1.5),(2,1),(1,0),(0,0)]
ref_points2 = [(1,0),(1,0),(2,-2.3),(3,2),(0.4,-1),(3.25,0.5),(3,1),(-3,1),(2.5,-1.5),(2,-1),(-1,0),(0.5,0)]
plot_trajectory(ref_points)
plot_trajectory(ref_points2)