#!/usr/bin/env python

import rospy
import math
import copy 

import matplotlib.pyplot as plt
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan

lp = lg.LaserProjection()
scanner_readings_x = []
scanner_readings_y = []

def scanCallBack(ros_data):
    global scanner_readings_x
    global scanner_readings_y
    scanner_readings_x = []
    scanner_readings_y = []

    pc2_msg = lp.projectLaser(ros_data)
    point_generator = pc2.read_points(pc2_msg)

    for point in point_generator:
        scanner_readings_x.append(point[0])
        scanner_readings_y.append(point[1])




if __name__=="__main__":
  
    global scanner_readings_x
    global scanner_readings_y

    # Start the ros node
    rospy.init_node('scaner_vishulizer')

    # Subscribers and publishers
    subscriber = rospy.Subscriber('/scan', LaserScan, scanCallBack)
    
    # Setting the rate
    set_rate = 5
    rate = rospy.Rate(set_rate)

    while not rospy.is_shutdown():

        scan_x = copy.deepcopy(scanner_readings_x)
        scan_y = copy.deepcopy(scanner_readings_y)

        if (len(scan_x) == len(scan_y)):
            print(len(scan_x))
            print(len(scan_y))
            print("------------------------")
            # Plot the trust
            plt.clf()
            plt.ylim((-4,4))
            plt.xlim((-4,4))
            plt.ylabel('Position Y')
            plt.xlabel('Position X')
            plt.scatter(scan_x,scan_y)
            plt.pause(0.000001)

        # Sleep
        rate.sleep()