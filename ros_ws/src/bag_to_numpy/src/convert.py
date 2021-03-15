#!/usr/bin/env python

import rospy
import array
import sys

import numpy as np

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

a = []
global filename

def shutdown_sequence():
    global filename
    print("Trying to save to " + str(filename))
    if len(a) > 0:
        x = np.stack(a)
        np.save(filename, x)
        print("Saved to " + str(filename))
    else:
        print("No data to save")


def create_message(pos,cmd,scan):
    global a
    final_output = array.array('f',(0.0 for i in range(0,367)))

    # Creat the final array output
    if pos is None:
        final_output[-3] = 0
    else:
        print("POS DATA")
        final_output[0] = pos[0]
        final_output[1] = pos[1]
        final_output[-3] = 1

    if cmd is None:
        final_output[-2] = 0
    else:
        print("CMD DATA")
        final_output[2] = cmd[0]
        final_output[3] = cmd[1]
        final_output[-2] = 1

    if scan is None:
        final_output[-1] = 0
    else:
        print("SCAN DATA")
        start_index = 0
        for data in scan:
            final_output[start_index + 4] = scan[start_index]
        final_output[-1] = 1
        
    a.append(final_output)

def command_call_back(ros_data):
    # command data
    #print("Linear: " + str(ros_data.linear.x))
    #print("Angular: " + str(ros_data.angular.z))

    # Pass the data to message creator
    cmd_data = [ros_data.linear.x, ros_data.angular.z]
    create_message(None,cmd_data,None)


def scan_call_back(ros_data):
    # Scan data
    #print("Scan: " + str(ros_data.ranges))

    # Pass the data to message creator
    scn_data = ros_data.ranges
    create_message(None,None,scn_data)

def pos_call_back(ros_data):
    # Positional data
    #print("Pos X: " + str(ros_data.transform.translation.x))
    #print("Pos Y: " + str(ros_data.transform.translation.y))

    # Pass the data to message creator
    pos_data = [ros_data.transform.translation.x, ros_data.transform.translation.y]
    create_message(pos_data,None,None)


if __name__=="__main__":
    global filename

    if len(sys.argv) < 2:
        print("Usage: rosrun bag_to_numpy convert.py filename")
        filename = "default.npy"
    else:
        # Get the parameters (filename)
        filename =  sys.argv[1]

    # Display the save location
    print("File set to `" + str(filename) + "'")

    # Start the ros node
    rospy.init_node('converter')
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    sub_vel = rospy.Subscriber('/cmd_vel', Twist, command_call_back)
    sub_scn = rospy.Subscriber('/scan', LaserScan, scan_call_back)
    sub_pos = rospy.Subscriber('vicon/BUGS/BUGS', TransformStamped, pos_call_back)

    # Setting the rate
    set_rate = 1
    rate = rospy.Rate(set_rate)

    # Main program loop
    while not rospy.is_shutdown():

        # We are still alive
        print("Converting")

        # Sleep
        rate.sleep()