#!/usr/bin/env python

import rospy
import math
import enum
import copy
import array
import sys
import select
import random
import time

import numpy as np
import matplotlib.pyplot as plt


from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan


pos = [0,0]
scanner_readings = array.array('i',(0 for i in range(0,90)))


def checkForEnterKey():
    i,o,e = select.select([sys.stdin],[],[],0.00001)
    for s in i:
            if s == sys.stdin:
                received_line = sys.stdin.readline()
                return received_line
    return "n"


def shutdown_sequence():
    # Create a stop message for the robot
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub_vel.publish(twist)


def PosCallBack(ros_data):
    global pos
    # Save the position data from Vicon into a global variable
    pos[0] = ros_data.transform.translation.x
    pos[1] = ros_data.transform.translation.y

    cp = PointStamped()
    cp.header.frame_id = "/world"
    cp.point.x = pos[0]
    cp.point.y = pos[1]
    cp.point.z = 0.0
    pub_pos.publish(cp)


def scanCallBack(ros_data):
    global scanner_readings

    # Create a deep copy of all scan data
    data_copy = copy.deepcopy(list(ros_data.ranges))

    # Copy only the readings from -45 degrees to 45 degrees
    scanner_readings = data_copy[359-45:359] + (data_copy[0:45])

    # Change all values which are 0 to inf
    for i in range(0,len(scanner_readings)):
        if scanner_readings[i] < 0.01:
            scanner_readings[i] = float('inf')


class DistancesEnum(enum.IntEnum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2


class DirectionEnum(enum.IntEnum):
    STRAIGHT = 0
    LEFT = 1
    RIGHT = 2


class ObsticleDetection:
    def CheckForObsticles(self, readings):

        # Calculate the closest obsticle in my center view
        mid_point = int(len(readings) / 2)
        center_readings = copy.deepcopy(readings[mid_point - 10:mid_point + 10])
        center_sorted_readings = sorted(center_readings)
        center_threeSmallestReadings = [center_sorted_readings[0], center_sorted_readings[1], center_sorted_readings[2]] 
        center_closest_obsticle = np.nanmean(center_threeSmallestReadings);

        # Calculate the closest obsticle in all my view
        # sorted_readings = sorted(copy.deepcopy(readings))
        # threeSmallestReadings = [sorted_readings[0], sorted_readings[1], sorted_readings[2]] 
        # all_closest_obsticle = np.nanmean(threeSmallestReadings);
        all_closest_obsticle = np.min(readings)

        if all_closest_obsticle > 1.5:
            return DistancesEnum.FAR
        elif center_closest_obsticle < 0.5:
            return DistancesEnum.CLOSE
        else:
            return DistancesEnum.MEDIUM

    def CheckSpace(self, readings):
        object_direction = 0

        for i in range(0,45):
            if readings[i] < 1:
                object_direction = 45 - i
                break 
            if readings[-i] < 1:
                object_direction = -45 + i
                break

        return object_direction


class RobotBehaviourSensors:

    def __init__(self, rate_in):
        # Inititilize the current goal and position
        self.current_position = [0,0]
        self.goal = [0,0]

        # Previous Positions is an array of 2 values (can be increased later)
        self.previous_position = []
        self.previous_list_length = 2
        for i in range(0,self.previous_list_length):
            self.previous_position.append(self.current_position)

        # Calculate the time between position updates
        self.dt = 1.0 / rate_in


    def setGoal(self, goal_in):
        # Set the goal
        self.goal = [goal_in.point.x, goal_in.point.y]

    def updatePosition(self, current_position):
        # Make a deep copy of the update
        current_position_dc = copy.deepcopy(current_position)

        # Set the current position (make sure its a deep copy, so that chaning current_position does not change all values in the previous array)
        self.current_position = copy.deepcopy(current_position_dc)

        # Save the position to the previous position array
        self.previous_position.insert(0, current_position_dc)
        if len(self.previous_position) > self.previous_list_length:
            # Remove the oldest previous position element
            self.previous_position.pop()

    def GetRobotHeading(self):
        # Calculate the change in distance in both X and Y
        deltaX = self.current_position[0] - self.previous_position[-1][0]
        deltaY = self.current_position[1] - self.previous_position[-1][1]

        # Calculate the angle between the two points
        current_heading = math.atan2(deltaY, deltaX)

        return current_heading

    def GetGoalHeading(self):
        # Calulate the difference in X and Y from current to goal
        deltaX = self.goal[0] - self.current_position[0]
        deltaY = self.goal[1] - self.current_position[1]

        # Calculate the angle to that goal
        desired_heading = math.atan2(deltaY, deltaX)
        return desired_heading

    def GetRobotVelocity(self):
        # Calculate the change in distance in both X and Y
        deltaX = self.current_position[0] - self.previous_position[-1][0]
        deltaY = self.current_position[1] - self.previous_position[-1][1]

        # Calculate the total change in distance and use this to get the velocity
        total_distance = math.sqrt(pow(deltaX,2) + pow(deltaY,2))
        current_velocity = total_distance / self.dt

        return current_velocity


class Planner:

    def __init__(self, rate):
        # PID values
        self.ang_prev_err = 0
        self.ang_integral = 0
        self.vel_prev_err = 0
        self.vel_integral = 0

        # PID parameters
        self.ang_Kp = 1.1
        self.ang_Ki = 0.0
        self.ang_Kd = 0.0
        self.vel_Kp = 0.5
        self.vel_Ki = 0.1
        self.vel_Kd = 0.0

        # Setting the rate
        self.dt = 1.0/rate

    def GetAngularVelocity(self, object_distance, object_side, robot_heading, desired_heading):

        # The robots desired heading is calculated 
        if object_distance == DistancesEnum.MEDIUM:
            desired_heading = desired_heading + object_side

        if object_distance == DistancesEnum.FAR:
            desired_heading = desired_heading

        # Angular velocity heading error
        error = desired_heading - robot_heading
        error = math.atan2(math.sin(error), math.cos(error))

        # Angular velocity PID
        self.ang_integral = self.ang_integral + (error * self.dt)
        derivate = (error - self.ang_prev_err) / self.dt
        self.ang_prev_err = error

        # Calculate the angle
        PID_angle = self.ang_Kp*error + self.ang_Ki*self.ang_integral + self.ang_Kd*derivate
        anglular_velocity = PID_angle

        return anglular_velocity

    def GetLinearVelocity(self, object_distance, goal_distance, robot_speed):

        # Desired speed is a function of distance
        desired_speed = min((goal_distance + 0.1 /2),1) * 0.5

        # If the object is close, stop the robot
        if object_distance == DistancesEnum.CLOSE:
            # Stop the robot
            desired_speed = 0

        # Velocity error
        error = desired_speed - robot_speed 
        error = math.atan2(math.sin(error), math.cos(error))

        # Linaer velocity PID
        self.vel_integral = self.vel_integral + (error * self.dt)
        derivate = (error - self.vel_prev_err) / self.dt
        self.vel_prev_err = error

        # Calculate speed
        PID_speed = self.vel_Kp*error + self.vel_Ki*self.vel_integral + self.vel_Kd*derivate
        linear_velocity = PID_speed

        return linear_velocity


if __name__=="__main__":
    global pos
    global scanner_readings

    # Declares if we are learning or not
    learning = True

    # Start the ros node
    rospy.init_node('turtle_driver')
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    pub_goal = rospy.Publisher('current_goal', PointStamped, queue_size=10)
    pub_pos = rospy.Publisher('current_location', PointStamped, queue_size=10)
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    subscriber = rospy.Subscriber('/scan', LaserScan, scanCallBack)
    subscriber = rospy.Subscriber('vicon/BUGS/BUGS', TransformStamped, PosCallBack)
    
    # Setting the rate
    set_rate = 20
    rate = rospy.Rate(set_rate)

    # Allow Ros to catch up
    time.sleep(1)

    # Creating all the robot objects
    OD_Obj = ObsticleDetection()
    RB_Obj = RobotBehaviourSensors(set_rate)
    PN_Obj = Planner(set_rate)

    # Defining where we want the robot to go
    goal = PointStamped()
    goal.header.frame_id = "/world"
    goal.point.x = random.uniform(-2,2)
    goal.point.y = random.uniform(-2,2)
    goal.point.z = 0.0
    # Publish current goal
    pub_goal.publish(goal)

    RB_Obj.setGoal(goal)

    # Remember the state of the robot
    current_path = DistancesEnum.FAR
    current_state = DirectionEnum.STRAIGHT
    previous_state = DirectionEnum.STRAIGHT

    # Define where the trust array
    trust = []

    # Used to stop incrementing the goal too many times
    incremented = False 

    while not rospy.is_shutdown():

        # Calculate the distance to the goal
        x_distance_to_goal = pos[0] - goal.point.x
        y_distance_to_goal = pos[1] - goal.point.y
        distance_to_goal = math.sqrt(pow(x_distance_to_goal,2) + pow(y_distance_to_goal,2))

        # Get the robot and goal headings
        RB_Obj.updatePosition(pos)
        current_heading = RB_Obj.GetRobotHeading()
        current_speed = RB_Obj.GetRobotVelocity()
        goal_heading = RB_Obj.GetGoalHeading()

        print("Velocity: " + '\t' + str(current_speed))
        print("Heading: " + '\t' + str(current_heading))

        # Check for obstacles
        obsticle_distance = OD_Obj.CheckForObsticles(scanner_readings)
        turn_direction = OD_Obj.CheckSpace(scanner_readings)
        print("Turn Direction: " + str(turn_direction))

        # Get the next move
        required_angular_velocity = PN_Obj.GetAngularVelocity(obsticle_distance, turn_direction, current_heading, goal_heading)
        required_linear_velocity = PN_Obj.GetLinearVelocity(obsticle_distance, distance_to_goal, current_speed)

        # Set the state of the robot 
        if -0.5 < required_angular_velocity < 0.5:
            current_state = DirectionEnum.STRAIGHT
        if required_angular_velocity > 0.5:
            current_state = DirectionEnum.LEFT
        if required_angular_velocity < -0.5:
            current_state = DirectionEnum.RIGHT

        # Publish the message
        twist = Twist()
        twist.linear.x = 1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = required_angular_velocity
        pub_vel.publish(twist)

        # Display the state of the robot
        current_path = obsticle_distance
        print("Transition: " + '\t' + str(current_path))
        print("State: " + '\t' + '\t' + str(current_state))
        print("D to Goal: " + '\t' + str(distance_to_goal))
        print("-------------------")
         
        # Check if the robot has reached its goal
        if distance_to_goal < 0.3:
            # Increment the goal counter
            goal.point.x = random.uniform(-2,2)
            goal.point.y = random.uniform(-2,2)
            RB_Obj.setGoal(goal)
            # Publish current goal
            pub_goal.publish(goal)

        # Save the previous state
        previous_state = current_state 

        # Sleep
        rate.sleep()