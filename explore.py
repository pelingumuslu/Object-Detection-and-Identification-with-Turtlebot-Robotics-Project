#!/usr/bin/env python
import rospy
import getch as g
import json
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import tf
from math import pi
import take_photo
#functions

def laser_callback(laser_msg):
    """motor_command = Twist()
    motor_command.angular.z=0.2
    motor_command_publisher.publish(motor_command)

    return"""
    #control variables
    turnLeft=False
    turnRight=False
    moveForward=False
    laser_ranges=laser_msg.ranges

    mostClosestPointInLeft=99999999;
    mostClosestPointInRight=9999999;

    totalRightDistance=0.0
    totalLeftDistance=0.0


    leftFalingEdgeIndex=-1
    rightFalingEdgeIndex=-1

    rangeSize=len(laser_ranges)
    middlePoint=rangeSize/2

    for index in range(rangeSize):
        currentDistance=laser_ranges[index];
        #finding most closest point in left direction
        if(index<middlePoint/2):
            totalRightDistance+=currentDistance
            if(mostClosestPointInRight>currentDistance):
                mostClosestPointInRight = currentDistance

            if index!=0 and rightFalingEdgeIndex==-1:
                difference=currentDistance-laser_ranges[index-1]
                #math absolute
                if(difference<0):
                    difference=difference*-1
                #if difference is greater than thresshold it is right fallin edge
                if(difference>0.5):
                    rightFalingEdgeIndex=index
        if index>middlePoint/2:
            totalLeftDistance+=currentDistance
            if mostClosestPointInLeft>currentDistance:
                mostClosestPointInLeft = currentDistance
            if index!=(rangeSize-1) and leftFalingEdgeIndex==-1:
                difference=currentDistance-laser_ranges[index+1]
                #math absolute
                if difference<0:
                    difference=difference*-1

                #if difference is greater than thresshold it is left falling edge
                if difference>0.5:
                    leftFalingEdgeIndex=index
    #if close to left truns right
    if (mostClosestPointInLeft<0.75):
        turnRight=True
    #if close to left truns left
    if(mostClosestPointInRight<0.75):
        turnLeft=True

    #if there is two falling edge there is a door between them
    if(leftFalingEdgeIndex!=-1 and rightFalingEdgeIndex!=-1):
        turnLeft=False
        turnRight=False
    #if there is a faling edge on right doors is on right side of the robot
    elif(rightFalingEdgeIndex!=-1 and laser_ranges[rightFalingEdgeIndex]<1.5):
        turnRight=True
        turnLeft=False
    #if there is a faling edge on right doors is on right side of the robot
    elif(leftFalingEdgeIndex!=-1 and laser_ranges[leftFalingEdgeIndex]<1.5):
        turnRight=False
        turnLeft=True

    #if it is too close to left it should return right even if there is a door on left
    if(mostClosestPointInLeft<1.0):
        turnRight=True
    #if it is too close to right it should return right even if there is a door on right
    if(mostClosestPointInRight<1.0):
        turnLeft=True

    if(turnRight and turnLeft):
        moveForward=False
    else:
        moveForward=True



    #if robot needs to turn both size it should not move but turn around
    motor_command = Twist()
    if(turnLeft and turnRight):
        motor_command.linear.x=0
        #it is for eleminating of turning same direction every timeinstead of that it turns to direction has more space
        if(totalLeftDistance>totalRightDistance):
            motor_command.angular.z=-1
        else:
            motor_command.angular.z=1
    elif(turnLeft):
        motor_command.angular.z=1
    elif(turnRight):
        motor_command.angular.z=-1
    else:
        motor_command.angular.z=0

    if(moveForward):
        motor_command.linear.x=0.5
    else:
        motor_command.linear.x=0
    #publish command:
    motor_command_publisher.publish(motor_command)

    return

if __name__=='__main__':

    rospy.init_node("build_map")
    takePhoto=take_photo.TakePhoto()

    laser_sub=rospy.Subscriber("/scan",LaserScan,laser_callback,queue_size=1000)
    global motor_command_publisher
    motor_command_publisher=rospy.Publisher("/cmd_vel_mux/input/navi",Twist,queue_size=100)
#    mapReader=rospy.Subscriber("/map",Transform,mapCallBack)
#    publish=rospy.Publisher("/nav_msgs/",OccupancyGrid,queue_size=100)

    if g.getch() == '.':
        print(takePhoto.objectCoordinates)
        file=open("input1.txt","w")
        file.write(json.dumps(takePhoto.objectCoordinates))
        file.close()

    # with open('file2.txt', 'w') as file:
    #     file.write(json.dumps(takePhoto.objectCoordinates))
