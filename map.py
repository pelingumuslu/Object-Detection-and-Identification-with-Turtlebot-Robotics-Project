#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import tf
from math import pi

#functions
def lasercallBack(msg): 
    
    laserscan=sensor_msgs.LaserScan()
    cmd=geometry_msgs.Twist()
    beams=laserscan.ranges.size()   #total number of ranges
    
    laserscan=msg
   
    total_alpha=(laserscan.angle_max-laserscan.angle_min)/laserscan.angle_increment   #range of robots front view
    
    min_dist=laserscan.ranges[0]
    min_beam=0
   
   #check all the beams to decide minimum distance point and return error message if the range is exceeded
    for i in range(0,beams,1):
        if laserscan.ranges[i]<laserscan.range_min or laserscan.ranges[i]>laserscan.range_max:
            ROS_DEBUG("Meaningless data at input %d",i)
            continue
   
        if min_dist > laserscan.ranges[i]: #if ith point is closer to an obstacle or wall update min distance
            min_dist=laserscan.ranges[i]
            min_beam=i #update the beam 
            
    front_dist=laserscan.ranges[beams/2]#front distance
    angle_diff=((beam/2)-min_beam)*laserscan.angle_increment #angle difference between front and min distance points
    
    #MUST DECIDE DIRECTION AND MULTIPLY WITH ANGLE_DIFF
    cmd.angular.z=(pi/4)-angle_diff #rotate robot to make it parallel to the wall
    
    #wisely move robot
    if front_dist<min_dist:
        cmd.linear.x=0
    elif front_dist<2*min_dist:
        cmd.linear.x=0.5
    else:
        cmd.linear.x=1.0
        
    laser_pub.publish(cmd)
        
        
def mapCallBack(messg):
    
    map_info=nav_msgs.OccupancyGrid()
    _map=messg.data
   
    size=map_info.data.size()
    
    free=0
    full=0
    unknown=0
    
    #place and check point(x,y) in the map matrix
    row=_map.translation.x
    col=_map.translation.y
   
    if map_info.data[row*map_info.info.width+col] == 0:     #free space
        ++free
        print("Point(x,y): (",row,",",col,") is free")
    elif map_info.data[row*map_info.info.width+col] == 1:    #fully occupied
        ++full
        print("Point(x,y): (",row,",",col,") is full")
    else :  #unknown
        ++unknown
        print("Point(x,y): (",row,",",col,") is unknown area")        
   
    mapReader.publish(map_info)
    

#main function
if __name__ == '__main__':
    
    rospy.init_node("build_map")
    
    laser_sub=rospy.Subscriber("/scan",100,lasercallBack)
    laser_pub=rospy.Publisher("/cmd_vel_mux/input/navi",Twist,100)

    mapReader=rospy.Subscriber("/map",Transform,mapCallBack)    
    publish=rospy.Publisher("/nav_msgs/",OccupancyGrid,100)
    

    listener=tf.TransformListener()
    
    
    delay=rospy.Rate(1.0);
    while not rospy.is_shutdown():
        
        try:
            
            (translation,orientation)=listener.lookupTransform("/odom","/base_link",rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        
    
        delay.sleep()
        
        
    print("Map is created successfullly...I mean hopefully :) ")
