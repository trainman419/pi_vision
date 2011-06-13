#!/usr/bin/env python

import roslib; roslib.load_manifest('pi_slam')
import rospy
import sys

from geometry_msgs.msg import Pose, Twist, Quaternion
from nav_msgs.msg import Odometry
from math import radians, sin, cos, atan2, copysign

class DeadReckoning():
    def __init__(self, goal_distance, goal_rotation, linear_speed, angular_speed):
        rospy.init_node('dead_reckoning')
        rate = rospy.get_param('~rate', 10)
        xy_goal_tolerance = rospy.get_param('~xy_goal_tolerance', 0.05)
        yaw_goal_tolerance = rospy.get_param('~yaw_goal_tolerance', 0.05)
        
        self.goal_distance = goal_distance
        self.goal_rotation = goal_rotation
        
        r = rospy.Rate(rate)
        
        rospy.loginfo("Moving Base to Target")

        self.start_pose = Pose()      
        self.pose = Pose()
        self.odom_started = False
        cmd_vel_msg = Twist()
        
        linear_max_speed = 0.5
        if linear_speed > linear_max_speed:
            linear_speed = linear_max_speed
            
        angular_max_speed = 1.0
        if angular_speed > angular_max_speed:
            angular_speed = angular_max_speed
        
        if goal_distance == 0:
            xy_goal_reached = True
        else:
            xy_goal_reached = False
            
        if goal_rotation == 0:
            yaw_goal_reached = True
        else:
            yaw_goal_reached = False
                        
#        # Convert the goal rotation to a quaternion
#        quaternion = Quaternion()
#        quaternion.x = 0.0 
#        quaternion.y = 0.0
#        quaternion.z = sin(radians(rotation) / 2.0)
#        quaternion.w = cos(radians(rotation) / 2.0)
        
        # Subscribe to the /odom topic
        rospy.Subscriber('/robot_pose_ekf/odom_combined', Odometry, self.update_odometry)
        cmd_vel_pub = rospy.Publisher('/turtlebot_node/cmd_vel', Twist)

        rospy.wait_for_message('/robot_pose_ekf/odom_combined', Odometry)
        rospy.sleep(1)
                      
        while not rospy.is_shutdown():
            if xy_goal_reached:
                if yaw_goal_reached:
                    cmd_vel_msg.linear.x = 0
                    cmd_vel_msg.angular.z = 0
                    cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.sleep(1)
                    rospy.loginfo("Goal Reached!")
                    rospy.signal_shutdown("Goal Reached!")
                else:
                   cmd_vel_msg.angular.z = angular_speed
            else:
                cmd_vel_msg.linear.x = linear_speed
                cmd_vel_msg.angular.z = 0
       
            cmd_vel_pub.publish(cmd_vel_msg)
            
            # Check to see if the goal has been reached
            if abs(self.goal_position - self.pose.position.x) < xy_goal_tolerance:
                xy_goal_reached = True
            else:
                linear_speed = copysign(linear_speed, self.goal_position - self.pose.position.x)
            
            current_rotation = 2.0 * atan2(self.pose.orientation.z, self.pose.orientation.w)
            if abs(self.goal_rotation - current_rotation) < yaw_goal_tolerance:
                yaw_goal_reached = True
            else:
                angular_speed = copysign(angular_speed, self.goal_rotation - current_rotation)
                
            rospy.loginfo(self.pose)
                
            r.sleep()
                
    def update_odometry(self, msg):
        if not self.odom_started:
            self.start_pose = msg.pose.pose
            self.start_rotation = 2.0 * atan2(self.start_pose.orientation.z, self.start_pose.orientation.w)
            self.goal_rotation = self.start_rotation + self.goal_rotation
            self.goal_position = self.start_pose.position.x + self.goal_distance
            self.odom_started = True
        else:
            self.pose = msg.pose.pose
        
if __name__ == '__main__':
    try:
        DeadReckoning(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    except rospy.ROSInterruptException:
        pass