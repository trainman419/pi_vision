#! /usr/bin/python

import roslib; roslib.load_manifest('pi_actions')
import rospy
from trajectory_msgs.msg import *
from math import pi

class MoveHeadTraj():
    def __init__(self):
        rospy.init_node('move_head_traj', anonymous=True)
        
        rate = rospy.get_param('~rate', 1)
        r = rospy.Rate(rate)
        
        traj_pub = rospy.Publisher('/head_traj_controller/command', JointTrajectory)
                
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = 'head_link'
        trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        trajectory.points.append(JointTrajectoryPoint(positions = [0, 0], velocities = [1, 0.2], time_from_start = rospy.Duration(0.0)))
        
        while not rospy.is_shutdown():
            rospy.loginfo(trajectory)
            traj_pub.publish(trajectory)
            r.sleep()

if __name__ == '__main__':
    try:
        move_head = MoveHeadTraj()
    except rospy.ROSInterruptException:
        pass

#g.trajectory.points.append(
#    JointTrajectoryPoint(positions = [0, 0, 0, -0.15, 0, -0.1, 0], 
#                         velocities = [0, 0, 0, 0, 0, 0, 0],
#                         time_from_start = rospy.Duration(1.0)))
#g.trajectory.points.append(
#    JointTrajectoryPoint(positions = [0, 1.0, 0, -2.0, pi, -1.0, 0], 
#                         velocities = [0, 0, 0, 0, 0, 0, 0],
#                         time_from_start = rospy.Duration(2.0)))

