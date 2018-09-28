#! /usr/bin/env python

import roslib
import rospy
import actionlib

from navigans_msgs.msg import ExternalPathAction, ExternalPathGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('navigans_control_client')
    client = actionlib.SimpleActionClient('navigans_local_planner', ExternalPathAction)
    client.wait_for_server()

    goal = ExternalPathGoal()
    goal.ignore_obstacles          = False
    goal.global_planning_use_poses = False
    goal.global_planning           = False
    goal.radius = []
    
    #goal.desired_path = Path()
    goal.desired_path.poses = []
    aPose = Pose()
    aPose.position.x = 0.0
    aPose.position.y = 0.0
    aPose.position.z = 0.0
    aPose.orientation.w = -1.0
    aPose.orientation.x = 0.0
    aPose.orientation.y = 0.0
    aPose.orientation.z = 0.0
    
    aPoseStamped = PoseStamped()
    aPoseStamped.pose = aPose

    now = rospy.get_rostime()
    rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    
    aPoseStamped.header.stamp.secs  = now.secs
    aPoseStamped.header.stamp.nsecs = now.nsecs
    aPoseStamped.header.frame_id = 'odom'
    goal.desired_path.poses.append( aPoseStamped )
    #goal.desired_path.poses.append( aPoseStamped )
    goal.desired_path.poses.append( aPoseStamped )
    print rospy.get_rostime()


    
    # Fill in the goal here
    client.send_goal( goal )
    if client.wait_for_result( rospy.Duration.from_sec(5.0) ):
        client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
    #    return None 
    #client.cancel_goal()
