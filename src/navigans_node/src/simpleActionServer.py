#! /usr/bin/env python

# Implementation of Action Server. 

import roslib
import rospy
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import numpy as np
import tf

from cmu_perception_msgs.msg import ExternalPathAction, ExternalPathGoal, ExternalPathResult, ExternalPathFeedback, TrackedObject, TrackedObjectSet

  
# --------------------------------------------------------
class NaviGANsServer:
  def __init__(self):
    # Instantiate action server
    self.server = actionlib.SimpleActionServer('navigans_local_planner', ExternalPathAction, self.execute, False)
    self.server.start()

    # Provide some feedback now, and get ready to publish feedback as the action is executed
    rospy.loginfo('[%s] initialized...' % rospy.get_name() )
    self.feedback = ExternalPathFeedback()

    # Read parameters from configuration file
    self.someValue = rospy.get_param('~useGPU', -1)
    print self.someValue
    # self.someOtherValue = rospy.get_param('~modelFn')

    self.euler_from_quaternion = tf.transformations.euler_from_quaternion
    self.listener = tf.TransformListener()
    self.husky_vel = rospy.Publisher('husky1/rcta_teleop/cmd_vel', Twist, queue_size=1)
    
  # --------------------------------------------------------
  def execute(self, goal):
    # Main callback function, which is executed when a goal is received
    
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, NaviGAN local planner' % (rospy.get_name() ) )
    
    # Get fields from the goal description    
    print goal.ignore_obstacles
    #print goal.desired_path.poses[0]

    # number of wayppoints included in Path
    print np.shape(goal.desired_path.poses)[0]

    # Main loop; this should be a while loop, as the robot traverses all the waypoints
    while not rospy.is_shutdown():
      try:
        (trans,rot) = self.listener.lookupTransform('/husky1/odom', '/husky1/base', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      currentY = trans[0] # Front is positive
      currentX = trans[1] # Left is positive
      heading = self.euler_from_quaternion(rot)[2]#in3.14, increase counterclockwise
      print( "Robot state: (%.3f, %.3f), < %.5f" % (currentX, currentY, heading) )
      
      """
      for i in range(0, np.shape(goal.desired_path.poses)[0] ):
      print goal.desired_path.poses[ i ]
      break #exit the while loop after you're done
      """
      cmd = Twist()
    
      cmd.linear.y = 0
      cmd.linear.z = 0
      cmd.angular.x = 0
      cmd.angular.y = 0

      cmd.angular.z = 0.0
      cmd.linear.x = 0.0
      self.husky_vel.publish(cmd)

      
    # Publish feedback   
    self.feedback.percent_complete = 0.0101
     
    self.server.publish_feedback(self.feedback)

    # Notify when action has ended successfully
    self.server.set_succeeded()

  # --------------------------------------------------------
  def trackerMsgCallback( self, data ):
    print data.header
    """
    print data.header.frame_id
    for k in range( len(data.objects) ):
      print("Object %d:" % k )
      print data.objects[ k ]
    print "Done\n"
    """

if __name__ == '__main__':
  rospy.init_node('navigans_control_server')

  server = NaviGANsServer()
  rospy.Subscriber( '/forward_lidar_tracking_data', TrackedObjectSet, server.trackerMsgCallback )
  rospy.spin()


