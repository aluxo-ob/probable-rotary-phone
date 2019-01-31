#! /usr/bin/env python

# Implementation of Action Server. 

import roslib
import rospy
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import numpy as np
import tf
import math as m

from cmu_perception_msgs.msg import ExternalPathAction, ExternalPathGoal, ExternalPathResult, ExternalPathFeedback, TrackedObject, TrackedObjectSet

# --------------------------------------------------------
class TrackingController:
  def __init__(self):
    self.K_x = 0.5
    self.K_y = 0.5
    self.K_theta = 0.25        
    self.active = False
    self.k0 = 1.0
    self.k1 = 1.0
    self.k2 = 1.0
    print "[INFO] Tracking Controller has been instantiated."

  # --------------------------------------------------------
  def setActive(self, state):
    self.active = state

  # --------------------------------------------------------
  def sgn(self, x):
    if x == 0.0:
      return 0.0
    elif x < 0.0:
      return -1.0
    else:
      return 1.0
    
  # --------------------------------------------------------
  def computeCommand(self, x_r, y_r, theta_r, x_c, y_c, theta_c, v_r, w_r):
    error_x = x_r - x_c
    error_y = y_r - y_c
    x_e     = m.cos( theta_c ) * error_x + m.sin( theta_c ) * error_y
    y_e     = m.cos( theta_c ) * error_y - m.sin( theta_c ) * error_x
    theta_e = theta_r - theta_c
    self.v = v_r * m.cos( theta_e ) + self.K_x * x_e
    self.w = w_r + v_r * ( self.K_y * y_e + self.K_theta * m.sin( theta_e ) )

    # Terminate if close to goal
    if m.fabs(error_x) <= 0.5 and m.fabs(error_y) <= 0.5 :
      self.v = 0.0
      self.w = 0.0

  # --------------------------------------------------------
  def computeCommandSMC(self, x_r, y_r, theta_r, v_r, w_r, x_d, y_d, theta_d, v_d, w_d, ):
    error_x = x_r - x_d
    error_y = y_r - y_d
    x_e     = m.cos( theta_d ) * error_x + m.sin( theta_d ) * error_y
    y_e     = m.cos( theta_d ) * error_y - m.sin( theta_d ) * error_x
    theta_e = theta_r - theta_d
    # Sliding surface
    s1 = -v_d + v_r * m.cos( theta_e ) + y_e * w_d + self.k1 * x_e
    s2 = v_r * m.sin( theta_e ) - x_e * w_d + self.k2 * y_e + self.k0 * self.sgn( y_e ) * theta_e
    # Compute reaching law terms
    q1 = 1.0
    q2 = 1.0
    p1 = 1.0
    p2 = 1.0
    r1 = q1 * s1 + p1 * self.sgn( s1 )
    r2 = q2 * s2 + p2 * self.sgn( s2 )
    R  = np.array( [ (-r1,), (-r2,) ] )
    # Compute individual terms of Jacobian matrix
    j11 = self.k1 * m.cos( theta_e ) + w_d * m.sin( theta_e )
    j12 = - v_r * m.sin( theta_e )
    j21 = - w_d * m.cos( theta_e ) + m.sin( theta_e ) * (self.k2 + self.k0 * self.sgn( y_e ) * theta_e )
    j22 = v_r * m.cos( theta_e ) + self.k0 * self.sgn( y_e )
    # Compute the inverse of the Jacobian matrix
    J = np.array( [ (j11, j12), (j21, j22) ] )
    try:
      Jinv = np.linalg.inv( J )
    except np.linalg.LinAlgError:
      # Not invertible. Skip this one.
      pass
    else:
      # continue with what you were doing
      print "Jinv ok"

    # Compute control commands
    u = np.matmul( Jinv, R )
    
    self.v = u[0]
    self.w = u[1]

    # Terminate if close to goal
    if m.fabs(error_x) <= 0.5 and m.fabs(error_y) <= 0.5 :
      self.v = 0.0
      self.w = 0.0   
    
  # --------------------------------------------------------
  def yawFromQuat(self, quat):
    #yaw (z-axis rotation)
    q_w = quat.w
    q_x = quat.x
    q_y = quat.y
    q_z = quat.z
    siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
    cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
    yaw = m.atan2(siny_cosp, cosy_cosp);
    return yaw
      
  
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
    self.targetFrame = rospy.get_param('/navigans_path/target_frame')
    self.sourceFrame = rospy.get_param('/navigans_path/source_frame')
    self.robotCmdTopic = rospy.get_param('/navigans_path/robot_control_topic')
    print self.someValue
    # self.someOtherValue = rospy.get_param('~modelFn')

    self.euler_from_quaternion = tf.transformations.euler_from_quaternion
    self.listener = tf.TransformListener()
    #self.husky_vel = rospy.Publisher('husky1/rcta_teleop/cmd_vel', Twist, queue_size=1)
    self.husky_vel = rospy.Publisher(self.robotCmdTopic, Twist, queue_size=1)
    self.tc = TrackingController()
    
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
      rate = rospy.Rate(5.0) 
      try:
        #(trans,rot) = self.listener.lookupTransform('/husky1/odom', '/husky1/base', rospy.Time(0))
        (trans,rot) = self.listener.lookupTransform(self.targetFrame, self.sourceFrame, rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      currentY = trans[1] # Front is positive
      currentX = trans[0] # Left is positive
      heading = self.euler_from_quaternion(rot)[2]#in3.14, increase counterclockwise
      print( "Robot state: (%.3f, %.3f), < %.5f" % (currentX, currentY, heading) )
      
      """
      for i in range(0, np.shape(goal.desired_path.poses)[0] ):
      print goal.desired_path.poses[ i ]
      break #exit the while loop after you're done
      """
      # Tracking control computeCommand( x_r, y_r, theta_r, x_c, y_c, theta_c, v_r, w_r )
      aGoal   = goal.desired_path.poses[0].pose
      # print aGoal.orientation
      x_r     = aGoal.position.x
      y_r     = aGoal.position.y
      theta_r = self.tc.yawFromQuat( aGoal.orientation )
      x_c     = currentX
      y_c     = currentY
      theta_c = heading
      v_r     = 0.95
      w_r     = 0.0
      # Per Kanayama et al. r = reference; c = current
      #self.tc.computeCommand( x_r, y_r, theta_r, x_c, y_c, theta_c, v_r, w_r )
      #computeCommandSMC(self, x_r, y_r, theta_r, v_r, w_r, x_d, y_d, theta_d, v_d, w_d, )
      self.tc.computeCommandSMC( x_r, y_r, theta_r, v_r, w_r, x_c, y_c, theta_c, v_r, w_r )
      print( "Robot goal: (%.3f, %.3f), < %.5f" % (x_r, y_r, theta_r) )
      print("Cmd: [%.3f, %.4f]" % (self.tc.v, self.tc.w) )
      print "."      
      cmd = Twist()
    
      cmd.linear.y = 0
      cmd.linear.z = 0
      cmd.angular.x = 0
      cmd.angular.y = 0

      cmd.angular.z = self.tc.w
      cmd.linear.x  = self.tc.v
        
      self.husky_vel.publish(cmd)
      rate.sleep()

      if self.tc.w == 0.0 and self.tc.v == 0.0 :
        break
      
    # Publish feedback   
    self.feedback.percent_complete = 1.0
     
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
  trackerTopic = rospy.get_param('/navigans_path/tracking_topic')

  server = NaviGANsServer()
  rospy.Subscriber( trackerTopic, TrackedObjectSet, server.trackerMsgCallback )
  rospy.spin()


