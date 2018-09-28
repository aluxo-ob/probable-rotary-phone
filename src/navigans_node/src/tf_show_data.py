#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg

#import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
    euler_from_quaternion = tf.transformations.euler_from_quaternion
    listener = tf.TransformListener()

    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    #spawner(4, 2, 0, 'turtle2')

    husky_vel = rospy.Publisher('husky1/rcta_teleop/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    #Gives you a scale of 3.14 
    rate = rospy.Rate(10.0)
    goalX = 1.9
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = 1
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/husky1/odom', '/husky1/base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        currentX = trans[0]#Front is positive
        currentY = trans[1]#Left is positive

        heading = euler_from_quaternion(rot)[2]#in3.14, increase counterclockwise
        print("Currently at", format(currentX,'.3f'), format(currentY,'.3f'), "heading", heading)			
	#print(angles)
		
	
        rate.sleep()#-2.68,front bumper-2.331
