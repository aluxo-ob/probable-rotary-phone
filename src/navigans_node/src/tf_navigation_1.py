[]#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
#import turtlesim.srv
#startpoint is 3.685 15.020
#WP1 : 0.271 11.946
#WP2 1.225 5.456
#WP3 : -2.889, 2.812
if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
    euler_from_quaternion = tf.transformations.euler_from_quaternion
    listener = tf.TransformListener()


    husky_vel = rospy.Publisher('husky1/rcta_teleop/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    rate = rospy.Rate(30.0)
    
    waypoint_list = np.array([[4.244,1.272],[9.744,3.530],[17.734,-0.516]])    
    waypoint_length = waypoint_list.shape[0]
    waypoint_index = int(0)
    #HYPERPARAMETERSs
    initial_threshold = 0.1
    turn_threshold =0.001
    angular_displacement_threshold = initial_threshold
    next_waypoint_threshold = 1
    arrival_threshold = 2
    print('program A')

    cmd = geometry_msgs.msg.Twist()
    
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0
    f_vel = 0.9
    a_vel = 0.9
    
    while not rospy.is_shutdown():
    #while True:
        try:
            (trans,rot) = listener.lookupTransform('/husky1/odom', '/husky1/base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print('nav')
        goalY, goalX = waypoint_list[waypoint_index,0],waypoint_list[waypoint_index,1]
    	currentY = trans[0]#Front is positive
    	currentX = trans[1]#Left is positive
    	heading = euler_from_quaternion(rot)[2]#in3.14, increase counterclockwise
    	#	print("Currently at", format(currentX,'.3f'), format(currentY,'.3f'), "heading", heading)			
    	#print(angles)
    	target_heading = math.atan2(goalX-currentX,goalY-currentY)
        #print("passing", currentY,currentX, "heading", heading, 'tgt at', target_heading)
	print('fvel', cmd.linear.x, 'avel',cmd.angular.z)
        angle_mag = abs(heading-target_heading)
        if angle_mag > angular_displacement_threshold:
            if heading > target_heading:
                cmd.angular.z = -a_vel*abs(heading-target_heading)**2/1
                cmd.linear.x = f_vel
                husky_vel.publish(cmd)
                #print('turning right')
		angular_displacement_threshold = turn_threshold
            else:
                cmd.angular.z = a_vel*abs(heading-target_heading)**2/1
                cmd.linear.x = f_vel
                husky_vel.publish(cmd)

		angular_displacement_threshold = turn_threshold
                #print('turning left')
        else:
            cmd.angular.z = 0
            cmd.linear.x = f_vel
            husky_vel.publish(cmd)
            angular_displacement_threshold = initial_threshold
            print("go straight")

        if (currentX-goalX)**2+(currentY-goalY)**2<next_waypoint_threshold**2:
            waypoint_index =  int(waypoint_index+1)
            print("moving to next waypoint")

        if waypoint_index == waypoint_length-1:
            if (currentX-goalX)**2+(currentY-goalY)**2<arrival_threshold**2:
                cmd.angular.z = 0
                cmd.linear.z = 0
                husky_vel.publish(cmd)
                print("arrived at destination")



	
        rate.sleep()#-2.68,front bumper-2.331
