#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'naviGANs.ui'
#
# Created: Tue Sep 25 12:02:41 2018
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

import roslib
import rospy
import actionlib
import math
import tf

from cmu_perception_msgs.msg import ExternalPathAction, ExternalPathGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from std_msgs.msg import Header

from PyQt4 import QtCore, QtGui
import numpy as np

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8( s ):
        return s
    
try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate( context, text, disambig ):
        return QtGui.QApplication.translate( context, text, disambig, _encoding )
except AttributeError:
    def _translate( context, text, disambig ):
        return QtGui.QApplication.translate( context, text, disambig )


class Ui_Form( object ):
    def setupUi( self, Form ):
        Form.setObjectName( _fromUtf8("Form") )
        Form.resize( 585, 354 )
        """
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setBold(False)
        #font.setWeight(75)
        Form.setFont(font)
        """
        self.pbClearList = QtGui.QPushButton( Form )
        self.pbClearList.setGeometry( QtCore.QRect(30, 310, 85, 23) )
        font = QtGui.QFont()
        font.setPointSize( 11 )
        font.setBold( True )
        font.setWeight( 75 )
        self.pbClearList.setFont( font )
        self.pbClearList.setObjectName( _fromUtf8("pbClearList") )
        self.pbStop = QtGui.QPushButton( Form )
        self.pbStop.setGeometry( QtCore.QRect(466, 30, 85, 23) )
        font = QtGui.QFont()
        font.setPointSize( 11 )
        font.setBold( True )
        font.setWeight( 75 )
        self.pbStop.setFont( font )
        self.pbStop.setObjectName( _fromUtf8("pbStop") )
        self.pbExecutePath = QtGui.QPushButton( Form )
        self.pbExecutePath.setGeometry( QtCore.QRect(360, 30, 101, 23) )
        font = QtGui.QFont()
        font.setPointSize( 11 )
        font.setBold( True )
        font.setWeight( 75 )
        self.pbExecutePath.setFont( font )
        self.pbExecutePath.setObjectName( _fromUtf8("pbExecutePath") )
        self.pbDeleteWP = QtGui.QPushButton( Form )
        self.pbDeleteWP.setGeometry( QtCore.QRect(200, 310, 85, 23) )
        font = QtGui.QFont()
        font.setPointSize( 11 )
        font.setBold( True )
        font.setWeight( 75 )
        self.pbDeleteWP.setFont( font )
        self.pbDeleteWP.setObjectName( _fromUtf8("pbDeleteWP") )
        self.wpLabel = QtGui.QLabel( Form )
        self.wpLabel.setGeometry( QtCore.QRect(10, 10, 231, 16) )
        font = QtGui.QFont()
        font.setPointSize( 12 )
        font.setBold( True )
        font.setWeight( 75 )
        self.wpLabel.setFont( font )
        self.wpLabel.setObjectName( _fromUtf8("wpLabel") )
        self.txtStatus = QtGui.QTextEdit( Form )
        self.txtStatus.setGeometry( QtCore.QRect(335, 60, 241, 270) )
        self.txtStatus.setObjectName( _fromUtf8("txtStatus") )
        self.txtStatus.setReadOnly( True )
        self.tableWPlist = QtGui.QTableWidget( Form )
        self.tableWPlist.setGeometry( QtCore.QRect(10, 30, 301, 271) )
        self.tableWPlist.setObjectName( _fromUtf8("tableWPlist") )
        self.tableWPlist.setColumnCount( 3 )
        self.tableWPlist.setRowCount( 0 ) 
        self.tableWPlist.setHorizontalHeaderLabels( _fromUtf8("No.;X;Y").split(";") )
        font.setPointSize( 9 )
        font.setBold( False )
        self.tableWPlist.setFont( font )
        self.txtStatus.setFont( font )

        self.waypointCount = 0
        self.wpList = np.zeros( (1, 3) )
        now = rospy.get_rostime()
        self.startTime = now.secs + now.nsecs / 1e9

        self.retranslateUi( Form )
        QtCore.QObject.connect( self.pbStop, QtCore.SIGNAL( _fromUtf8("clicked()") ), self.actionStop )
        QtCore.QObject.connect( self.pbExecutePath, QtCore.SIGNAL( _fromUtf8("clicked()") ), self.actionExecute )
        QtCore.QObject.connect( self.pbClearList, QtCore.SIGNAL( _fromUtf8("clicked()") ), self.clearWPlist )
        QtCore.QObject.connect( self.pbDeleteWP, QtCore.SIGNAL( _fromUtf8("clicked()") ), self.deleteWP )
        QtCore.QMetaObject.connectSlotsByName( Form )

        # Need to 'click' coordinate in sensor frame, but then these should be published in /odom frame
        #self.euler_from_quaternion = tf.transformations.euler_from_quaternion
        #self.listener = tf.TransformListener()

    def actionStop( self ):
        self.client.cancel_all_goals()
        print "actionStop"
        self.updateLog("Stop execution !!")

    def actionExecute( self ):
        print "actionExecute"    
        """
        self.goal.desired_path.poses = []
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
        self.goal.desired_path.poses.append( aPoseStamped )
        #goal.desired_path.poses.append( aPoseStamped )
        self.goal.desired_path.poses.append( aPoseStamped )
        print rospy.get_rostime()                
        """

        # Fill in the goal here
        self.client.send_goal( self.goal )

        # Publish list as a simple topic, not action-related
        self.goalsListPub.publish(self.goalsList)

        if self.client.wait_for_result( rospy.Duration.from_sec(2.0) ):
            self.client.get_result()
            self.updateLog("Goal has been sent")
        else:
            self.client.cancel_all_goals()
            print('        the action timed-out')            
            self.updateLog("Goal could not be sent")

    def clearWPlist( self ):
        print "clearWP list"
        while self.waypointCount > 0:
            self.tableWPlist.removeRow( 0 )
            self.waypointCount -=1
        self.waypointCount = 0
        self.wpList = np.zeros( (1, 3) )
        self.updateLog( "Cleared list of waypoints" )
        self.goal.desired_path.poses = []
 
    def deleteWP( self ):
        print "deleteWP"
        #self.tableWPlist.setItem(0,1,QtGui.QTableWidgetItem("3.1416"))
        indexes = self.tableWPlist.selectionModel().selectedRows()
        for index in sorted(indexes ):
            print(' Row %d is selected' % index.row() )
        self.goal.desired_path.poses = []

    def retranslateUi( self, Form ):
        Form.setWindowTitle(_translate("Form", "NaviGANs Client", None))
        self.pbClearList.setText(_translate("Form", "Clear list", None))
        self.pbStop.setText(_translate("Form", "STOP", None))
        self.pbExecutePath.setText(_translate("Form", "Execute Path", None))
        self.pbDeleteWP.setText(_translate("Form", "Delete WP", None))
        # self.wpLabel.setText(_translate("Form", "<html><head/><body><p>List of way points to traverse:</p></body></html>", None))
        self.wpLabel.setText(_translate("Form", "List of way points to traverse:", None))
        
    def initializeAction( self ):
        self.client = actionlib.SimpleActionClient('navigans_local_planner', ExternalPathAction)
        self.client.wait_for_server( rospy.Duration(2.0) )
        self.goal                           = ExternalPathGoal()
        self.goal.ignore_obstacles          = False
        self.goal.global_planning_use_poses = False
        self.goal.global_planning           = False
        self.goal.radius = []
        self.goal.desired_path.poses = []
        self.visualFrame = rospy.get_param('/navigans_path/visual_frame')
        self.targetFrame = rospy.get_param('/navigans_path/target_frame')
        self.posesListTopic = rospy.get_param('/navigans_path/poses_list_topic')
        self.goalsList      = Path()
        self.goalsListPub   = rospy.Publisher(self.posesListTopic, Path, queue_size=1)

    def wayPointCallback( self, data ):
        print "Received waypoint from RViz"
        print("( %.3f, %.3f, %.3f )" % (data.pose.position.x, data.pose.position.y, data.pose.position.z))
        print("[%.4f, %.4f, %.4f, %.4f]" % (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
        
        try:
            # lookupTransform(target_frame, source_frame, time) -> (position, quaternion)
            # (trans,rot) = listener.lookupTransform('/husky1/odom', '/velodyne', rospy.Time())
            (trans,rot) = listener.lookupTransform(self.targetFrame, self.visualFrame, rospy.Time())
            msgTime = rospy.Time.from_sec( data.header.stamp.secs + data.header.stamp.nsecs/1e9 )
            #(trans,rot) = listener.lookupTransform('/husky1/odom', '/velodyne', msgTime )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Exception thrown by TF"
            return

        print "Transform waypoint from /velodyne to /husky1/odom"
        """
        goalPoint                 = PointStamped()
        goalPoint.header.frame_id = "velodyne"
        goalPoint.header.stamp    = rospy.Time(0)
        goalPoint.point           = data.pose.position
        p = listener.transformPoint("/husky1/odom", goalPoint)
        """
        data.header.stamp = rospy.Time(0)
        #odomData = listener.transformPose('/husky1/odom', data)
        odomData = listener.transformPose(self.targetFrame, data)
        #print("( %.3f, %.3f, %.3f )" % (data.pose.position.x, data.pose.position.y, data.pose.position.z))        
        
        p = odomData.pose.position
        q = odomData.pose.orientation
        """
        aPose = Pose()
        aPose.position.x = 0.0
        aPose.position.y = 0.0
        aPose.position.z = 0.0
        aPose.orientation.w = -1.0
        aPose.orientation.x = 0.0
        aPose.orientation.y = 0.0
        aPose.orientation.z = 0.0
        """
        aPoseStamped = PoseStamped()
        aPoseStamped.pose = odomData.pose # aPose
        
        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        
        aPoseStamped.header.stamp.secs  = now.secs
        aPoseStamped.header.stamp.nsecs = now.nsecs
        aPoseStamped.header.frame_id = self.targetFrame
        self.goal.desired_path.poses.append( aPoseStamped )
        self.goalsList.poses.append( aPoseStamped )
        """
        newData = np.array( [data.pose.position.x, data.pose.position.y, data.pose.position.z] )
        if self.waypointCount == 0:
            self.wpList = newData
        else:
            self.wpList = np.vstack( (self.wpList, newData) )
        self.tableWPlist.setRowCount( self.waypointCount + 1 )
        self.tableWPlist.setItem( self.waypointCount,0,QtGui.QTableWidgetItem(str( self.waypointCount + 1 )))
        self.tableWPlist.setItem( self.waypointCount,1,QtGui.QTableWidgetItem(str( data.pose.position.x )))
        self.tableWPlist.setItem( self.waypointCount,2,QtGui.QTableWidgetItem(str( data.pose.position.y )))
        self.tableWPlist.setRowHeight( self.waypointCount, 20 )
        self.tableWPlist.setColumnWidth( 0, 83 )

        data = "(%.3f, %.3f, %.3f)" % (data.pose.position.x, data.pose.position.y, data.pose.position.z)
        self.updateLog(" --> " + data)
        """

        # Update waypoint table in GUI
        newData = np.array( [p.x, p.y, p.z] )
        if self.waypointCount == 0:
            self.wpList = newData
        else:
            self.wpList = np.vstack( (self.wpList, newData) )
        self.tableWPlist.setRowCount( self.waypointCount + 1 )
        self.tableWPlist.setItem( self.waypointCount,0,QtGui.QTableWidgetItem(str( self.waypointCount + 1 )))
        self.tableWPlist.setItem( self.waypointCount,1,QtGui.QTableWidgetItem(str( p.x )))
        self.tableWPlist.setItem( self.waypointCount,2,QtGui.QTableWidgetItem(str( p.y )))
        self.tableWPlist.setRowHeight( self.waypointCount, 20 )
        self.tableWPlist.setColumnWidth( 0, 83 )

        Data = "(%.3f, %.3f, %.3f)" % (p.x, p.y, p.z)
        self.updateLog(" --> " + Data)        
        
        self.waypointCount += 1
        #print("There are {} waypoint in {} array").format(self.waypointCount, self.wpList)



    def updateLog( self, message ):
        now = rospy.get_rostime()
        #status = str( now.secs+now.nsecs/1e9 - self.startTime ) + ": " + message
        logTime = "[%.4f]" % (now.secs+now.nsecs/1e9 - self.startTime)
        status = str(logTime) + ": " + message
        # https://www.python-course.eu/python3_formatted_output.php
        self.txtStatus.append( status )

if __name__ == "__main__":
    import sys
    rospy.init_node( 'navigans_control_client' )

    app = QtGui.QApplication( sys.argv )
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi( Form )
    ui.initializeAction()
    rospy.Subscriber( '/move_base_simple/goal', PoseStamped, ui.wayPointCallback )

    listener = tf.TransformListener()

    Form.show()
    sys.exit( app.exec_() )


