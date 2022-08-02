#!/usr/bin/env python2.7
import rospy
import numpy as np
#for publisher
from smarc_msgs.msg import GotoWaypoint, GotoWaypointResult

# for listeners
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Header, Bool, Empty
import tf2_geometry_msgs

import tf2_ros
from vision_msgs.msg import ObjectHypothesisWithPose, Detection2DArray, Detection2D



class GoToStart(object):
    def __init__(self,name):
        self.vehicle=name
        print('INIT')
        # Publisher
        self.waypoint_topic = '/{}/algae_farm/wp'.format(self.vehicle)#/sam/algae_farm/wp
        self.waypoint_topic_type = GotoWaypoint #ROS topic type
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, self.waypoint_topic_type,queue_size=5)

        self.enable = Bool()
        self.enable.data = False
        self.enable_pub = rospy.Publisher('/sam/algae_farm/enable', Bool, queue_size=1)

        # Subscriber
        self.rawr = None
        self.yaw_sub = rospy.Subscriber('/{}/dr/yaw'.format(self.vehicle), Float64, self.raw)
        self.current_pose = None
        self.odom_sub = rospy.Subscriber('/{}/dr/odom'.format(self.vehicle), Odometry,self.update_pose)
        self.wp_result = False
        self.waypoint_result_sub = rospy.Subscriber('/{}/ctrl/goto_waypoint/result'.format(self.vehicle), GotoWaypointResult, self.result)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.counter = 0
        self.x_goal = 0
        self.y_goal = 0
        self.x_auv = 0
        self.y_auv = 0
        self.qx_auv = 0
        self.qy_auv = 0
        self.qz_auv = 0
        self.qw_auv = 0
        
        rospy.loginfo('Activated GoToStart node')
        rospy.spin()

    def result(self,msg):
        self.wp_result = msg.result.reached_waypoint
        print('RESULT', self.wp_result)

    def wait_for_transform(self, from_frame, to_frame):
        #tf_buffer = tf2_ros.Buffer()
        trans = None
        while trans is None:
            try:
                trans = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time())
                #trans = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as error:
                print('Failed to transform. Error: {}'.format(error))
        return trans

    def raw(self,msg):
        self.rawr=float(msg.data)

    def transform_pose(self, pose, from_frame, to_frame):
        trans = self.wait_for_transform(from_frame=from_frame,to_frame=to_frame)
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed

    def update_pose(self, msg):
        # might need to transform pose to another frame
        to_frame = 'utm'
        transformed_pose = self.transform_pose(msg.pose, from_frame=msg.header.frame_id, to_frame=to_frame)
        self.current_pose = transformed_pose

        if self.counter == 0:
            self.counter = 1
            # get pose of SAM
            self.x_auv = self.current_pose.pose.position.x
            self.y_auv = self.current_pose.pose.position.y
            self.qx_auv = self.current_pose.pose.orientation.x
            self.qy_auv = self.current_pose.pose.orientation.y
            self.qz_auv = self.current_pose.pose.orientation.z
            self.qw_auv = self.current_pose.pose.orientation.w
            #auv_yaw = self.rawr*180/np.pi
            self.x_goal = self.x_auv
            self.y_goal = self.y_auv + 10

        # publish enable
        self.enable.data = True
        self.enable_pub.publish(self.enable)

        # set STARTING WP to publish
        msg = GotoWaypoint()
        msg.travel_depth = -1
        msg.goal_tolerance = 2
        msg.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH
        msg.speed_control_mode = GotoWaypoint.SPEED_CONTROL_SPEED
        #msg.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
        msg.travel_speed = 1.0
        #msg.travel_rpm = 1000
        msg.pose.header.frame_id = 'utm'
        msg.pose.header.stamp = rospy.Time(0)

        msg.pose.pose.position.x = self.x_goal
        msg.pose.pose.position.y = self.y_goal
        # msg.pose.pose.orientation.x = self.qx_auv
        # msg.pose.pose.orientation.y = self.qy_auv
        # msg.pose.pose.orientation.z = self.qz_auv
        # msg.pose.pose.orientation.w = self.qw_auv

        # publish WP
        self.waypoint_pub.publish(msg) 
        print('PUBLISHED STARTING WP')
        #print(msg)

        if self.wp_result == True:
            # publish not enable
            self.enable.data = False
            self.enable_pub.publish(self.enable)
            print('disabled wp publishing')
            rospy.signal_shutdown('sent starting WP')


def main ():
    rospy.init_node('GoToStart',disable_signals=False)
    #rospy.Rate(5)  # ROS Rate at 5Hz
    rospy.Rate(11.) # ROS Rate at 10Hz
    robot_name = 'sam'
    print('entering GoToStart...')
    initial_nax = GoToStart(robot_name)

    while not rospy.is_shutdown():
        rospy.spin()

    
if __name__ == '__main__':
    main()