#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# sonar_to_costmap.py
# ################################################################################
# edited WHS, OJ , 23.12.2020 #
#
# brings Sonar detected Obstacles into move_base local costmap
# using point_cloud - message
#
# edit
# only Gazebo
# copy content of turtlebot3.burger.gazebo_sonar.xacro
#              to turtlebot3.burger.gazebo_sonar.xacro
# copy content of turtlebot3.burger.urdf_sonar.xacro
#              to turtlebot3.burger.urdf.xacro
#
# real Bot and Gazebo
# edit costmap_common_params_burger.yaml
#    observation_sources: scan sonar
#    scan: ...
#    sonar: {sensor_frame: base_link, data_type: PointCloud,
#             topic: /sonar/point_cloud, marking: true, clearing: true}
#
# edit move_base.launch  => /cmd_vel to /move_base/cmd_vel
#     <arg name="cmd_vel_topic" default="/move_base/cmd_vel" />
#
# usage
#   $1 roslaunch turtlebot3_gazebo turtlebot3_house.launch
#   $2 roslaunch turtlebot3_navigation turtlebot3_navigation.launch
#                map_file:=$HOME/catkin_ws/src/rtc/rtc_maps/gazebo_house_map_2020_12_07.yaml
#   $3 roslaunch rts sonar_twist_mux.launch
#   $4 rosrun rtc sonar_obstacle_avoidance.py
#   $5 rosrun rtc sonar_to_costmap.py
# ------------------------------------------------------------------

import rospy
import std_msgs.msg
import rostopic
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Range
import turtlebot3_msgs
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import PointCloud  # Message für die Sonar-Hindernisse
from nav_msgs.msg import Odometry

import math
import statistics


# TODO increase number of points per Sensor to make sure the robot goes way around obstacles

class Sonar_to_Point_Cloud():
    def __init__(self):
        rospy.loginfo("Publishing PointCloud")

        self.cloud_pub = rospy.Publisher('sonar/point_cloud',
                                         PointCloud,
                                         queue_size=10)

        # receive sonars
        self.sonar_sub_left = rospy.Subscriber('sensor_state',
                                               SensorState,
                                               self.get_sonar,
                                               queue_size=10)

        # receive odom for speed
        self.sonar_sub_left = rospy.Subscriber('odom',
                                               Odometry,
                                               self.update_odom,
                                               queue_size=10)

        self.ros_hz = rostopic.ROSTopicHz(-1)
        rospy.Subscriber('/sensor_state', rospy.AnyMsg, self.ros_hz.callback_hz, callback_args='/sensor_state')
        self.dist_sonars3 = 0.0
        self.dist_sonars1 = 0.0


        self.window_size = 20 
        self.padding = 5
        self.dists_sonars3 = [0.0] * self.window_size 

        self.old_dists3 = [-1] * 2

        self.dists_sonars1 = [0.0] * self.window_size 

        self.old_dists1 = [-1] * 2

        #rospy.loginfo(f" Sonar Data sonars1: {self.dists_sonars1}")

        self.robot_speed = 0.11 # in m/s # TODO get actual movespeed of robot
        self.robot_twist = 0.0
        self.sonar_update_frequency = 20 # in Hz # TODO get actual frequency of publishing distance

        self.rate = rospy.Rate(10)

        # only use every xth message
        #self.msg_rate = 5
        #self.msg_counter = 0

        while not rospy.is_shutdown():
            self.rate.sleep()

# TODO filter detection of floor
    
    # check if echo is from floor
    def is_floor(self, current_dist, old_dists):
        # if robot stands still, have to go for a minimum change
        max_change = max(self.robot_speed / self.sonar_update_frequency, 0.01)
        if old_dists[0] == -1 or old_dists[1] == -1:
            floor = False
        else:
            # if sonar distances change faster than the robot moves
            # has to be echo from floor
            floor = abs(old_dists[0] - current_dist) > max_change \
                    or abs(old_dists[1] - old_dists[0]) > max_change
        
        # TODO testcode, when turning this method does not work properly
        # so filter turning
        if self.robot_twist > 0.0:
            floor = False
        
        # TODO for testing
        floor = False

        return floor

    def update_odom(self, odom):
        self.robot_speed = round(odom.twist.twist.linear.x, 2)
        self.robot_twist = round(odom.twist.twist.angular.z, 1)


# TODO floor still gets detected but stable at one of the many distances
# try making sure a distance will only be published as point cloud, if it
# is repeatedly measured
# add distance 0.0 to dists_sonars1 and dist_sonars1 if floor detected
# TODO still unstable, try doing a history of 2 distances for floor checking
    def get_sonar(self, sensor_state):
        try:
            self.sonar_update_frequency = round(self.ros_hz.get_hz('/sensor_state')[0], 0)
        except TypeError:
            # no data, don't need to update frequency
            pass
        #print(f'sonar update frequency: {self.sonar_update_frequency}')
        current_dist_front = sensor_state.sonar / 100
        current_dist_back = sensor_state.cliff / 100
        #rospy.loginfo(f'current dist: {current_dist}')
        dists_front = self.dists_sonars3
        dists_back = self.dists_sonars1
        #if current_dist > 0.0:
        dists_front.pop(0)
        dists_front.append(current_dist_front)
        dists_back.pop(0)
        dists_back.append(current_dist_back)

        sorted_dists_front = sorted(dists_front)
        sorted_dists_back = sorted(dists_back)
        padded_dists_front = sorted_dists_front[self.padding:self.window_size - self.padding]
        padded_dists_back = sorted_dists_back[self.padding:self.window_size - self.padding]
        mean_dist_front = statistics.mean(padded_dists_front)
        mean_dist_back = statistics.mean(padded_dists_back)
        new_dist_front = round(mean_dist_front, 2)
        new_dist_back = round(mean_dist_back, 2)
        #if not self.is_floor(new_dist, self.old_dists3):
        self.dist_sonars3 = new_dist_front
        self.dists_sonars3 = dists_front
        
        self.dist_sonars1 = new_dist_back
        self.dists_sonars1 = dists_back
        # is floor, act as if there was no detection
        #else:
        #    self.dist_sonars3 = 0.0
        #    self.dists_sonars3.pop(0)
        #    self.dists_sonars3.append(self.dist_sonars3)

        self.cloud_build()
        self.old_dists3.pop(0)
        self.old_dists3.append(new_dist_front)
        
        self.old_dists1.pop(0)
        self.old_dists1.append(new_dist_back)

    def cloud_build(self):

        # only use every xth message
        #if not self.msg_counter == self.msg_rate - 1:
        #    return

        # Instanziiere leere PointCloud
        cloud = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()  # Leere Instanz
        header.stamp = rospy.Time.now()  # Fülle Zeitstempel
        header.frame_id = 'base_link'
        cloud.header = header

        # dimensions of robot see 
        # /catkin_ws/src/turtlebot3/turtlebot3_navigation/param/costmap_common_params_burger.yaml
        width = 0.18
        length_forward = 0.04
        length_backward = 0.12
        depth_of_sensor = 0.01
        angle_diagonal = math.radians(45) # convert 45°
        angle_front = math.radians(0)
        angle_left = math.radians(90)
        angle_right = -angle_left
        diagonal_from_mid = (width / 2)**2 + length_forward**2
        # angle between pointcloud points of diagonal sensor
        angle_dist = math.radians(2)

        # maximum distance the sensors should be used
        max_dist = 2.0

        # minimum distance the sensors should be used
        min_dist = 0.01

        point_range = range(-4, 5)

        # sonars3 (vorne) BDPIN_GPIO_5, BDPIN_GPIO_6
        if(self.dist_sonars3 < max_dist and self.dist_sonars3 > min_dist):
            for i in point_range:
                p3 = Point32()
                p3.x = math.cos(angle_front + i * angle_dist) * (self.dist_sonars3 + length_forward)
                p3.y = math.sin(angle_front + i * angle_dist) * (self.dist_sonars3 + length_forward)
                p3.z = 0.0

                cloud.points.append(p3)
        
        if(self.dist_sonars1 < max_dist and self.dist_sonars1 > min_dist):
            for i in point_range:
                p1 = Point32()
                p1.x = - math.cos(angle_front + i * angle_dist) * (self.dist_sonars1 + length_backward)
                p1.y = math.sin(angle_front + i * angle_dist) * (self.dist_sonars1 + length_backward)
                p1.z = 0.0

                cloud.points.append(p1)

        # Senden
        self.cloud_pub.publish(cloud)


if __name__ == '__main__':
    rospy.init_node('sonar_controller', anonymous=True)
    try:
        sonar = Sonar_to_Point_Cloud()
    except rospy.ROSInterruptException:
        pass
