import rospy
import std_msgs.msg
import rostopic
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Range
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import PointCloud  # to draw obstacles detected by sonar

import math
import statistics


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

        
        # receive frequency of sensor_state topic
        self.ros_hz = rostopic.ROSTopicHz(-1)
        rospy.Subscriber('/sensor_state', 
                         rospy.AnyMsg, 
                         self.ros_hz.callback_hz, 
                         callback_args='/sensor_state')


        self.dist_front = 0.0
        self.dist_back = 0.0

        self.window_size = 20 
        self.padding = 5

        self.dists_front = [0.0] * self.window_size 
        self.dists_back = [0.0] * self.window_size 

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.rate.sleep()


    # get sensor data from sonar sensors and preprocess it
    def get_sonar(self, sensor_state):

        # distance is passed in cm
        # have to convert to meters
        current_dist_front = sensor_state.sonar / 100
        current_dist_back = sensor_state.cliff / 100

        dists_front = self.dists_front
        dists_back = self.dists_back

        # move window
        dists_front.pop(0)
        dists_front.append(current_dist_front)
        dists_back.pop(0)
        dists_back.append(current_dist_back)

        sorted_dists_front = sorted(dists_front)
        sorted_dists_back = sorted(dists_back)

        # remove outliers
        padded_dists_front = sorted_dists_front[self.padding:self.window_size - self.padding]
        padded_dists_back = sorted_dists_back[self.padding:self.window_size - self.padding]

        mean_dist_front = statistics.mean(padded_dists_front)
        mean_dist_back = statistics.mean(padded_dists_back)
        new_dist_front = round(mean_dist_front, 2)
        new_dist_back = round(mean_dist_back, 2)
        
        # limit detection range to filter noise at higher ranges
        max_det_range = 0.4
        
        if new_dist_front > max_det_range:
            new_dist_front = 0.0

        if new_dist_back > max_det_range:
            new_dist_back = 0.0


        self.dist_front = new_dist_front
        self.dists_front = dists_front
        
        self.dist_back = new_dist_back
        self.dists_back = dists_back

        self.cloud_build()

    def cloud_build(self):

        # Instanziiere leere PointCloud
        cloud = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()  # Leere Instanz
        header.stamp = rospy.Time.now()  # Fülle Zeitstempel
        header.frame_id = 'base_link'
        cloud.header = header

        # dimensions of robot see 
        # /catkin_ws/src/turtlebot3/turtlebot3_navigation/param/costmap_common_params_burger.yaml
        length_forward = 0.04
        length_backward = 0.12
        
        # angle of the sensor
        #sensor_angle = math.radians(0)
        # angle between multiple points to be published
        #angle_dist = math.radians(2)
        
        # distance between pointcloud points
        point_dist = 0.01

        # maximum distance the sensors should be used in meters
        max_dist = 2.0

        # minimum distance the sensors should be used in meters
        min_dist = 0.01


        # points to publish
        # range(-x, y)
        # publishes x points to the left
        # and y points to the right
        # of the middle axis of the sensor
        point_range = range(-5, 6)

        if(self.dist_front < max_dist and self.dist_front > min_dist):
            for i in point_range:
                p3 = Point32()
                # uses sin/cos to create a cone of points that gets wider as distance increases
                #p3.x = math.cos(sensor_angle + i * angle_dist) * (self.dist_front + length_forward)
                #p3.y = math.sin(sensor_angle + i * angle_dist) * (self.dist_front + length_forward)

                # creates points at a right angle towards the middle axis of the sensor
                p3.x = self.dist_front + length_forward
                p3.y = point_dist * i * 2
                p3.z = 0.0

                cloud.points.append(p3)
        
        # sonars1 (hinten) BDPIN_GPIO_9, BDPIN_GPIO_10
        if(self.dist_back < max_dist and self.dist_back > min_dist):
            for i in point_range:
                p1 = Point32()
                # uses sin/cos to create a cone of points that gets wider as distance increases
                #p1.x = - math.cos(sensor_angle + i * angle_dist) * (self.dist_back + length_backward)
                #p1.y = math.sin(sensor_angle + i * angle_dist) * (self.dist_back + length_backward)

                # creates points at a right angle towards the middle axis of the sensor
                p1.x = - (self.dist_back + length_backward)
                p1.y = point_dist * i * 2
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
