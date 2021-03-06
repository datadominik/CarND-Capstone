#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
from geometry_msgs.msg import TwistStamped
import numpy as np
import yaml



class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.current_velocity_x = None
        self.lights = []

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.is_site = self.config['detector']['is_site']
        rospy.logwarn(self.is_site)
        self.state_count_threshold = 2

        if self.is_site:
            self.min_dist_threshold = 25
        else:
            self.min_dist_threshold = 100

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.last_car_wp = -1
        self.n_ignore = 0

        self.state_count = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        sub5 = rospy.Subscriber('/current_velocity', TwistStamped, self.actual_velocity_cb)


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def actual_velocity_cb(self, msg):
        self.current_velocity_x = msg.twist.linear.x

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        if self.is_site:
            ignore_count = 2
        else:
            ignore_count = 5

        self.has_image = True
        self.camera_image = msg

        self.n_ignore += 1
        if self.n_ignore % ignore_count != 0:
            return

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        if self.state != state:
            self.state_count = 0
            self.state = state

        elif self.state_count >= self.state_count_threshold:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_coordinates(self, poi):
        x = poi.position.x
        y = poi.position.y
        z = poi.position.z
        return x, y, z

    def get_distance(self, poi_1, poi_2):
        x1, y1, z1 = poi_1
        x2, y2, z2 = poi_2
        dist = np.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
        return dist

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        dist = [self.get_distance(self.get_coordinates(pose), self.get_coordinates(waypoint.pose.pose))
                for waypoint in self.waypoints.waypoints]

        min_idx = np.argmin(dist)
        return min_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        detection = False
        if self.current_velocity_x > 0.1:
            detection = True

        #Get classification
        return self.light_classifier.get_classification(cv_image, detection, self.is_site)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if self.waypoints is not None:
            if(self.pose):
                car_waypoint = self.get_closest_waypoint(self.pose.pose)
                car_position = self.get_coordinates(self.pose.pose)

                dist = np.array([self.get_distance(np.array(car_position), self.get_coordinates(light.pose.pose)) for light in self.lights])
                min_dist_idx = np.argmin(dist)
                min_dist = np.min(dist)

                light = self.lights[min_dist_idx]

                stop_line_position = stop_line_positions[min_dist_idx]
                stop_line = Pose()
                stop_line.position.x = stop_line_position[0]
                stop_line.position.y = stop_line_position[1]

                stop_line_waypoint = self.get_closest_waypoint(stop_line)
                light_waypoint = self.get_closest_waypoint(light.pose.pose)

                if min_dist < self.min_dist_threshold and car_waypoint < light_waypoint:
                    state = self.get_light_state(light)

                    if state is None:
                        return stop_line_waypoint, self.last_state
                    else:
                        return stop_line_waypoint, state

                elif min_dist < self.min_dist_threshold:
                    return self.last_wp, self.last_state

                else:
                    return -1, TrafficLight.UNKNOWN

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
