#!/usr/bin/env python

import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
FREQUENCY = 20


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.bwp_subscription = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.is_initialized = False
        self.base_waypoints = []
        self.num_wps = -1
        self.current_pos = None
        self.next_wp_idx = -1
        self.msg_seq = 0

        self.run()

    def pose_cb(self, msg):
        """
        :parm msg Contains the the current position of our car.
        A position msg looks like the following:
        header:
          seq: 1142
          stamp:
            secs: 1512299036
            nsecs: 206657886
          frame_id: /world
        pose:
          position:
            x: 1131.22
            y: 1183.27
            z: 0.1069651
          orientation:
            x: 0.0
            y: 0.0
            z: 0.0436201197059
            w: 0.999048189607

	    """
        self.current_pos = msg.pose
        #self.is_initialized = True and len(self.base_waypoints) > 0

    def waypoints_cb(self, waypoints):
        """
        header:
            ...
        waypoints:
            pose:
              header:
                ...
              pose:
                position:
                  x: 909.48
                  y: 1128.67
                  z: 0.0
                orientation:
                  x: 0.0
                  y: 0.0
                  z: 0.0
                  w: 1.0
            twist:
              header:
                ...
              twist:
                linear:
                  x: 5.14739724942
                  y: 0.0
                  z: 0.0
                angular:
                  x: 0.0
                  y: 0.0
                  z: 0.0
        :param waypoints:
        :return:
        """
        # The base waypoints should not change at all. So taking them once
        # at the beginning should be sufficient
        if len(self.base_waypoints) == 0:
            self.base_waypoints = waypoints.waypoints
            self.num_wps = len(self.base_waypoints)
            #self.is_initialized = True and self.current_pos != None

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def run(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            if self.current_pos != None and len(self.base_waypoints) > 0:
                self.next_wp_idx = self.get_next_waypoint(self.current_pos)

                if (self.next_wp_idx + LOOKAHEAD_WPS > self.num_wps):
                    rospy.logdebug("New round has started")

                final_wps = [self.base_waypoints[(i + self.next_wp_idx) % self.num_wps] for i in range(LOOKAHEAD_WPS)]
                self.publish_final_waypoints(final_wps)

            rate.sleep()

    def publish_final_waypoints(self, final_waypoints):
        msg = Lane()
        msg.header = self.create_header()
        msg.waypoints = final_waypoints
        self.final_waypoints_pub.publish(msg)

    def create_header(self):
        header = std_msgs.msg.Header()
        header.seq = self.msg_seq
        header.stamp = rospy.Time.now()
        header.frame_id = '/world'
        self.msg_seq += 1
        return header

    def get_closest_waypoint(self, x, y):
        """
        Searches the closest waypoint
        :param x:
        :param y:
        :return:
        """
        closest_len = 100000.0;
        closest_waypoint_idx = -1;

        start_idx = 0 if self.next_wp_idx == -1 else self.next_wp_idx

        wp_x = 0
        wp_y = 0
        # Iterate over all waypoints and calculate the distance. Take possible overflows into account!!!
        for i in range(self.num_wps):
            next_idx = (i + start_idx) % self.num_wps
            wp_pose = self.base_waypoints[next_idx].pose.pose.position
            dist = math.sqrt((wp_pose.x-x)**2 + (wp_pose.y-y)**2)

            if dist < closest_len:
                closest_waypoint_idx = next_idx
                closest_len = dist
                wp_x = wp_pose.x
                wp_y = wp_pose.y

        # TODO: Check waypoint_idx. Could be still -1!!
        if closest_waypoint_idx == -1:
            ropsy.logerror("Couldn't find closest waypoint")

        return closest_waypoint_idx

    def get_next_waypoint(self, current_pos):
        """
        Searches the next closest waypoint that is in front of our car. The search
        is based on the car's current position
        :param current_pos:
        :return:
        """

        car_x = self.current_pos.position.x
        car_y = self.current_pos.position.y
        car_theta = math.atan2(self.current_pos.orientation.y, self.current_pos.orientation.x)

        closest_wp_idx = self.get_closest_waypoint(car_x, car_y)
        wp_x = self.base_waypoints[closest_wp_idx].pose.pose.position.x
        wp_y = self.base_waypoints[closest_wp_idx].pose.pose.position.y

        # check heading
        heading = math.atan2((wp_y - car_y), (wp_x - car_x));
        angle = abs(car_theta - heading);

        if angle > math.pi/4:
            closest_wp_idx += 1

        rospy.logdebug("curr_pos: ({0},{1}), closest_wp_idx: {2}, closest_wp: ({3},{4})"
                       .format(car_x, car_y, closest_wp_idx, wp_x, wp_y))
        return closest_wp_idx;

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
