#!/usr/bin/env python

import rospy
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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.bwp_subscription = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.is_initialized = False
        self.base_waypoints = []
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
            self.is_initialized = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def run(self):

        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            if self.is_initialized and self.current_pos != None:
                next_wp_id = self.get_next_waypoint(self.current_pos)

                # TODO: check overflow
                #if next_wp_id + LOOKAHEAD_WPS > len(self.base_waypoints):
                #   THEN take all wps until the end and continue with at the beginning

                final_wps = self.base_waypoints[next_wp_id:next_wp_id+LOOKAHEAD_WPS]
                self.publish_final_waypoints(final_wps)

            rate.sleep()

    def publish_final_waypoints(self, final_waypoints):
        msg = Lane()
        msg.header.seq = self.msg_seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/world'
        msg.waypoints = final_waypoints
        self.final_waypoints_pub.publish(msg)
        self.msg_seq += 1

        #print("published #final_waypoints: {0}, #frame {1}".format(len(final_waypoints), self.msg_seq))
        print("Published: ", msg.waypoints[0])

    def create_header(self, msg):
        msg.header.seq = self.msg_seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/world'
        return msg

    def get_closest_waypoint(self, x, y):
        """
        Searches the closest waypoint
        :param x:
        :param y:
        :return:
        """
        closest_len = 100000.0;
        closest_waypoint_idx = -1;

        # Iterate over all waypoints and calculate the distance
        for i in range(len(self.base_waypoints)):
            wp_pose = self.base_waypoints[i].pose.pose.position
            dist = math.sqrt((wp_pose.x-x)**2 + (wp_pose.y-y)**2)

            if dist < closest_len:
                closest_waypoint_idx = i
                closest_len = dist

        # TODO: Check waypoint_idx. Could be still -1!!

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
