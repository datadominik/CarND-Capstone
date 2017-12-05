#!/usr/bin/env python

import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish. You can change this number
FREQUENCY = 20      # Update rate in Hz

# The car's state
NONE = 0        # Stays in this state until the base_waypoints and the first position habe been received
DRIVE = 1       # The car will be in the DRIVE state as long as there is no RED traffic light in front of it
STOPPING = 2    # The car will be in the STOPPING state if a RED traffic light has been detected. It will stay in
                # this state until the traffic light is GREEN again.

SAFETY_DISTANCE = 10    # The safety distance to the traffic light.
                        # This is 10m in germany as long as there is not stop line

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        # Subscriptions
        self.bwp_subscription = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publisher: final_waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables
        self.current_state = NONE
        self.base_waypoints = []
        self.num_wps = -1
        self.current_pos = None
        self.next_wp_idx = -1
        self.msg_seq = 0
        self.next_tl_wp = -1 # waypoint index of the next upcoming traffic light

        # Start processing loop
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
        self.update_state()
        # rospy.loginfo("cur_pos, {0}, {1}, {2}, {3}, {4}, {5}, {6}"
        #               .format(self.current_pos.position.x, self.current_pos.position.y, self.current_pos.position.z,
        #                       self.current_pos.orientation.x, self.current_pos.orientation.y,
        #                       self.current_pos.orientation.z, self.current_pos.orientation.w))

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
            self.update_state()

    def traffic_cb(self, msg):
        self.next_tl_wp = msg.data
        self.update_state()
        rospy.logdebug("Next traffic light wp: {0}".format(msg.data))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def update_state(self):
        """
        Carries out a possible state transition:
        NONE        -> {DRIVE, STOPPING}
        DRIVE       -> {STOPPING}
        STOPPING    -> {DRIVE}
        """
        if self.current_state == NONE:
            if self.current_pos is not None and len(self.base_waypoints) > 0 and self.next_tl_wp == -1:
                self.current_state = DRIVE
                rospy.logdebug("NONE -> DRIVE")
            elif self.current_pos is not None and len(self.base_waypoints) > 0 and self.next_tl_wp > -1:
                self.current_state = STOPPING
                rospy.logdebug("NONE -> STOPPING")

        if self.current_state == DRIVE:
            if self.next_tl_wp > -1:
                self.current_state = STOPPING
                rospy.logdebug("DRIVE -> STOPPING")

        if self.current_state == STOPPING:
            if self.next_tl_wp == -1:
                self.current_state = DRIVE
                rospy.logdebug("STOPPING -> DRIVE")


    def run(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            if self.current_state == DRIVE:
                final_wps = self.get_final_waypoints()
                self.publish_final_waypoints(final_wps)

            elif self.current_state == STOPPING:
                # A red traffic light has been detected. Get the nearest waypoint to the traffic_light.
                # We want to stop our car before a certain safety distance to the traffic light.
                # 1.) Calc stop_point: stop_point = tf_nearest_wp - safety_distance
                # 2.) Get final_waypoints. Integrate stop_point into the list and update waypoint veclocities
                #     until velocity == 0 at stop_point.
                #     Set all subsequent waypoint velocities in the final_waypoints list to 0
                final_wps = self.get_final_waypoints()

                # check if tl_wp is in the final_wps
                if self.next_wp_idx <= self.next_tl_wp and self.next_wp_idx < self.next_wp_idx + LOOKAHEAD_WPS:
                    rospy.logdebug("Traffic wp is in final_waypoints list: {0} <= {1} < {2}"
                                   .format(self.next_wp_idx, self.next_tl_wp, self.next_wp_idx + LOOKAHEAD_WPS))

                    # traffic light waypoint index in the final_weypoints list
                    tl_wp_idx_in_fwps = self.next_tl_wp - self.next_wp_idx
                    self.slow_down(final_wps, tl_wp_idx_in_fwps, SAFETY_DISTANCE)
                else:
                    rospy.logwarn("Traffic wp is not in final_waypoints list: {0} <= {1} < {2}"
                                   .format(self.next_wp_idx, self.next_tl_wp, self.next_wp_idx + LOOKAHEAD_WPS))

                rospy.logdebug("In STOPPING state")

            rate.sleep()

    def get_final_waypoints(self):
        self.next_wp_idx = self.get_next_waypoint(self.current_pos)
        final_wps = [self.base_waypoints[(i + self.next_wp_idx) % self.num_wps] for i in range(LOOKAHEAD_WPS)]
        return final_wps

    def slow_down(self, final_wps, tl_wp_idx, safety_dist):
        """
        Deccelerates
        :param final_wps: final waypoints
        :param tl_wp_idx: traffic light index in the final_wps list
        :param safety_dist: a safety distance to the traffic light. we should definitely stop there
        :return:
        """
        len_fwps = len(final_wps)
        asser(len_fwps > tl_wp_idx)

        # Get velocity of the first wp in final_wps
        wp_v = self.get_waypoint_velocity(final_wps[0])

        stop_wp_idx = -1

        # iterate over final_waypoints in reversed order
        # set velocity to 0 in all waypoints up to the tl_wp_idx - safety_dist
        for i in range(len_fwps-1, 0, -1):
            # waypoint if after traffic light waypoint
            if i >= tl_wp_idx:
                self.set_waypoint_velocity(final_wps, i, 0.0)
            # waypoint is within the safety distance
            elif i < tl_wp_idx and self.distance(final_wps, i, tl_wp_idx) < safety_dist:
                self.set_waypoint_velocity(final_wps, i, 0.0)
            else:
                # from now own we must reduce velocity or in other words:
                # increase velocity with const acceleration as we're going backwards
                if stop_wp_idx == -1:
                    stop_wp_idx = i+1

                # calc dist between stop_wp and this wp
                s = self.distance(final_wps, i, stop_wp_idx)

                # TODO: get real a from global param settings
                a = 3.0

                # how fast can we go with const acceleration
                # s = (a*t^2) / 2
                # t = sqrt((2*s) / a)
                # v = a * t

                t = math.sqrt((2.0*s)/a)
                v = a * t

                rospy.logdebug("s:{0}, t:{1}, v:{2}, a:{3}".format(s,t,v,a))

                # TODO: check if we have already reached the speed limit
                self.set_waypoint_velocity(final_wps, i, v)



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

        rospy.logdebug("curr_pos: ({0},{1}), closest_wp_idx: {2}, closest_wp: ({3},{4})"
                       .format(car_x, car_y, closest_wp_idx, wp_x, wp_y))

        if angle > math.pi/4:
            closest_wp_idx += 1
            #rospy.logdebug("Angle: {0} greater PI/4, new_wp_idx {1}".format(angle, closest_wp_idx))


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
