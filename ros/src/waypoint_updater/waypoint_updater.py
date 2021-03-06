#!/usr/bin/env python

import threading
import rospy
import std_msgs.msg
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
FREQUENCY = 10       # Update rate in Hz
MPH2MPS = 0.44704    # Mile per hour to meters per second conversion factor

# The car's state
INIT = 0        # Stays in this state until the base_waypoints and the first position habe been received
DRIVE = 1       # The car will be in the DRIVE state as long as there is no RED traffic light in front of it
WAIT = 2        # The car will be in the WAIT state if a RED traffic light has been detected. It will stay in
                # this state until the traffic light is GREEN again.
STOP = 3        # Final state. Vehicle stops.

SAFETY_DISTANCE = 3  # Safety distance to the closest waypoint of the traffic light stopline

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.INFO)

        # Subscriptions
        self.bwp_subscription = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.actual_velocity_cb)

        # Publisher: final_waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables
        self.v_limit = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        self.decel_limit = -1.0*rospy.get_param('/dbw_node/decel_limit')
        self.accel_limit = rospy.get_param('/dbw_node/accel_limit')

        self.current_state = INIT       # initial state of the statemachine
        self.base_waypoints = None      # a list of all waypoint
        self.final_wps = None           # final waypoints sent to \final_waypoints
        self.num_wps = -1               # total number of all waypoints
        self.current_pos = None         # current position of the vehicle
        self.prev_wp_idx = -1           # previous next closest waypoint ahead of the vehicle
        self.next_wp_idx = -1           # next closest waypoint ahead of the vehicle
        self.msg_seq = 0                # message sequence number counter
        self.next_tl_wp = -1            # waypoint index of the next upcoming traffic light
        self.final_wp = -1
        self.lock = threading.Lock()

        self.loggedStopEvent = False

        # Start processing loop
        self.run()

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def actual_velocity_cb(self, msg):
        current_velocity_x = msg.twist.linear.x


        if current_velocity_x < 0.0001:
            # if self.current_state == WAIT:
            #     rospy.logwarn("Wait_waypoint: {0}, tl_waypoint: {1}".format(self.next_wp_idx, self.next_tl_wp))
            if self.current_state == STOP and self.loggedStopEvent == False:
                rospy.logwarn("Stopped_waypoint: {0}, final_waypoint: {1}".format(self.next_wp_idx, self.final_wp))
                self.loggedStopEvent = True



    def pose_cb(self, msg):
        """
        :parm msg Contains the the current position of our car.
	    """
        self.current_pos = msg.pose
        self.update_state()


    def waypoints_cb(self, waypoints):
        """
        :param waypoints:
        :return:
        """
        # The base waypoints should not change at all. So taking them once
        # at the beginning should be sufficient
        if self.base_waypoints is None:
            self.base_waypoints = waypoints.waypoints
            self.num_wps = len(self.base_waypoints)
            self.final_wp = self.num_wps - 1

            for i in range(self.num_wps):
                self.set_waypoint_velocity(self.base_waypoints, i, 0.0)

            self.update_state()

    def traffic_cb(self, msg):
        with self.lock:
            self.next_tl_wp = msg.data
            #rospy.logwarn("Traffic message " + str(msg.data))
        self.update_state()


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def update_state(self, is_final_state = False):
        """
        Carries out a possible state transition:
        NONE        -> {DRIVE, WAIT}
        DRIVE       -> {WAIT}
        STOPPING    -> {DRIVE}
        _           -> {STOP}
        """
        if self.current_state == INIT:
            if self.current_pos is not None and self.base_waypoints is not None and self.next_tl_wp == -1:
                self.current_state = DRIVE
                rospy.logdebug("NONE -> DRIVE")
            elif self.current_pos is not None and self.base_waypoints is not None and self.next_tl_wp > -1:
                self.current_state = WAIT
                rospy.logwarn("NONE -> WAIT. TRAFFIC LIGHT AT WP: " + str(self.next_tl_wp))
        elif is_final_state:
            self.current_state = STOP
            rospy.logwarn("-> STOP")
        elif self.current_state == DRIVE:
            if self.next_tl_wp > -1:
                self.current_state = WAIT
                rospy.logwarn("DRIVE -> WAIT. TRAFFIC LIGHT AT WP: " + str(self.next_tl_wp))
        elif self.current_state == WAIT:
            if self.next_tl_wp == -1:
                self.current_state = DRIVE
                rospy.logwarn("WAIT -> DRIVE. TRAFFIC LIGHT AT WP: " + str(self.next_tl_wp))

    def run(self):
        rate = rospy.Rate(FREQUENCY)

        while not rospy.is_shutdown():
            if self.current_state == DRIVE:
                final_wps = self.get_final_waypoints()
                v_curr = self.get_waypoint_velocity(final_wps[0])
                v_start = v_curr if v_curr > 0.00001 else 1.0
                self.accelerate(final_wps, v_start)
                self.publish_final_waypoints(final_wps)
            elif self.current_state == WAIT:
                with self.lock:
                    _next_tl_wp_idx = self.next_tl_wp

                if _next_tl_wp_idx > -1:
                    final_wps = self.get_final_waypoints()
                    tl_wp_idx_in_fwps = _next_tl_wp_idx - self.next_wp_idx
                    # if we overshoot, i.e. stopped ofter the traffic_light stopline waypoint
                    # then tl_wp_idx_in_fwps could be negative which means that we can safely set
                    # the velocity of all final_waypoints to zero -> set tl_wp_idx_in_fwps = 0
                    tl_wp_idx_in_fwps = tl_wp_idx_in_fwps if tl_wp_idx_in_fwps >= 0 else 0
                    self.deccelerate(final_wps, tl_wp_idx_in_fwps, SAFETY_DISTANCE, self.next_wp_idx)
                    self.publish_final_waypoints(final_wps)
                else:
                    rospy.logwarn("Invalid traffic waypoint index: " + str(_next_tl_wp_idx))
            elif self.current_state == STOP:
                final_wps = self.get_final_waypoints()
                final_wp_idx = self.final_wp - self.next_wp_idx
                self.deccelerate(final_wps, final_wp_idx, SAFETY_DISTANCE, self.next_wp_idx)
                self.publish_final_waypoints(final_wps)

            rate.sleep()

    def get_final_waypoints(self):
        if self.next_wp_idx > -1:
            self.prev_wp_idx = self.next_wp_idx

        self.next_wp_idx = self.get_next_waypoint(self.current_pos)
        final_wps = [self.base_waypoints[(i + self.next_wp_idx) % self.num_wps] for i in range(LOOKAHEAD_WPS)]

        if self.current_state != STOP and (self.next_wp_idx < self.final_wp < self.next_wp_idx + LOOKAHEAD_WPS):
            self.update_state(is_final_state=True)

        return final_wps

    def accelerate(self, final_wps, v_start):
        """
        Accelerates the vehicle until it has reached the speed limit or
        the last waypoint in the final_wps list
        :param final_wps: The final_waypoints which will be published
        :param v_start: The start velocity
        :return:
        """

        self.set_waypoint_velocity(final_wps, 0, v_start)

        for i in range(1, len(final_wps)):
            # how fast can we go with const acceleration
            # s = (a*t^2) / 2
            # t = sqrt((2*s) / a)
            # v = a * t
            s = self.distance(final_wps, 0, i)
            a = self.accel_limit
            t = math.sqrt((2.0 * s) / a)
            v = v_start + (a * t) * MPH2MPS

            if v > self.v_limit:
                v = self.v_limit

            # if not self.has_accelerated:
            #     x,y,z = self.get_position(final_wps[i].pose.pose.position)
            #     rospy.logdebug("accel: s:{0}, t:{1}, v:{2}, a:{3}, x:{4}, y:{5}".format(s,t,v,a,x,y))

            self.set_waypoint_velocity(final_wps, i, v)

    def deccelerate(self, final_wps, tl_wp_idx, safety_dist, start_idx):
        """
        Deccelerates until vehicle has stopped.
        :param final_wps: final waypoints
        :param tl_wp_idx: traffic light index in the final_wps list
        :param safety_dist: a safety distance to the traffic light. we should definitely stop there
        :return:
        """
        len_fwps = len(final_wps)
        assert(len_fwps > tl_wp_idx)

        stop_wp_idx = -1

        # iterate over final_waypoints in reversed order
        # set velocity to 0 in all waypoints up to the tl_wp_idx - safety_dist
        for i in range(len_fwps-1, -1, -1):
            # waypoint is after traffic light waypoint
            if i >= tl_wp_idx:
                self.set_waypoint_velocity(final_wps, i, 0.0)
            # waypoint is within the safety distance
            elif i < tl_wp_idx and self.distance(final_wps, i, tl_wp_idx) < safety_dist:
                self.set_waypoint_velocity(final_wps, i, 0.0)
            else:
                # from now on we must reduce velocity or in other words:
                # increase velocity with const acceleration as we're going backwards
                if stop_wp_idx == -1:
                    stop_wp_idx = i+1

                # 1.) s = (a*t^2) / 2
                # 2.) from 1.) we get -> t = sqrt((2*s) / a)
                # 3.) v = a * t

                # calc dist between stop_wp and this wp
                s = self.distance(final_wps, i, stop_wp_idx)
                a = self.decel_limit
                t = math.sqrt((2.0*s)/a)
                v = (a * t) * MPH2MPS

                if v < self.get_waypoint_velocity(final_wps[i]):
                    self.set_waypoint_velocity(final_wps, i, v)
                    #rospy.loginfo("deccel: s:{0}, t:{1}, v:{2}, a:{3}".format(s, t, v, a))
                else:
                    break

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
            rospy.logerror("Couldn't find closest waypoint")

        return closest_waypoint_idx

    def get_current_yaw(self):
        """
        Taken from: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        :return:
        """
        orientation = [
            self.current_pos.orientation.x,
            self.current_pos.orientation.y,
            self.current_pos.orientation.z,
            self.current_pos.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation)
        return euler[2]   # z direction

    def get_next_waypoint(self, current_pos):
        """
        Searches the next closest waypoint that is in front of our car. The search
        is based on the car's current position
        :param current_pos:
        :return:
        """

        car_x, car_y, car_z = self.get_position(self.current_pos.position)
        car_theta = self.get_current_yaw()
        closest_wp_idx = self.get_closest_waypoint(car_x, car_y)

        wp_x, wp_y, _ = self.get_position(self.base_waypoints[closest_wp_idx].pose.pose.position)

        # check heading
        heading = math.atan2((wp_y - car_y), (wp_x - car_x));
        angle = abs(car_theta - heading);

        if angle > math.pi/4:
            closest_wp_idx += 1

        return closest_wp_idx;

    def get_position(self, position):
        x = position.x
        y = position.y
        z = position.z
        return x,y,z

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
