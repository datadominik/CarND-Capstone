import rospy
import yaw_controller
import pid
import string

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
KD = 1
KI = 1
KP = 1
PID_MIN = 0
PID_MAX = 1



class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.last_time = rospy.get_time()
        self.yaw_control = yaw_controller.YawController(
            kwargs['wheel_base'], kwargs['steer_ratio'], 1, kwargs['max_lat_accel'], kwargs['max_steer_angle'])

        self.brake_deadband = kwargs['brake_deadband']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']


    def control(self, desired_velocity_x, desired_vel_angular_z, current_velocity_x, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            return 1.,0.,0.

        steer = 0
        #steer = self.yaw_control.get_steering( desired_velocity_x, desired_vel_angular_z, current_velocity_x)
        #rospy.loginfo("Steer: "+str(steer))
        return 1., 0., 0.
