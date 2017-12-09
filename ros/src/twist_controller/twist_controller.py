import rospy
import yaw_controller
import pid
import string
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
KD_accel = 0.01
KI_accel = 0.1
KP_accel = 1.0
KD_yaw = 1
KI_yaw = 1
KP_yaw = 1




class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.last_time = rospy.get_time()
        self.yaw_control = yaw_controller.YawController(
            kwargs['wheel_base'], kwargs['steer_ratio'], ONE_MPH, kwargs['max_lat_accel'], kwargs['max_steer_angle'])
        #self.yaw_PID = PID(KP_yaw, KI_yaw, KD_yaw, -max_steer_angle, max_steer_angle)

        self.brake_deadband = kwargs['brake_deadband']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        self.throttle_PID = pid.PID(KP_accel, KI_accel, KD_accel, 0, 1)

        self.throttle_lpf = LowPassFilter(0.5, 0.05)

    def control(self, desired_velocity_x, desired_vel_angular_z, current_velocity_x, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        actual_time = rospy.get_time()
        throttle = 0
        brake = 0
        steer = self.yaw_control.get_steering( desired_velocity_x, desired_vel_angular_z, current_velocity_x)

        if not dbw_enabled:
            #yaw_PID.reset()
            self.throttle_PID.reset()
            self.last_time = actual_time
            return 0., 0., 0.

        diff_velocity = desired_velocity_x - current_velocity_x
        diff_time = actual_time - self.last_time
        self.last_time = actual_time

        new_accel = min(max(self.decel_limit, diff_velocity), self.accel_limit)

        if diff_velocity > 0:
            brake = 0
            throttle = self.throttle_PID.step(new_accel - self.throttle_lpf.get(), diff_time)

        else:
            throttle = 0
            # Torque  = F * radius -> F = mass * acceleration
            brake = abs(new_accel * self.vehicle_mass * self.wheel_radius)

        #steer = 0
        rospy.loginfo("DV: "+str(desired_velocity_x)+" DA: "+str(desired_vel_angular_z)+" CV: "+str(current_velocity_x)+ " new_accel: "+str(new_accel))        
        
        rospy.loginfo("Steer: "+str(steer)+" Throttle: "+str(throttle)+" Brake: "+str(brake))
        return throttle, brake, steer
