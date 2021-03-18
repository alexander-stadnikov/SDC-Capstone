import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
        decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio,
        max_lat_accel, max_steer_angle):
        
        self.yawController = YawController(wheel_base, steer_ratio, 0.1,
            max_lat_accel, max_steer_angle)
        self.throttleController = PID(kp=0.3, ki=0.1, kd=0, mn=0, mx=0.5)
        self.velocityLpf = LowPassFilter(tau=0.5, ts=0.02)
        self.vehicleMass = vehicle_mass
        self.fuelCapacity = fuel_capacity
        self.brakeDeadband = brake_deadband
        self.decelLimit = decel_limit
        self.accelLimit = accel_limit
        self.wheelRadius = wheel_radius
        self.lastTime = rospy.get_time()


    def control(self, current_velocity, dbw_enabled, linear_velocity, angular_velocity):
        if not dbw_enabled:
            return 0, 0, 0

        current_velocity = self.velocityLpf.filt(current_velocity)
        velocityError = linear_velocity - current_velocity
        
        currentTime = rospy.get_time()
        sampleTime = currentTime - self.lastTime
        self.lastTime = currentTime

        steering = self.yawController.get_steering(linear_velocity, angular_velocity, current_velocity)
        throttle = self.throttleController.step(velocityError, sampleTime)
        brake = 0

        if linear_velocity == 0 and current_velocity < 0.1:
            throttle = 0
            brake = 400
        elif throttle < 0.1 and velocityError < 0:
            throttle = 0
            dec = max(velocityError, self.decelLimit)
            brake = abs(dec) * self.vehicleMass * self.wheelRadius
        return throttle, brake, steering
