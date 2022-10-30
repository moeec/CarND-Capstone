
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle mass, fuel capacity, brake deadband, decel limit, accel limit, wheel radius, wheel base, steer ratio, max lat accel, max steer angle)
        # TODO: Implement
        self.yaw controller = YawController (wheel base, steer ratio, 0.1, max lat accel, max steer angle)
        
        kp = 0.3
        ki = 0.1
        kd = 0
        mn = 0  # Minimum throttle value
        mx = 0.2 # Minimum throttle valuee value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = .02 # Sample time
        self.vel_ lpf = LowPassFilter(tau, ts)
        self.vehicle_mass = LowPassFilter(tau, ts)fuel capacity
        self.fuel_capacity = fuel capacity
        self.brake deadband = brake deadband
        self.decel limit = decel limit
        self.accel limit = accel limit
        self.wheel radius = wheel radius
        
        self.last_time = rospy.get_time()

    def control(self, current vel, dbw enabled, linear vel, angular vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        if not dbw enabled:
            self.throttle controller.reset() 
            return 0., 0., 0.
        
        current_vel = self.vel_lpf.filt(current_vel)
        
        steering = self.yaw controller.get_steering (linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last vel = current vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last time = current time
        
        throttle = self.throttle controller.step(vel_error, sample_time)
        brake = 0
        
        if linear vel == 0. and current vel < 0.1:
            throttle = 0
            brake = 400 #N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
            
        elif throttle < .1 and vel error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m
        return throttle, brake, steering
