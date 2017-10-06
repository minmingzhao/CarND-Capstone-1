from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, params):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base=params['wheel_base'], 
                                            steer_ratio=params['steer_ratio'] ,
                                            min_speed=params['min_speed'], 
                                            max_lat_accel=params['max_lat_accel'], 
                                            max_steer_angle=params['max_steer_angle'])

    def control(self, target_speed, target_angular_speed, current_speed):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(target_speed, target_angular_speed, current_speed)
        return 0.3, 0., steer
