from typing import List
from rclpy.context import Context
from rclpy.node import Node
import rclpy
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
import rclpy.time
from std_msgs.msg import Header

import numpy as np

class PID:
    def __init__(self, setpoint):
        self.kp = 0.03 
        self.ki = 0.05
        self.kd = 0.05
        self.setpoint = setpoint
        self.prev_error = 0
        self.bias = 0
        self.integral = 0
        self.windup_upper_lim = 1.0
        self.windup_lower_lim = -1.0

    def updatePID(self,measure,dt):
        error = self.setpoint - measure

        #pid computation
        proportional = error
        self.integral += error * dt #dt = time step between each update
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        if self.integral >= self.windup_upper_lim:
            self.integral -= error * dt
        elif self.integral <= self.windup_lower_lim:
            self.integral -= error * dt

        print(f"P {proportional}, I {self.integral}, D {derivative}")

        #compute the output and print
        output = (self.kp * proportional) + (self.ki * self.integral) + (self.kd * derivative) + self.bias
        print(f"Output: {output}")
        return output
    
class CSController(Node):
    def __init__(self, node_name: str, *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)

        self.current_speed = 0.0

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom/perfect",
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.01, self.pid_update)
        self.control_publisher = self.create_publisher(
            ControlCommand,
            "/control_command",
            10
        )

        setpoint = 10.0 # m/s
        self.dt = 0.01 #s
        self.speed_pid = PID(setpoint)

    
    def odom_callback(self, odometry_msg: Odometry):
        self.current_speed = odometry_msg.twist.twist.linear.x # m/s

    def pid_update(self):
        control_signal = self.speed_pid.updatePID(self.current_speed, self.dt)
        self.get_logger().info(f"Input: {self.current_speed}, control signal {control_signal}")

        control_command = ControlCommand()
        control_command.header = Header()
        control_command.header.stamp = rclpy.time.Time().to_msg()

        control_command.throttle    = control_signal
        control_command.steering    = 0.5
        control_command.brake       = 0.0

        self.control_publisher.publish(control_command)

def main(args=None):
     rclpy.init(args=args)

     node = CSController("CSController")

     rclpy.spin(node)
     rclpy.shutdown()

if __name__ == "__main__":
    main()
