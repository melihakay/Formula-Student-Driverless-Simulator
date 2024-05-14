from typing import List
from rclpy.context import Context
from rclpy.node import Node
import rclpy
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
import rclpy.time
from std_msgs.msg import Header


class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integrator = 0.0
        self.differentiator = 0.0
        self.prev_measurement = 0.0
        self.T = 0.1               #sampling time of discrete controller in seconds
        self.limMax = 1.0
        self.limMin = 0.0
        self.tau = 0.1 # derivative low-pass filter time constant

    def update(self, measured_value):
        
        #error signal
        error = self.setpoint - measured_value

        #proportional
        self.proportional = self.kp * error

        #integral
        self.integrator += 0.5 * self.ki * self.T * (error * self.prev_error)

        #anti-windup scheme
        limMinInt = 0
        limMaxInt = 0

        if(self.limMax > self.proportional):
           limMaxInt = self.limMax - self.proportional
        else:
           limMaxInt = 0

        if(self.limMin < self.proportional):
            limMinInt = self.limMin - self.proportional
        else:
            limMinInt = 0

        #clamp integrator
        if(self.integrator > limMaxInt):
            self.integrator = limMaxInt
        elif (self.integrator < limMinInt):
            self.integrator = limMinInt

        #Derivative (band limited dfferentiator)
        self.tau = 1
        self.differentiator = ( 2* self.kd * (measured_value - self.prev_measurement)) + (2 * self.tau - self.T) * self.differentiator / (2 * self.tau + self.T)

        #Compute output and apply limits
        print(f"P {self.proportional}, I {self.integrator}, D {self.differentiator}")
        output = self.proportional + self.integrator + self.differentiator
        print(f"Output: {output}")


        if(output > self.limMax):
            output = self.limMax
        elif(output < self.limMin):
            output = self.limMin

        print(f"Clipped output: {output}")

        #Store error and measurement for later use
        self.prev_error = error
        self.prev_measurement = measured_value

        return output


class ConstSpeedController(Node):
    def __init__(self, node_name: str, *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)

        self.current_speed = 0.0

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom/perfect",
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.pid_update)
        self.control_publisher = self.create_publisher(
            ControlCommand,
            "/control_command",
            10
        )

        kp = 0.05
        ki = 0.01
        kd = 0.05
        setpoint = 3.0 # m/s

        self.pid = PIDController(kp, ki, kd, setpoint)

    
    def odom_callback(self, odometry_msg: Odometry):
        self.current_speed = odometry_msg.twist.twist.linear.x # m/s

    def pid_update(self):
        control_signal = self.pid.update(self.current_speed)
        self.get_logger().info(f"Input: {self.current_speed}, control signal {control_signal}")

        control_command = ControlCommand()
        control_command.header = Header()
        control_command.header.stamp = rclpy.time.Time().to_msg()

        control_command.throttle    = control_signal
        control_command.steering    = -0.5
        control_command.brake       = 0.0

        self.control_publisher.publish(control_command)

def main(args=None):
    rclpy.init(args=args)

    node = ConstSpeedController("ConstSpeedController")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()