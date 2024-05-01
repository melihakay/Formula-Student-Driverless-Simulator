from typing import List
from rclpy.context import Context
from rclpy.node import Node
import rclpy
from rclpy.parameter import Parameter

from fs_msgs.msg import ControlCommand
from sensor_msgs.msg import Joy

class JoyController(Node):
    def __init__(self, node_name: str, *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)

        self.get_logger().info("Initialized controller")
        self.publisher = self.create_publisher(ControlCommand, "/control_command", 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)


    def listener_callback(self, joy_state):
        buttons = joy_state.buttons
        axes    = joy_state.axes

        max_steering = 1.0 # 0.4
        max_throttle = 0.2
        max_brake = 1.0 # 0.3

        # Change multiplication
        steering = axes[0]  # -1 to 1
        throttle = axes[5] # 1 to -1, released to full
        brake = axes[2]

        steering = -min(max(-max_steering, steering), max_steering)
        throttle = min( 1 - ((throttle + 1) / 2), max_throttle)
        brake = min( 1 - ((brake + 1) / 2), max_brake)

        msg = ControlCommand()
        msg.throttle = throttle
        msg.steering = steering
        msg.brake = brake

        self.publisher.publish(msg)
        self.get_logger().debug(f"Command: T {msg.throttle}, S {msg.steering}, B {msg.brake}")

def main(args=None):
    rclpy.init(args=args)

    node = JoyController("Joy_Controller")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()