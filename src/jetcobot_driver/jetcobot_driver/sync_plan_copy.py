#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot

class MycobotReceiver(Node):
    def __init__(self):
        super().__init__("mycobot_receiver")
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 1000000)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.mc = MyCobot(port, baud)
        self.get_logger().info(f"Connected to MyCobot at {port}, baud: {baud}")
        self.subscription = self.create_subscription(JointState,'joint_states',self.callback,10)

    def callback(self, msg):
        self.get_logger().info(f"******: {msg.position}")
        joint_names = [f'{i}_Joint' for i in range(1, 7)]
        if (len(msg.name) != len(msg.position) or
                len(set(msg.name)) != len(msg.name) or
                not all(name in msg.name for name in joint_names)):
            self.get_logger().error(
                'Joint state is missing required joints or contains duplicate names'
            )
            return

        positions = dict(zip(msg.name, msg.position))
        data_list = [positions[name] for name in joint_names]

        data_list[5] = data_list[5] - 0.7853981633974483
        
        self.mc.send_radians(data_list, 80)
        self.get_logger().info(f"Received joints: {data_list}")

def main(args=None):
    rclpy.init(args=args)
    node = MycobotReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
