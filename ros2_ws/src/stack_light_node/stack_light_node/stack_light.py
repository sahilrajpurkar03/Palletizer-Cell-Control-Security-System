import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool

class StackLightNode(Node):
    def __init__(self):
        super().__init__('stack_light_node')
        self.stack_pub = self.create_publisher(Int8, 'stack_light', 10)

        self.door_closed = True
        self.emergency_pressed = False

        self.create_subscription(Bool, 'door_state', self.door_callback, 10)
        self.create_subscription(Bool, 'emergency_state', self.emergency_callback, 10)
        self.timer = self.create_timer(1.0, self.evaluate_stack_light)

    def door_callback(self, msg):
        self.door_closed = msg.data

    def emergency_callback(self, msg):
        self.emergency_pressed = msg.data

    def evaluate_stack_light(self):
        if self.emergency_pressed:
            status = -1  # Red
        elif not self.door_closed:
            status = 1   # Yellow
        else:
            status = 0   # Green

        msg = Int8()
        msg.data = status
        self.stack_pub.publish(msg)
        self.get_logger().info(f'Stack Light Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = StackLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
