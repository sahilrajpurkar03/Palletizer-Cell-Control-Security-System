import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class BarcodePublisher(Node):
    def __init__(self):
        super().__init__('barcode_publisher')
        self.publisher_ = self.create_publisher(String, 'barcode_topic', 10)
        self.latest_barcode = '00000'
        self.create_timer(1.0, self.publish_barcode)

    def publish_barcode(self):
        barcode = ''.join([str(random.randint(0, 9)) for _ in range(5)])
        self.latest_barcode = barcode
        msg = String()
        msg.data = barcode
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Barcode: {barcode}')

def main(args=None):
    rclpy.init(args=args)
    node = BarcodePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
