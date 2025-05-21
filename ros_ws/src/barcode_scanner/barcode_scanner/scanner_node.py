import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
import random

class BarcodeScanner(Node):
    def __init__(self):
        super().__init__('barcode_scanner')
        self.publisher = self.create_publisher(String, 'barcode', 10)
        self.service = self.create_service(Trigger, 'get_barcode', self.get_barcode_callback)
        self.timer = self.create_timer(1.0, self.publish_barcode)
        self.current_barcode = "00000"
        
    def publish_barcode(self):
        # Generate random 5-digit barcode
        self.current_barcode = str(random.randint(10000, 99999))
        msg = String()
        msg.data = self.current_barcode
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing barcode: {msg.data}')
        
    def get_barcode_callback(self, request, response):
        response.success = True
        response.message = self.current_barcode
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BarcodeScanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()