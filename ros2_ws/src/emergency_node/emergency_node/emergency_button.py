import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class EmergencyButtonNode(Node):
    def __init__(self):
        super().__init__('emergency_button_node')
        self.state = False
        self.publisher_ = self.create_publisher(Bool, 'emergency_state', 10)
        self.timer = self.create_timer(1.0, self.publish_state)

        self.create_service(Trigger, 'press_emergency', self.press)
        self.create_service(Trigger, 'release_emergency', self.release)

    def publish_state(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Emergency Pressed: {self.state}')

    def press(self, request, response):
        self.state = True
        response.success = True
        response.message = "Emergency button pressed."
        return response

    def release(self, request, response):
        self.state = False
        response.success = True
        response.message = "Emergency button released."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
