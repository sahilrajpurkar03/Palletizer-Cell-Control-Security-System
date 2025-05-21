import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from example_interfaces.srv import Trigger

class EmergencyButton(Node):
    def __init__(self):
        super().__init__('emergency_button')
        self.publisher = self.create_publisher(Bool, 'e_button_status', 10)
        self.activate_service = self.create_service(Trigger, 'activate_e_button', self.activate_callback)
        self.reset_service = self.create_service(Trigger, 'reset_e_button', self.reset_callback)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.button_pressed = False  # Default to not pressed
        
    def publish_status(self):
        msg = Bool()
        msg.data = self.button_pressed
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing e-button status: {"Pressed" if msg.data else "Not pressed"}')
        
    def activate_callback(self, request, response):
        self.button_pressed = True
        response.success = True
        response.message = "Emergency button activated"
        return response
        
    def reset_callback(self, request, response):
        self.button_pressed = False
        response.success = True
        response.message = "Emergency button reset"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyButton()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()