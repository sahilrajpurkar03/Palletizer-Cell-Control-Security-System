import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool 
from rclpy.subscription import Subscription

class StackLight(Node):
    def __init__(self):
        super().__init__('stack_light')
        self.publisher = self.create_publisher(Int8, 'stack_light_status', 10)
        
        # Subscribers for other nodes' data
        self.door_sub = self.create_subscription(
            Bool, 'door_status', self.door_callback, 10)
        self.e_button_sub = self.create_subscription(
            Bool, 'e_button_status', self.e_button_callback, 10)
            
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.door_closed = True
        self.e_button_pressed = False
        
    def door_callback(self, msg):
        self.door_closed = msg.data
        
    def e_button_callback(self, msg):
        self.e_button_pressed = msg.data
        
    def publish_status(self):
        msg = Int8()
        
        if self.e_button_pressed:
            msg.data = -1  # Emergency
        elif not self.door_closed:
            msg.data = 1    # Paused
        else:
            msg.data = 0    # Operational
            
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing stack light status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = StackLight()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()