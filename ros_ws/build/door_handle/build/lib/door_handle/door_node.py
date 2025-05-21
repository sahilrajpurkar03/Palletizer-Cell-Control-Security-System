import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from example_interfaces.srv import Trigger

class DoorHandle(Node):
    def __init__(self):
        super().__init__('door_handle')
        self.publisher = self.create_publisher(Bool, 'door_status', 10)
        self.service = self.create_service(Trigger, 'toggle_door', self.toggle_door_callback)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.door_closed = True  # Default to closed
        
    def publish_status(self):
        msg = Bool()
        msg.data = self.door_closed
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing door status: {"Closed" if msg.data else "Open"}')
        
    def toggle_door_callback(self, request, response):
        self.door_closed = not self.door_closed
        response.success = True
        response.message = "Door toggled"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DoorHandle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()