import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class DoorStatePublisher(Node):
    def __init__(self):
        super().__init__('door_state_publisher')
        self.state = True  # True = Closed, False = Open
        self.publisher_ = self.create_publisher(Bool, 'door_state', 10)
        self.timer = self.create_timer(2.0, self.publish_state)

        # Service to toggle door state
        self.srv = self.create_service(Trigger, 'toggle_door', self.toggle_door)

    def publish_state(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Door Closed: {self.state}')

    def toggle_door(self, request, response):
        self.state = not self.state
        response.success = True
        response.message = f'Door state changed to {self.state}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DoorStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
