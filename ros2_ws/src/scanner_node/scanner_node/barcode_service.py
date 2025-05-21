from scanner_node.scanner_publisher import BarcodePublisher
from rclpy.node import Node
from std_srvs.srv import Trigger

class BarcodeService(Node):
    def __init__(self, publisher: BarcodePublisher):
        super().__init__('barcode_service')
        self.publisher_ref = publisher
        self.srv = self.create_service(Trigger, 'get_barcode', self.get_barcode)

    def get_barcode(self, request, response):
        response.success = True
        response.message = self.publisher_ref.latest_barcode
        return response
