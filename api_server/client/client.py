from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
import random
import logging
from typing import Optional

# ROS 2 imports for checking system status
from std_msgs.msg import Bool, Int8

app = FastAPI()

# Mock ROS2 node for demonstration
class CellStatusChecker(Node):
    def __init__(self):
        super().__init__('cell_status_checker')
        self.door_closed = True
        self.e_button_pressed = False
        
        # Create subscriptions (in real implementation, these would connect to actual nodes)
        self.door_sub = self.create_subscription(
            Bool, 'door_status', self.door_callback, 10)
        self.e_button_sub = self.create_subscription(
            Bool, 'e_button_status', self.e_button_callback, 10)
    
    def door_callback(self, msg):
        self.door_closed = msg.data
    
    def e_button_callback(self, msg):
        self.e_button_pressed = msg.data

# Initialize ROS2 node
rclpy.init()
cell_status = CellStatusChecker()

class PickConfirmation(BaseModel):
    pickId: int
    pickSuccessful: bool
    errorMessage: Optional[str] = None
    itemBarcode: Optional[str] = None

@app.post("/confirmPick")
async def confirm_pick(pick_request: dict):
    """
    Endpoint that simulates the robotic cell processing
    """
    try:
        pick_id = pick_request.get("pickId")
        quantity = pick_request.get("quantity", 1)
        
        # Check cell status
        rclpy.spin_once(cell_status, timeout_sec=0.1)
        
        if cell_status.e_button_pressed:
            return PickConfirmation(
                pickId=pick_id,
                pickSuccessful=False,
                errorMessage="Emergency button pressed",
                itemBarcode=None
            )
            
        if not cell_status.door_closed:
            return PickConfirmation(
                pickId=pick_id,
                pickSuccessful=False,
                errorMessage="Cell door open",
                itemBarcode=None
            )
        
        # Simulate successful pick
        barcode = str(random.randint(10000, 99999))  # Mock barcode
        return PickConfirmation(
            pickId=pick_id,
            pickSuccessful=True,
            errorMessage=None,
            itemBarcode=barcode
        )
        
    except Exception as e:
        logging.error(f"Error processing pick: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8081)