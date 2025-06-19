from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
import threading
import random
import logging
from typing import Optional
from std_msgs.msg import Bool

from threading import Lock

app = FastAPI()

# ROS 2 Node with thread-safe state
class CellStatusChecker(Node):
    def __init__(self):
        super().__init__('cell_status_checker')
        self._lock = Lock()
        self.door_closed = True
        self.e_button_pressed = False

        self.door_sub = self.create_subscription(
            Bool, 'door_status', self.door_callback, 10)
        self.e_button_sub = self.create_subscription(
            Bool, 'e_button_status', self.e_button_callback, 10)

    def door_callback(self, msg):
        with self._lock:
            self.door_closed = msg.data

    def e_button_callback(self, msg):
        with self._lock:
            self.e_button_pressed = msg.data

    def get_status(self):
        with self._lock:
            return self.door_closed, self.e_button_pressed

# Initialize ROS2 and spin in background thread
rclpy.init()
cell_status = CellStatusChecker()

def spin_ros_node():
    rclpy.spin(cell_status)

ros_thread = threading.Thread(target=spin_ros_node, daemon=True)
ros_thread.start()

# Data model for response
class PalletizeConfirmation(BaseModel):
    palletId: int
    palletizeSuccessful: bool
    errorMessage: Optional[str] = None
    lastBoxBarcode: Optional[str] = None

@app.post("/confirmPalletize")
async def confirm_palletize(palletize_request: dict):
    """
    Endpoint that simulates the robotic palletizer cell processing
    """
    try:
        pallet_id = palletize_request.get("palletId")
        box_count = palletize_request.get("boxCount", 1)

        # Get live status
        door_closed, e_button_pressed = cell_status.get_status()

        logging.info(f"[Pallet ID {pallet_id}] Door Closed: {door_closed}, Emergency Pressed: {e_button_pressed}")

        if e_button_pressed:
            return PalletizeConfirmation(
                palletId=pallet_id,
                palletizeSuccessful=False,
                errorMessage="Emergency button pressed",
                lastBoxBarcode=None
            )

        if not door_closed:
            return PalletizeConfirmation(
                palletId=pallet_id,
                palletizeSuccessful=False,
                errorMessage="Cell door open",
                lastBoxBarcode=None
            )

        # Simulate successful palletizing
        barcode = str(random.randint(10000, 99999))  # Mock barcode
        return PalletizeConfirmation(
            palletId=pallet_id,
            palletizeSuccessful=True,
            errorMessage=None,
            lastBoxBarcode=barcode
        )

    except Exception as e:
        logging.error(f"Error processing palletize: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8081)
