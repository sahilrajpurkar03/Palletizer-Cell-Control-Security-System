from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String
from example_interfaces.srv import Trigger
import json
import asyncio
from typing import Dict
import os
from pathlib import Path

# Performance tuning
os.environ["UVICORN_WEB_CONCURRENCY"] = "1"
os.environ["WEB_CONCURRENCY"] = "1"

app = FastAPI(title="Automated Palletizer HMI")

# Path configuration
BASE_DIR = Path(__file__).parent
frontend_path = BASE_DIR.parent / "frontend"

# Mount static files
app.mount("/static", StaticFiles(directory=frontend_path), name="static")

# ROS 2 Node for HMI
class HMINode(Node):
    def __init__(self):
        super().__init__('hmi_node', parameter_overrides=[])
        self.state = {
            "door_closed": True,
            "e_button_pressed": False,
            "stack_light": 0,
            "barcode": "00000",
            "last_request": None,
            "last_response": None
        }
        
        # QoS profile for real-time updates
        qos_profile = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        
        # Subscribers
        self.create_subscription(Bool, 'door_status', self.door_callback, qos_profile)
        self.create_subscription(Bool, 'e_button_status', self.e_button_callback, qos_profile)
        self.create_subscription(Int8, 'stack_light_status', self.light_callback, qos_profile)
        self.create_subscription(String, 'barcode', self.barcode_callback, qos_profile)
    
    def door_callback(self, msg):
        self.state["door_closed"] = msg.data
    
    def e_button_callback(self, msg):
        self.state["e_button_pressed"] = msg.data
    
    def light_callback(self, msg):
        self.state["stack_light"] = msg.data
    
    def barcode_callback(self, msg):
        self.state["barcode"] = msg.data

# Initialize ROS 2 node
rclpy.init()
hmi_node = HMINode()

# WebSocket manager
class ConnectionManager:
    def __init__(self):
        self.active_connections = {}
        self.last_state = None
    
    async def broadcast_state(self):
        while True:
            rclpy.spin_once(hmi_node, timeout_sec=0.01)
            
            # Only send if state changed
            if hmi_node.state != self.last_state:
                self.last_state = hmi_node.state.copy()
                for connection in list(self.active_connections.values()):
                    try:
                        await connection.send_json(self.last_state)
                    except:
                        pass  # Silently handle disconnected clients
            
            await asyncio.sleep(0.1)

manager = ConnectionManager()

@app.get("/", response_class=HTMLResponse)
async def read_root():
    index_path = frontend_path / "index.html"
    with open(index_path) as f:
        return HTMLResponse(content=f.read(), status_code=200)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    manager.active_connections[websocket.client.host] = websocket
    try:
        while True:
            await websocket.receive_text()  # Keep connection alive
    except WebSocketDisconnect:
        del manager.active_connections[websocket.client.host]

# Manual control endpoints
@app.post("/toggle_door")
async def toggle_door():
    node = None
    try:
        node = Node('hmi_temp_node')
        client = node.create_client(Trigger, 'toggle_door')
        
        if not client.wait_for_service(timeout_sec=1.0):
            raise HTTPException(status_code=408, detail="Service timeout")
            
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        return {"success": True, "message": "Door toggled"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if node:
            node.destroy_node()

@app.post("/trigger_emergency")
async def trigger_emergency():
    node = None
    try:
        node = Node('hmi_temp_node')
        client = node.create_client(Trigger, 'activate_e_button')
        
        if not client.wait_for_service(timeout_sec=1.0):
            raise HTTPException(status_code=408, detail="Service timeout")
            
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        return {"success": True, "message": "Emergency activated"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if node:
            node.destroy_node()

@app.post("/reset_emergency")
async def reset_emergency():
    node = None
    try:
        node = Node('hmi_temp_node')
        client = node.create_client(Trigger, 'reset_e_button')
        
        if not client.wait_for_service(timeout_sec=1.0):
            raise HTTPException(status_code=408, detail="Service timeout")
            
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        return {"success": True, "message": "Emergency reset"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if node:
            node.destroy_node()

@app.post("/update_request")
async def update_request(data: dict):
    hmi_node.state["last_request"] = data.get("last_request")
    hmi_node.state["last_response"] = data.get("last_response")
    return {"status": "updated"}

if __name__ == "__main__":
    import uvicorn
    import threading
    
    # Start broadcast thread
    threading.Thread(
        target=lambda: asyncio.run(manager.broadcast_state()),
        daemon=True
    ).start()
    
    # Start FastAPI server
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info",
        ws_ping_interval=1.0,
        ws_ping_timeout=3.0
    )
