from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import httpx
import logging
from typing import Optional

app = FastAPI()

# Configuration
CLIENT_ENDPOINT = "http://localhost:8081/confirmPick"

class PickRequest(BaseModel):
    pickId: int
    quantity: int

class PickResponse(BaseModel):
    pickId: int
    pickSuccessful: bool
    errorMessage: Optional[str] = None
    itemBarcode: Optional[str] = None

@app.post("/pick")
async def handle_pick_request(request: PickRequest):
    """
    Endpoint that receives pick requests from WMS
    """
    logging.info(f"Received pick request: {request}")
    
    try:
        # Forward to robotic cell client (in a real scenario, this would be async)
        async with httpx.AsyncClient() as client:
            response = await client.post(
                CLIENT_ENDPOINT,
                json={
                    "pickId": request.pickId,
                    "quantity": request.quantity
                },
                timeout=30.0
            )
            
            if response.status_code != 200:
                raise HTTPException(status_code=500, detail="Client processing failed")
                
            return response.json()
            
    except Exception as e:
        logging.error(f"Error processing pick request: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)