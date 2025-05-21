from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import httpx
import logging
from typing import Optional

app = FastAPI()

# Configuration
CLIENT_ENDPOINT = "http://localhost:8081/confirmPick"
HMI_ENDPOINT = "http://localhost:8000/update_request"  

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
        async with httpx.AsyncClient() as client:
            # Forward to robotic cell client
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

            # Forward to HMI (added section)
            try:
                async with httpx.AsyncClient() as hmi_client:
                    hmi_response = await hmi_client.post(
                        HMI_ENDPOINT,
                        json={
                            "last_request": request.dict(),
                            "last_response": response.json()
                        },
                        timeout=1.0
                    )
            except Exception as hmi_error:
                logging.warning(f"Could not update HMI: {hmi_error}")
                
            return response.json()
            
    except Exception as e:
        logging.error(f"Error processing pick request: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)