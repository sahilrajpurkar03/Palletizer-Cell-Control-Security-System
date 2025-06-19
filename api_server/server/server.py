from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import httpx
import logging
from typing import Optional

app = FastAPI()

# Configuration
CLIENT_ENDPOINT = "http://localhost:8081/confirmPalletize"
HMI_ENDPOINT = "http://localhost:8000/update_request"

class PalletizeRequest(BaseModel):
    palletId: int
    boxCount: int

class PalletizeResponse(BaseModel):
    palletId: int
    palletizeSuccessful: bool
    errorMessage: Optional[str] = None
    lastBoxBarcode: Optional[str] = None

@app.post("/palletize")
async def handle_palletize_request(request: PalletizeRequest):
    """
    Endpoint that receives palletizing requests from WMS
    """
    logging.info(f"Received palletize request: {request}")

    try:
        async with httpx.AsyncClient() as client:
            # Forward to robotic cell client
            response = await client.post(
                CLIENT_ENDPOINT,
                json={
                    "palletId": request.palletId,
                    "boxCount": request.boxCount
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
        logging.error(f"Error processing palletize request: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)
