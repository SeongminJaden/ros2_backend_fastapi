import asyncio
import random
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("robot_status")

app = FastAPI()

@app.websocket("/ws/status")
async def robot_status(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = {
            "battery": random.randint(50, 100),
            "velocity": round(random.uniform(0.0, 5.0), 2),
            "position": {
                "x": round(random.uniform(-10.0, 10.0), 2),
                "y": round(random.uniform(-10.0, 10.0), 2),
            }
        }
        await websocket.send_json(data)
        await asyncio.sleep(1)
        
@app.websocket("/ws/control")
async def control_socket(websocket: WebSocket):
    await websocket.accept()
    logger.info("Control WebSocket 연결됨")
    try:
        while True:
            data = await websocket.receive_text()
            logger.info(f"Control WebSocket 메시지 받음: {data}")
    except WebSocketDisconnect:
        logger.info("Control WebSocket 연결 종료됨")
