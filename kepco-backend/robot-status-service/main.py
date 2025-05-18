import asyncio
import random
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("robot_status")

app = FastAPI()
# CORS 허용 (React에서 요청 가능하도록)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
class CommandRequest(BaseModel):
    command: str

@app.post("/api/control")
async def control_robot(cmd: CommandRequest):
    if cmd.command == "emergency_stop":
        # 긴급 정지 처리 로직
        print("긴급 정지 명령 받음")
        # TODO: 실제 정지 동작 수행
        return {"status": "emergency_stop executed"}
    
    elif cmd.command == "resume":
        # 정지 해제 처리 로직
        print("정지 해제 명령 받음")
        # TODO: 실제 정지 해제 동작 수행
        return {"status": "resume executed"}

    else:
        raise HTTPException(status_code=400, detail="Invalid command")

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
