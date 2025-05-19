from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import requests

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

ROBOT_STATUS_HOST = "172.20.10.6"
ROBOT_STATUS_PORT = 8000

@app.get("/check-robot")
def check_robot():
    try:
        url = f"http://{ROBOT_STATUS_HOST}:{ROBOT_STATUS_PORT}/status"
        response = requests.get(url)
        response.raise_for_status()
        return {"status": "success", "data": response.json()}
    except Exception as e:
        return {"status": "error", "message": str(e)}
    
@app.post("/api/slam")
async def control_robot(cmd: CommandRequest):
    if cmd.command == "start_slam":
        # 긴급 정지 처리 로직
        print("slam 시작")
        # TODO: 실제 정지 동작 수행
        return {"status": "emergency_stop executed"}
    
    elif cmd.command == "pause_slam":
        # 정지 해제 처리 로직
        print("slam 일시정지")
        # TODO: 실제 정지 해제 동작 수행
        return {"status": "resume executed"}

    elif cmd.command == "end_slam":
        # 정지 해제 처리 로직
        print("slam 종료")
        # TODO: 실제 정지 해제 동작 수행
        return {"status": "resume executed"}

    else:
        raise HTTPException(status_code=400, detail="Invalid command")

@app.post("/api/nav")
async def control_robot(cmd: CommandRequest):
    if cmd.command == "start_nav":
        # 긴급 정지 처리 로직
        print("nav 시작")
        # TODO: 실제 정지 동작 수행
        return {"status": "emergency_stop executed"}
    
    elif cmd.command == "pause_nav":
        # 정지 해제 처리 로직
        print("nav 일시정지")
        # TODO: 실제 정지 해제 동작 수행
        return {"status": "resume executed"}

    elif cmd.command == "end_nav":
        # 정지 해제 처리 로직
        print("nav 종료")
        # TODO: 실제 정지 해제 동작 수행
        return {"status": "resume executed"}

    else:
        raise HTTPException(status_code=400, detail="Invalid command")
    