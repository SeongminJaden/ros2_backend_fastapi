import asyncio
import random
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 로그 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("robot_status")

# ROS 노드 정의
class ROSPublisher(Node):
    def __init__(self):
        super().__init__('fastapi_ros_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish_cmd(self, key: str):
        twist = Twist()
        if key == "w":
            twist.linear.x = 0.5
        elif key == "s":
            twist.linear.x = -0.5
        elif key == "a":
            twist.angular.z = 1.0
        elif key == "d":
            twist.angular.z = -1.0
        else:
            return  # 유효하지 않은 키는 무시
        self.publisher_.publish(twist)
        logger.info(f"ROS cmd_vel 퍼블리시됨: {key}")

# ROS2 초기화 및 노드 생성
rclpy.init()
ros_node = ROSPublisher()

# FastAPI 앱 생성
app = FastAPI()

# CORS 허용 (React 접근 허용)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# POST API: 긴급 정지 등 명령
class CommandRequest(BaseModel):
    command: str

@app.post("/api/control")
async def control_robot(cmd: CommandRequest):
    if cmd.command == "emergency_stop":
        logger.info("긴급 정지 명령 받음")
        # TODO: 실제 정지 동작 수행
        return {"status": "emergency_stop executed"}
    
    elif cmd.command == "resume":
        logger.info("정지 해제 명령 받음")
        # TODO: 실제 재개 동작 수행
        return {"status": "resume executed"}

    else:
        raise HTTPException(status_code=400, detail="Invalid command")

# WebSocket: 상태 스트리밍
@app.websocket("/ws/status")
async def robot_status(websocket: WebSocket):
    await websocket.accept()
    try:
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
    except WebSocketDisconnect:
        logger.info("Status WebSocket 연결 종료됨")
    except Exception as e:
        logger.error(f"Status WebSocket 에러 발생: {e}")

# WebSocket: 키 입력 받아 ROS로 퍼블리시
@app.websocket("/ws/control")
async def control_socket(websocket: WebSocket):
    await websocket.accept()
    logger.info("Control WebSocket 연결됨")
    try:
        while True:
            data = await websocket.receive_text()
            logger.info(f"Control WebSocket 메시지 받음: {data}")
            ros_node.publish_cmd(data)
    except WebSocketDisconnect:
        logger.info("Control WebSocket 연결 종료됨")
    except Exception as e:
        logger.error(f"Control WebSocket 에러 발생: {e}")
