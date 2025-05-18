import asyncio
import random
import logging
import json

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("robot_status")

app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class CommandRequest(BaseModel):
    command: str

# ROS 2 노드 클래스 정의
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)
        self.get_logger().info(f"Published Twist: linear_x={linear_x}, angular_z={angular_z}")

# 전역 변수로 노드 할당
teleop_node = None

@app.on_event("startup")
async def startup_event():
    global teleop_node
    # ROS 2 초기화 (별도 스레드에서 돌릴 수도 있지만 여기선 간단하게)
    rclpy.init()
    teleop_node = TeleopNode()

@app.on_event("shutdown")
async def shutdown_event():
    global teleop_node
    if teleop_node:
        teleop_node.destroy_node()
    rclpy.shutdown()

@app.post("/api/control")
async def control_robot(cmd: CommandRequest):
    if cmd.command == "emergency_stop":
        logger.info("긴급 정지 명령 받음")
        teleop_node.publish_twist(0.0, 0.0)  # 정지
        return {"status": "emergency_stop executed"}

    elif cmd.command == "resume":
        logger.info("정지 해제 명령 받음")
        # 여기선 아무 동작 안함, 필요하면 재시작 명령 추가 가능
        return {"status": "resume executed"}

    else:
        raise HTTPException(status_code=400, detail="Invalid command")
    
@app.websocket("/ws/control")
async def control_socket(websocket: WebSocket):
    await websocket.accept()
    logger.info("Control WebSocket 연결됨")
    try:
        while True:
            data = await websocket.receive_text()
            logger.info(f"Control WebSocket 메시지 받음: {data}")

            try:
                msg = json.loads(data)
                key = msg.get("key", "")
            except Exception as e:
                logger.error(f"JSON 파싱 실패: {e}")
                key = ""

            linear_x = 0.0
            angular_z = 0.0

            if key == 'w':
                linear_x = 0.5
            elif key == 's':
                linear_x = -0.5
            elif key == 'a':
                angular_z = 0.5
            elif key == 'd':
                angular_z = -0.5
            elif key == 'x':
                linear_x = 0.0
                angular_z = 0.0

            teleop_node.publish_twist(linear_x, angular_z)

    except WebSocketDisconnect:
        logger.info("Control WebSocket 연결 종료됨")
    except Exception as e:
        logger.error(f"Control WebSocket 에러 발생: {e}")