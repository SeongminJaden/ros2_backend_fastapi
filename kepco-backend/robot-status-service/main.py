import asyncio
import logging
import json
import math

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
import tf_transformations

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

emergency_active = False
stop_task = None

class CommandRequest(BaseModel):
    command: str

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_battery = {
            "battery": "0.0",
            "temperature": "0.0",
        }
        self.latest_odom = {
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def battery_callback(self, msg: BatteryState):
        self.latest_battery["battery"] = round(msg.percentage, 1)
        self.latest_battery["temperature"] = round(msg.temperature, 1)

    def odom_callback(self, msg: Odometry):
        self.latest_odom["linear_x"] = msg.twist.twist.linear.x
        self.latest_odom["linear_y"] = msg.twist.twist.linear.y
        self.latest_odom["roll"] = f"{msg.pose.pose.position.x:.2f}"
        self.latest_odom["pitch"] = f"{msg.pose.pose.position.y:.2f}"
        self.latest_odom["angular_z"] = msg.twist.twist.angular.z

        # Orientation(Quaternion) → Euler 각도 변환
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.latest_odom["yaw"] = yaw

    def publish_twist(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.twist_pub.publish(twist)

teleop_node = None  # 전역 변수

@app.on_event("startup")
async def startup_event():
    global teleop_node
    rclpy.init()
    teleop_node = TeleopNode()
    # ROS 콜백을 위한 주기적 spin_once 비동기 태스크 생성
    asyncio.create_task(ros_spin())

@app.on_event("shutdown")
async def shutdown_event():
    global teleop_node
    if teleop_node:
        teleop_node.destroy_node()
    rclpy.shutdown()

async def emergency_loop():
    global emergency_active
    while emergency_active:
        teleop_node.publish_twist(0.0, 0.0)
        logger.info("긴급 정지 신호 반복 전송 중...")
        await asyncio.sleep(0.5)  # 0.5초마다 반복

async def ros_spin():
    """ROS spin_once를 주기적으로 호출하여 콜백 처리"""
    while True:
        rclpy.spin_once(teleop_node, timeout_sec=0.1)
        await asyncio.sleep(0.1)

@app.post("/api/control")
async def control_robot(cmd: CommandRequest):
    global emergency_active, stop_task

    if cmd.command == "emergency_stop":
        if not emergency_active:
            logger.info("긴급 정지 명령 받음")
            emergency_active = True
            stop_task = asyncio.create_task(emergency_loop())
        return {"status": "emergency_stop loop started"}

    elif cmd.command == "resume":
        if emergency_active:
            logger.info("정지 해제 명령 받음")
            emergency_active = False
            if stop_task:
                stop_task.cancel()
                try:
                    await stop_task
                except asyncio.CancelledError:
                    logger.info("긴급 정지 루프 종료됨")
        return {"status": "resume executed"}

    else:
        raise HTTPException(status_code=400, detail="Invalid command")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    logger.info("WebSocket 연결됨")

    linear_x = 0.0
    angular_z = 0.0

    MAX_LINEAR_X = 2.0
    MIN_LINEAR_X = -2.0
    MAX_ANGULAR_Z = 1.0
    MIN_ANGULAR_Z = -1.0

    async def publish_loop():
        while True:
            if not emergency_active:  # 긴급정지 상태에서는 publish하지 않음
                teleop_node.publish_twist(linear_x, angular_z)
            await asyncio.sleep(0.1)

    publish_task = asyncio.create_task(publish_loop())

    try:
        while True:
            try:
                data = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                logger.info(f"명령 수신: {data}")

                try:
                    msg = json.loads(data)
                    key = msg.get("key", "")
                except Exception as e:
                    logger.error(f"JSON 파싱 실패: {e}")
                    key = ""

                if key == 'w':
                    linear_x = min(linear_x + 0.1, MAX_LINEAR_X)
                elif key == 'x':
                    linear_x = max(linear_x - 0.1, MIN_LINEAR_X)
                elif key == 'a':
                    angular_z = min(angular_z + 0.1, MAX_ANGULAR_Z)
                elif key == 'd':
                    angular_z = max(angular_z - 0.1, MIN_ANGULAR_Z)
                elif key == 's':
                    linear_x = 0.0
                    angular_z = 0.0

            except asyncio.TimeoutError:
                # 키 입력 없을 때 상태 데이터 전송
                status = {
                    "velocity": f"{round(float(teleop_node.latest_odom.get('linear_x', '0.0')), 2)}",
                    "angular_velocity": f"{round(float(teleop_node.latest_odom.get('angular_z', '0.0')), 2)}",
                    "battery": f"{round(float(teleop_node.latest_battery.get('battery', '0.0')), 2)}",
                    "temperature": f"{round(float(teleop_node.latest_battery.get('temperature', '0.0')), 2)}",
                    "roll": f"{round(float(teleop_node.latest_odom.get('roll', '0.0')), 2)}",
                    "pitch": f"{round(float(teleop_node.latest_odom.get('pitch', '0.0')), 2)}",
                    "yaw": f"{round(float(teleop_node.latest_odom.get('yaw', '0.0')), 2)}",
                }
                await websocket.send_json({"type": "status", "data": status})

    except WebSocketDisconnect:
        logger.info("WebSocket 연결 종료됨")
    except Exception as e:
        logger.error(f"WebSocket 에러 발생: {e}")
    finally:
        publish_task.cancel()


@app.get("/status")
def get_status():
    return {"robot": "OK"}