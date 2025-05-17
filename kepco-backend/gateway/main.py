from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {
        "message": "Welcome to KEPCO Backend",
        "services": {
            "image": "http://localhost:8001/image/{image_id}",
            "pcd": "http://localhost:8002/pcd/{scene_id}",
            "robot_status_ws": "ws://localhost:8003/ws/status"
        }
    }
