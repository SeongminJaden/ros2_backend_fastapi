from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# CORS 허용 (React에서 요청 가능하도록)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/slam/start")
async def slam_start():
    print("🚀 SLAM 시작")
    return {"status": "started", "message": "SLAM 시작됨"}

@app.post("/slam/pause")
async def slam_start():
    print("🚀 nav 시작")
    return {"status": "started", "message": "nav 시작됨"}

@app.post("/slam/end")
async def slam_end():
    print("🛑 SLAM 종료")
    return {"status": "ended", "message": "SLAM 종료됨"}

@app.post("/nav/start")
async def slam_start():
    print("🚀 nav 시작")
    return {"status": "started", "message": "nav 시작됨"}

@app.post("/nav/pause")
async def slam_start():
    print("🚀 nav 시작")
    return {"status": "started", "message": "nav 시작됨"}

@app.post("/nav/end")
async def slam_end():
    print("🛑 nav 종료")
    return {"status": "ended", "message": "nav 종료됨"}