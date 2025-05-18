from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import cv2
import time

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# === FLIR 영상 스트리밍 ===
def generate_flir_stream():
    camera = cv2.VideoCapture(2)  # FLIR 카메라
    if not camera.isOpened():
        print("[Error] FLIR 카메라 (/dev/video2)를 열 수 없습니다.")
        return

    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                time.sleep(0.01)
                continue

            # 온도 표현 강조
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n'
            )
    except Exception as e:
        print(f"[Fatal] FLIR stream error: {e}")
    finally:
        camera.release()

# === RealSense 영상 스트리밍 ===
def generate_realsense_stream():
    camera = cv2.VideoCapture(8)  # RealSense 카메라
    if not camera.isOpened():
        print("[Error] RealSense 카메라 (/dev/video8)를 열 수 없습니다.")
        return

    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                time.sleep(0.01)
                continue

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n'
            )
    except Exception as e:
        print(f"[Fatal] RealSense stream error: {e}")
    finally:
        camera.release()

# === FastAPI 라우터 ===
@app.get("/video/flir")
async def video_flir():
    return StreamingResponse(generate_flir_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/video/realsense")
async def video_realsense():
    return StreamingResponse(generate_realsense_stream(), media_type="multipart/x-mixed-replace; boundary=frame")
