FROM python:3.10-slim

# 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libusb-1.0-0-dev \
    libsm6 \
    libxext6 \
    libxrender-dev \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    && apt-get clean

# OpenCV 설치 (헤드리스 버전)
RUN pip install opencv-python-headless

# pyrealsense2 설치
RUN pip install pyrealsense2

# FastAPI 및 uvicorn 설치
# WebSocket 지원 포함
RUN pip install fastapi "uvicorn[standard]"

# 코드 복사
WORKDIR /app
COPY . /app

# 기본 실행 명령어
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
