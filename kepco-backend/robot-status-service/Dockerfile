FROM ros:humble

# 필수 의존성 설치
RUN apt update && apt install -y \
    python3-pip \
    ros-humble-geometry-msgs \
    ros-humble-rclpy \
    curl \
    && rm -rf /var/lib/apt/lists/*

# FastAPI 설치
RUN pip install fastapi uvicorn

# 작업 디렉토리
WORKDIR /app

# FastAPI 코드 복사
COPY . /app

# 포트 열기
EXPOSE 8000

# FastAPI 실행
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
