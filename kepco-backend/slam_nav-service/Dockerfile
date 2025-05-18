FROM python:3.10

WORKDIR /app
COPY . /app

# WebSocket 지원 포함
RUN pip install fastapi "uvicorn[standard]"

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
