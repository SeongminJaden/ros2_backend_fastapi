from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import mysql.connector
from typing import List
from pydantic import BaseModel

app = FastAPI()

# CORS 설정 (React에서 호출 가능하도록)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 보안상 실제 배포시 도메인으로 변경하세요
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic 모델 정의 (DB 테이블 컬럼과 맞추기)
class MapData(BaseModel):
    map_name: str
    map_num: int
    map_type: str
    map_date: str  # YYYY-MM-DD 형식으로 받을 예정
    map_size: str
    note: str

# MySQL 연결 정보
db_config = {
    "host": "192.168.0.49",
    "user": "myuser",
    "password": "mypassword",
    "database": "slamDB"
}

@app.get("/api/maps", response_model=List[MapData])
def get_maps():
    conn = mysql.connector.connect(**db_config)
    cursor = conn.cursor(dictionary=True)
    cursor.execute("""
        SELECT 
            map_name, 
            map_num, 
            map_type, 
            DATE_FORMAT(map_date, '%Y-%m-%d') AS map_date, 
            map_size, 
            note 
        FROM maps
        ORDER BY map_date DESC
    """)
    results = cursor.fetchall()
    cursor.close()
    conn.close()
    return results
