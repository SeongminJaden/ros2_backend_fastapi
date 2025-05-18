from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from typing import List
from pydantic import BaseModel
from dotenv import load_dotenv
import mysql.connector
import os

# .env 파일 로드
load_dotenv()

app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class MapData(BaseModel):
    map_name: str
    map_num: int
    map_type: str
    map_date: str
    map_size: str
    note: str

db_config = {
    "host": os.getenv("DB_HOST"),
    "port": os.getenv("DB_PORT"),
    "user": os.getenv("DB_USER"),
    "password": os.getenv("DB_PASSWORD"),
    "database": os.getenv("DB_NAME")
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
