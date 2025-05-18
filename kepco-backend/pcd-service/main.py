from fastapi import FastAPI
from fastapi.responses import FileResponse
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()
# CORS 설정 (React에서 호출 가능하도록)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 보안상 실제 배포시 도메인으로 변경하세요
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class MapInfoRequest(BaseModel):
    map_num: int
    map_type: str

@app.get("/image/{image_id}")
def get_image(image_id: str):
    print(f"Received image request for image_id: {image_id}")
    return {"message": f"Received image request for image_id: {image_id}"}

@app.post("/api/mapinfo")
def map_info(data: MapInfoRequest):
    # 실제 DB 조회 후 image_id 결정하는 로직 필요
    print(f"받은 map_num: {data.map_num}, map_type: {data.map_type}")

    # 예시로 map_num을 image_id로 그대로 씀
    image_id = str(data.map_num)
    return {"image_id": image_id}