from fastapi import FastAPI
from fastapi.responses import FileResponse

app = FastAPI()

@app.get("/pcd/{scene_id}")
def get_pcd(scene_id: str):
    return FileResponse(f"./pcd/{scene_id}.pcd")
