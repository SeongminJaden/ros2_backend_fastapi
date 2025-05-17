from fastapi import FastAPI
from fastapi.responses import FileResponse

app = FastAPI()

@app.get("/image/{image_id}")
def get_image(image_id: str):
    return FileResponse(f"./images/{image_id}.jpg")
