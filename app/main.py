from fastapi import FastAPI
from app.routes import commands, status,detections,logs


app = FastAPI(title="Rover")
app.include_router(commands.router)
app.include_router(status.router)
app.include_router(detections.router)
app.include_router(logs.router)


@app.get("/")
async def root():
    return {"message": "Rover is running properly 👨🏾‍💻"}


