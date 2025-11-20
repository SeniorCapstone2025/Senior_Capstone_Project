from fastapi import FastAPI
from app.routes import commands, status

app = FastAPI(title="Rover")
app.include_router(commands.router)
app.include_router(status.router)


@app.get("/")
async def root():
    return {"message": "Rover is running properly 👨🏾‍💻"}


