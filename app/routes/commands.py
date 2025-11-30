#Fastapi is a effictive framework in python to create RESTAPI

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from datetime import datetime

from app.database import get_db_connection, save_command
router = APIRouter(
    prefix="/command", tags=["commands"]
)

class Command(BaseModel):
    command: str

valid_commands = {
    "start": "Rover started scanning.",
    "pause": "Rover paused scanning.",
    "cancel": "Rover canceled scanning and returned to base.",
    "search": "Rover is searching for item.",
    "reboot": "Rover is rebooting."
}

@router.post("/")
async def send_command(data: Command):
    command = data.command.lower()

    if command not in valid_commands:
        raise HTTPException(status_code=400, detail="invalid command")
    message = valid_commands[command]
    save_command(command,"OK")

    response = {
        "command": command,
        "timestamp": datetime.now().isoformat(),
        "message": message,
        "status": "OK"
    }
    return response

@router.get("/test-db")
async def test_db():
    conn = get_db_connection()
    if conn:
        return {"message": "Database connection successful"}
    else:
        return {"message": "Database connection failed"}