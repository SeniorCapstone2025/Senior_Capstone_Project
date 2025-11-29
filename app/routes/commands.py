#Fastapi is a effictive framework in python to create RESTAPI

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from datetime import datetime

router = APIRouter(prefix="/command", tags=["commands"])

class Command(BaseModel):
    command: str

valid_commands = {
        "start": "Rover started scanning. ",
        "pause": "Rover paused scanning. ",
        "cancel": "Rover canceled scanning and returned to base. ",
        "search": "Rover is searching for item. ",
        "reboot": "Rover is rebooting. "
    }
@router.post("/")
async def send_command(data: Command):
        """
            Receives a command from the frontend and returns a placeholder response.
            Later, this will send the command to ROS2 via WebSocket.
            """

        command = data.command.lower()

        if command not in valid_commands:
            raise HTTPException(status_code=400, detail="Invalid command")

        response = {"command": command,
                    "timestamp": datetime.now().isoformat(),
                    "message": valid_commands[command],
                    "status": "OK"}

        return response

from app.database import get_db_connection

@router.get("/test-db")
async def test_db():
    connection = get_db_connection()
    if connection:
        return {"message": "Database connection successful!"}
    else:
        return {"message": "Database connection failed."}
