#Fastapi is a effictive framework in python to create RESTAPI

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from datetime import datetime
import logging

from app.database import save_command, get_supabase_client
from app.utils.state_manager import rover_state

logger = logging.getLogger(__name__)
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
    "reboot": "Rover is rebooting.",
    "return": "Rover returning to base."
}

@router.post("/")
async def send_command(data: Command):
    command = data.command.lower()

    if command not in valid_commands:
        raise HTTPException(status_code=400, detail="invalid command")

    # Update rover state
    rover_state.update_state(command)

    message = valid_commands[command]
    save_command(command, "OK")

    response = {
        "command": command,
        "timestamp": datetime.now().isoformat(),
        "message": message,
        "status": "OK",
        "new_rover_state": rover_state.get_state()
    }
    return response

@router.get("/test-db")
async def test_db():
    """Test Supabase database connection"""
    try:
        client = get_supabase_client()
        # Try a simple query to test connection
        result = client.table("command_logs").select("count", count="exact").limit(1).execute()
        return {
            "message": "Database connection successful",
            "database": "Supabase PostgreSQL",
            "status": "connected"
        }
    except Exception as e:
        logger.error(f"Database connection test failed: {e}")
        raise HTTPException(status_code=500, detail=f"Database connection failed: {str(e)}")