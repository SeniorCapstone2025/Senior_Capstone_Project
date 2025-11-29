from fastapi import APIRouter
from app.database import get_db_connection

router = APIRouter(
    prefix="/logs",
    tags=["logs"]
)

@router.get("/commands")
async def commands_logs():
    conn = get_db_connection()
    cursor = conn.cursor(dictionary=True)

    cursor.execute("SELECT * FROM command_logs ORDER BY TIMESTAMP DESC")
    logs = cursor.fetchall()

    conn.close()
    return {"logs": logs}

@router.get("/status")
async def status_logs():
    conn = get_db_connection()
    cursor = conn.cursor(dictionary=True)

    cursor.execute("SELECT * FROM status_logs ORDER BY TIMESTAMP DESC")
    logs = cursor.fetchall()

    conn.close()
    return {"logs": logs}

@router.get("/detections")
async def detections():
    conn = get_db_connection()
    cursor = conn.cursor(dictionary=True)

    cursor.execute("SELECT * FROM detections ORDER BY detected_at DESC")
    logs = cursor.fetchall()

    conn.close()
    return {"logs": logs}
