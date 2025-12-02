from supabase import create_client, Client
from app.config import get_settings
from typing import Optional
import logging

logger = logging.getLogger(__name__)

# Global Supabase client instance
_supabase_client: Optional[Client] = None


def get_supabase_client() -> Client:
    """Get or create Supabase client singleton"""
    global _supabase_client
    if _supabase_client is None:
        settings = get_settings()
        _supabase_client = create_client(
            settings.supabase_url,
            settings.supabase_service_key
        )
        logger.info("Supabase client initialized")
    return _supabase_client


def save_command(command: str, status: str):
    """Save command log to Supabase"""
    try:
        client = get_supabase_client()
        data = {"command": command, "status": status}
        result = client.table("command_logs").insert(data).execute()
        logger.info(f"Command saved: {command} - {status}")
        return result
    except Exception as e:
        logger.error(f"Error saving command: {e}")
        raise


def save_status(state: str, battery: float, current_task: Optional[str]):
    """Save status log to Supabase"""
    try:
        client = get_supabase_client()
        data = {
            "state": state,
            "battery": battery,
            "current_task": current_task
        }
        result = client.table("status_logs").insert(data).execute()
        logger.info(f"Status saved: {state} - Battery: {battery}%")
        return result
    except Exception as e:
        logger.error(f"Error saving status: {e}")
        raise


def save_detected_object(object_name: str, confidence: float, x: float, y: float):
    """Save detected object to Supabase"""
    try:
        client = get_supabase_client()
        data = {
            "object_name": object_name,
            "confidence": confidence,
            "x": x,
            "y": y
        }
        result = client.table("detections").insert(data).execute()
        logger.debug(f"Detection saved: {object_name} ({confidence:.2f})")
        return result
    except Exception as e:
        logger.error(f"Error saving detection: {e}")
        raise


def get_command_logs(limit: int = 50):
    """Retrieve command logs from Supabase"""
    try:
        client = get_supabase_client()
        result = client.table("command_logs")\
            .select("*")\
            .order("timestamp", desc=True)\
            .limit(limit)\
            .execute()
        return result.data
    except Exception as e:
        logger.error(f"Error fetching command logs: {e}")
        raise


def get_status_logs(limit: int = 50):
    """Retrieve status logs from Supabase"""
    try:
        client = get_supabase_client()
        result = client.table("status_logs")\
            .select("*")\
            .order("timestamp", desc=True)\
            .limit(limit)\
            .execute()
        return result.data
    except Exception as e:
        logger.error(f"Error fetching status logs: {e}")
        raise


def get_detection_logs(limit: int = 50):
    """Retrieve detection logs from Supabase"""
    try:
        client = get_supabase_client()
        result = client.table("detections")\
            .select("*")\
            .order("detected_at", desc=True)\
            .limit(limit)\
            .execute()
        return result.data
    except Exception as e:
        logger.error(f"Error fetching detection logs: {e}")
        raise