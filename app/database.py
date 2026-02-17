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

def get_inventory_item(item_id: str):
    try:
        client = get_supabase_client()
        result = client.table("inventory")\
            .select("*")\
            .eq("item_id", item_id)\
            .execute()
        
        return result
    
    except Exception as e:
        logger.error(f"Error fetching inventory item {item_id}: {e}")
        return None


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


# =============================================================================
# Scan Management
# =============================================================================

def create_scan(scan_id: str, shelf_ids: list) -> dict:
    """
    Create a new scan record.

    Args:
        scan_id: Unique scan identifier
        shelf_ids: List of shelf IDs to scan

    Returns:
        Created scan record
    """
    try:
        client = get_supabase_client()
        result = client.table("scans").insert({
            "scan_id": scan_id,
            "status": "pending",
            "shelf_ids": shelf_ids
        }).execute()
        logger.info(f"Scan created: {scan_id} for shelves {shelf_ids}")
        return result.data[0] if result.data else None
    except Exception as e:
        logger.error(f"Error creating scan {scan_id}: {e}")
        raise


def update_scan_status(scan_id: str, status: str):
    """
    Update scan status.

    Args:
        scan_id: Scan identifier
        status: New status (pending, in_progress, completed, failed)
    """
    try:
        client = get_supabase_client()
        update_data = {"status": status}

        # Set completed_at timestamp when scan finishes
        if status in ("completed", "failed"):
            from datetime import datetime
            update_data["completed_at"] = datetime.now().isoformat()

        result = client.table("scans")\
            .update(update_data)\
            .eq("scan_id", scan_id)\
            .execute()
        logger.info(f"Scan {scan_id} status updated to {status}")
        return result
    except Exception as e:
        logger.error(f"Error updating scan {scan_id} status: {e}")
        raise


def get_scan(scan_id: str) -> Optional[dict]:
    """Get scan by ID."""
    try:
        client = get_supabase_client()
        result = client.table("scans")\
            .select("*")\
            .eq("scan_id", scan_id)\
            .execute()
        return result.data[0] if result.data else None
    except Exception as e:
        logger.error(f"Error fetching scan {scan_id}: {e}")
        return None


# =============================================================================
# Shelf Inventory
# =============================================================================

def get_shelf_inventory(shelf_id: str) -> list:
    """
    Get expected inventory items for a shelf.

    Args:
        shelf_id: Shelf identifier (matches QR code)

    Returns:
        List of item names expected on this shelf
    """
    try:
        client = get_supabase_client()
        result = client.table("shelf_inventory")\
            .select("expected_items")\
            .eq("shelf_id", shelf_id)\
            .execute()

        if result.data and len(result.data) > 0:
            return result.data[0].get("expected_items", [])
        else:
            logger.warning(f"No inventory found for shelf {shelf_id}")
            return []
    except Exception as e:
        logger.error(f"Error fetching shelf inventory for {shelf_id}: {e}")
        return []


# =============================================================================
# Scan Results
# =============================================================================

def save_scan_results(data: dict):
    """
    Save per-shelf scan results.

    Args:
        data: Dict with scan_id, shelf_id, expected_items, detected_items,
              missing_items, unexpected_items, match
    """
    try:
        client = get_supabase_client()
        result = client.table("scan_results").insert({
            "scan_id": data.get("scan_id"),
            "shelf_id": data.get("shelf_id"),
            "expected_items": data.get("expected_items", []),
            "detected_items": data.get("detected_items", []),
            "missing_items": data.get("missing_items", []),
            "unexpected_items": data.get("unexpected_items", []),
            "match": data.get("match", False)
        }).execute()
        logger.info(f"Scan results saved: {data.get('scan_id')} / {data.get('shelf_id')}")
        return result
    except Exception as e:
        logger.error(f"Error saving scan results: {e}")
        raise


def get_scan_results(scan_id: str) -> list:
    """Get all results for a scan."""
    try:
        client = get_supabase_client()
        result = client.table("scan_results")\
            .select("*")\
            .eq("scan_id", scan_id)\
            .order("scanned_at")\
            .execute()
        return result.data
    except Exception as e:
        logger.error(f"Error fetching scan results for {scan_id}: {e}")
        return []