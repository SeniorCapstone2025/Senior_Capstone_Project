from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from datetime import datetime
import logging

from app.database import get_supabase_client

logger = logging.getLogger(__name__)

router = APIRouter(
    prefix="/inventory", tags=["inventory"]
)


class InventoryItem(BaseModel):
    item_id: str
    name: str
    quantity: int
    description: str | None = None


@router.get("/{item_id}")
async def get_inventory_item(item_id: str):
    """
    Get inventory item by ID
    Note: Requires 'inventory' table in Supabase with columns: item_id, name, quantity, description
    """
    try:
        client = get_supabase_client()
        result = client.table("inventory")\
            .select("*")\
            .eq("item_id", item_id)\
            .execute()

        if result.data and len(result.data) > 0:
            return result.data[0]
        else:
            raise HTTPException(status_code=404, detail="Item not found")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching inventory item {item_id}: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to fetch inventory item: {str(e)}"
        )


@router.post("/")
async def create_inventory_item(item: InventoryItem):
    """
    Create new inventory item
    Note: Requires 'inventory' table in Supabase
    """
    try:
        client = get_supabase_client()
        result = client.table("inventory").insert(item.dict()).execute()
        return {
            "message": "Inventory item created successfully",
            "data": result.data[0] if result.data else None
        }
    except Exception as e:
        logger.error(f"Error creating inventory item: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to create inventory item: {str(e)}"
        )