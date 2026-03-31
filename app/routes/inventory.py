"""
Inventory and Scan Management Routes
Handles scan sessions, scan results, and inventory data
"""

from fastapi import APIRouter, HTTPException, Query
from typing import List, Optional
import logging
import json
from datetime import datetime

from app.database import (
    get_scan,
    get_scan_results,
    create_scan,
    update_scan_status,
    save_scan_results,
    get_shelf_inventory,
)

logger = logging.getLogger(__name__)

router = APIRouter(
    prefix="/inventory",
    tags=["inventory"],
)


# ─────────────────────────────────────────────────────────────────────────────
# Scan Sessions
# ─────────────────────────────────────────────────────────────────────────────

@router.post("/scans")
async def create_scan_session(scan_data: dict):
    """
    Create a new scan session.
    
    Expected payload:
    {
        "scan_id": "SCN-20240318-004",
        "shelf_ids": ["A3-S1", "A3-S2", "A4-S1"],
        "operator": "M.A.R.S. Unit 01",
        "aisle": "A3–A7"
    }
    """
    try:
        scan_id = scan_data.get("scan_id")
        shelf_ids = scan_data.get("shelf_ids", [])
        
        if not scan_id:
            raise ValueError("scan_id is required")
        if not shelf_ids:
            raise ValueError("shelf_ids is required")
        
        # Create scan in database
        scan = create_scan(scan_id, shelf_ids)
        
        if not scan:
            raise ValueError("Failed to create scan")
        
        return {
            "status": "success",
            "scan": scan,
            "message": f"Scan {scan_id} created successfully"
        }
    except Exception as e:
        logger.error(f"Error creating scan: {e}")
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/scans/{scan_id}")
async def get_scan_session(scan_id: str):
    """
    Get a specific scan session with all its results.
    
    Returns:
    {
        "scan": {...},
        "results": [
            {
                "shelf_id": "A3-S1",
                "expected_items": [...],
                "detected_items": [...],
                "missing_items": [...],
                "unexpected_items": [...]
            }
        ]
    }
    """
    try:
        scan = get_scan(scan_id)
        
        if not scan:
            raise HTTPException(status_code=404, detail=f"Scan {scan_id} not found")
        
        # Get results for this scan
        results = get_scan_results(scan_id)
        
        # Calculate metrics
        metrics = _calculate_scan_metrics(results)
        
        return {
            "status": "success",
            "scan": scan,
            "results": results,
            "metrics": metrics
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching scan {scan_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.patch("/scans/{scan_id}/status")
async def update_scan_session_status(scan_id: str, status_update: dict):
    """
    Update scan status (pending, in_progress, completed, failed).
    
    Payload:
    {
        "status": "completed"
    }
    """
    try:
        status = status_update.get("status")
        
        if not status:
            raise ValueError("status is required")
        
        if status not in ("pending", "in_progress", "completed", "failed"):
            raise ValueError("Invalid status value")
        
        update_scan_status(scan_id, status)
        
        scan = get_scan(scan_id)
        
        return {
            "status": "success",
            "message": f"Scan {scan_id} status updated to {status}",
            "scan": scan
        }
    except Exception as e:
        logger.error(f"Error updating scan status: {e}")
        raise HTTPException(status_code=400, detail=str(e))


# ─────────────────────────────────────────────────────────────────────────────
# Scan Results
# ─────────────────────────────────────────────────────────────────────────────

@router.post("/scans/{scan_id}/results")
async def save_shelf_scan_results(scan_id: str, result_data: dict):
    """
    Save scan results for a specific shelf.
    
    Payload:
    {
        "shelf_id": "A3-S1",
        "expected_items": [
            {"sku": "SKU-10041", "name": "Wireless Keyboard MK-7", "qty": 4}
        ],
        "detected_items": [
            {"sku": "SKU-10041", "name": "Wireless Keyboard MK-7", "qty": 4}
        ],
        "missing_items": [],
        "unexpected_items": [],
        "match": true
    }
    """
    try:
        shelf_id = result_data.get("shelf_id")
        
        if not shelf_id:
            raise ValueError("shelf_id is required")
        
        # Add scan_id to result data
        result_data["scan_id"] = scan_id
        
        save_scan_results(result_data)
        
        return {
            "status": "success",
            "message": f"Results saved for {shelf_id}",
            "scan_id": scan_id,
            "shelf_id": shelf_id
        }
    except Exception as e:
        logger.error(f"Error saving scan results: {e}")
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/scans/{scan_id}/results")
async def get_scan_session_results(scan_id: str):
    """
    Get all scan results for a specific scan session.
    """
    try:
        scan = get_scan(scan_id)
        
        if not scan:
            raise HTTPException(status_code=404, detail=f"Scan {scan_id} not found")
        
        results = get_scan_results(scan_id)
        metrics = _calculate_scan_metrics(results)
        
        return {
            "status": "success",
            "scan_id": scan_id,
            "results": results,
            "metrics": metrics
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching scan results: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ─────────────────────────────────────────────────────────────────────────────
# Shelf Inventory
# ─────────────────────────────────────────────────────────────────────────────

@router.get("/shelves/{shelf_id}/expected-items")
async def get_shelf_expected_items(shelf_id: str):
    """
    Get expected inventory items for a specific shelf.
    
    Returns:
    {
        "shelf_id": "A3-S1",
        "expected_items": [...]
    }
    """
    try:
        items = get_shelf_inventory(shelf_id)
        
        return {
            "status": "success",
            "shelf_id": shelf_id,
            "expected_items": items
        }
    except Exception as e:
        logger.error(f"Error fetching shelf inventory: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ─────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ─────────────────────────────────────────────────────────────────────────────

def _calculate_scan_metrics(results: List[dict]) -> dict:
    """
    Calculate overall metrics from scan results.
    
    Returns metrics like total_expected, total_found, accuracy, etc.
    """
    if not results:
        return {
            "total_shelves": 0,
            "total_expected": 0,
            "total_found": 0,
            "total_missing": 0,
            "total_unexpected": 0,
            "accuracy": 0.0,
            "ok_shelves": 0,
            "warning_shelves": 0,
            "missing_shelves": 0,
            "unexpected_shelves": 0,
        }
    
    total_expected = 0
    total_found = 0
    total_missing = 0
    total_unexpected = 0
    ok_shelves = 0
    warning_shelves = 0
    missing_shelves = 0
    unexpected_shelves = 0
    
    for result in results:
        expected = result.get("expected_items", [])
        detected = result.get("detected_items", [])
        missing = result.get("missing_items", [])
        unexpected = result.get("unexpected_items", [])
        
        # Count items
        expected_count = sum(item.get("qty", 1) for item in expected)
        detected_count = sum(item.get("qty", 1) for item in detected)
        missing_count = sum(item.get("qty", 1) for item in missing)
        unexpected_count = sum(item.get("qty", 1) for item in unexpected)
        
        total_expected += expected_count
        total_found += detected_count
        total_missing += missing_count
        total_unexpected += unexpected_count
        
        # Categorize shelf status
        if missing_count > 0:
            missing_shelves += 1
        elif unexpected_count > 0:
            unexpected_shelves += 1
        elif detected_count < expected_count:
            warning_shelves += 1
        else:
            ok_shelves += 1
    
    accuracy = 0.0
    if total_expected > 0:
        accuracy = round((total_found / total_expected) * 100, 1)
    
    return {
        "total_shelves": len(results),
        "total_expected": total_expected,
        "total_found": total_found,
        "total_missing": total_missing,
        "total_unexpected": total_unexpected,
        "accuracy": accuracy,
        "ok_shelves": ok_shelves,
        "warning_shelves": warning_shelves,
        "missing_shelves": missing_shelves,
        "unexpected_shelves": unexpected_shelves,
    }
