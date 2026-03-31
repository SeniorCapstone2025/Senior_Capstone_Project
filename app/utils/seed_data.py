"""
Test data seeding utility for inventory scan demonstration.
This module provides functions to populate the database with sample scan data
for testing and demonstration purposes.
"""

import json
from datetime import datetime
from app.database import (
    create_scan,
    save_scan_results,
    update_scan_status,
)

def seed_demo_scan_data():
    """
    Create a demo scan with shelf results.
    This is useful for testing the InventoryMetrics component.
    """
    scan_id = "SCN-20240318-004"
    shelf_ids = ["A3-S1", "A3-S2", "A4-S1", "A4-S2", "A5-S1", "A5-S2"]
    
    # Create scan
    scan = create_scan(scan_id, shelf_ids)
    print(f"✓ Created scan: {scan_id}")
    
    # Define demo data for each shelf
    demo_shelves = [
        {
            "shelf_id": "A3-S1",
            "expected_items": [
                {"sku": "SKU-10041", "name": "Wireless Keyboard MK-7", "qty": 4},
                {"sku": "SKU-10042", "name": "Monitor Stand 27\"", "qty": 2},
                {"sku": "SKU-10043", "name": "USB-C Hub 7-Port", "qty": 6},
            ],
            "detected_items": [
                {"sku": "SKU-10041", "name": "Wireless Keyboard MK-7", "qty": 4},
                {"sku": "SKU-10042", "name": "Monitor Stand 27\"", "qty": 2},
                {"sku": "SKU-10043", "name": "USB-C Hub 7-Port", "qty": 6},
            ],
            "missing_items": [],
            "unexpected_items": [],
            "match": True,
        },
        {
            "shelf_id": "A3-S2",
            "expected_items": [
                {"sku": "SKU-10051", "name": "Ergonomic Mouse M500", "qty": 8},
                {"sku": "SKU-10052", "name": "Mouse Pad XL", "qty": 5},
                {"sku": "SKU-10053", "name": "Wrist Rest Gel", "qty": 4},
            ],
            "detected_items": [
                {"sku": "SKU-10051", "name": "Ergonomic Mouse M500", "qty": 8},
                {"sku": "SKU-10052", "name": "Mouse Pad XL", "qty": 3},
                {"sku": "SKU-10053", "name": "Wrist Rest Gel", "qty": 4},
            ],
            "missing_items": [
                {"sku": "SKU-10052", "name": "Mouse Pad XL", "qty": 2},
            ],
            "unexpected_items": [],
            "match": False,
        },
        {
            "shelf_id": "A4-S1",
            "expected_items": [
                {"sku": "SKU-10061", "name": "Webcam HD 1080p", "qty": 3},
                {"sku": "SKU-10062", "name": "Ring Light 10in", "qty": 2},
                {"sku": "SKU-10063", "name": "Desk Microphone USB", "qty": 3},
            ],
            "detected_items": [
                {"sku": "SKU-10062", "name": "Ring Light 10in", "qty": 2},
                {"sku": "SKU-10063", "name": "Desk Microphone USB", "qty": 3},
            ],
            "missing_items": [
                {"sku": "SKU-10061", "name": "Webcam HD 1080p", "qty": 3},
            ],
            "unexpected_items": [],
            "match": False,
        },
        {
            "shelf_id": "A4-S2",
            "expected_items": [
                {"sku": "SKU-10071", "name": "HDMI Cable 6ft", "qty": 10},
                {"sku": "SKU-10072", "name": "Display Port Cable", "qty": 6},
            ],
            "detected_items": [
                {"sku": "SKU-10071", "name": "HDMI Cable 6ft", "qty": 10},
                {"sku": "SKU-10072", "name": "Display Port Cable", "qty": 6},
                {"sku": "SKU-99901", "name": "VGA Adapter (unregistered)", "qty": 2},
            ],
            "missing_items": [],
            "unexpected_items": [
                {"sku": "SKU-99901", "name": "VGA Adapter (unregistered)", "qty": 2},
            ],
            "match": False,
        },
        {
            "shelf_id": "A5-S1",
            "expected_items": [
                {"sku": "SKU-10081", "name": "Laptop Stand Aluminum", "qty": 5},
                {"sku": "SKU-10082", "name": "Portable SSD 1TB", "qty": 4},
                {"sku": "SKU-10083", "name": "USB Flash Drive 64GB", "qty": 12},
            ],
            "detected_items": [
                {"sku": "SKU-10081", "name": "Laptop Stand Aluminum", "qty": 5},
                {"sku": "SKU-10082", "name": "Portable SSD 1TB", "qty": 2},
                {"sku": "SKU-10083", "name": "USB Flash Drive 64GB", "qty": 10},
            ],
            "missing_items": [
                {"sku": "SKU-10082", "name": "Portable SSD 1TB", "qty": 2},
                {"sku": "SKU-10083", "name": "USB Flash Drive 64GB", "qty": 2},
            ],
            "unexpected_items": [],
            "match": False,
        },
        {
            "shelf_id": "A5-S2",
            "expected_items": [
                {"sku": "SKU-10091", "name": "Power Strip 6-Outlet", "qty": 4},
                {"sku": "SKU-10092", "name": "Cable Management Kit", "qty": 6},
                {"sku": "SKU-10093", "name": "Surge Protector", "qty": 3},
            ],
            "detected_items": [
                {"sku": "SKU-10091", "name": "Power Strip 6-Outlet", "qty": 4},
                {"sku": "SKU-10092", "name": "Cable Management Kit", "qty": 6},
                {"sku": "SKU-10093", "name": "Surge Protector", "qty": 3},
            ],
            "missing_items": [],
            "unexpected_items": [],
            "match": True,
        },
    ]
    
    # Save results for each shelf
    for shelf_data in demo_shelves:
        result = {**shelf_data, "scan_id": scan_id}
        save_scan_results(result)
        print(f"✓ Saved results for {shelf_data['shelf_id']}")
    
    # Mark scan as completed
    update_scan_status(scan_id, "completed")
    print(f"✓ Marked scan as completed")
    
    print(f"\n✅ Demo scan data seeded successfully!")
    print(f"   Scan ID: {scan_id}")
    print(f"   Shelves: {', '.join(shelf_ids)}")


if __name__ == "__main__":
    seed_demo_scan_data()
