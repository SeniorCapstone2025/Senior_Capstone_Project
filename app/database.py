import sqlite3
import json
import logging
from datetime import datetime
from typing import Optional

from app.config import get_settings

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Connection helper
# ---------------------------------------------------------------------------

_db_initialized = False


def _get_connection() -> sqlite3.Connection:
    """Return a new SQLite connection, initialising the DB on first call."""
    global _db_initialized
    settings = get_settings()
    conn = sqlite3.connect(settings.db_path)
    conn.row_factory = sqlite3.Row          # dict-like access on rows
    conn.execute("PRAGMA journal_mode=WAL")  # better concurrency
    conn.execute("PRAGMA foreign_keys=ON")
    if not _db_initialized:
        _init_db(conn)
        _db_initialized = True
    return conn


def _init_db(conn: sqlite3.Connection):
    """Create tables + seed data (idempotent)."""
    cur = conn.cursor()

    cur.executescript("""
        CREATE TABLE IF NOT EXISTS command_logs (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            command     TEXT NOT NULL,
            status      TEXT NOT NULL,
            timestamp   TEXT NOT NULL DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS status_logs (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            state       TEXT NOT NULL,
            battery     REAL NOT NULL,
            current_task TEXT,
            timestamp   TEXT NOT NULL DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS detections (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            object_name TEXT NOT NULL,
            confidence  REAL NOT NULL,
            x           REAL NOT NULL,
            y           REAL NOT NULL,
            detected_at TEXT NOT NULL DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS inventory (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            item_id     TEXT NOT NULL UNIQUE,
            name        TEXT NOT NULL,
            quantity    INTEGER NOT NULL DEFAULT 0,
            description TEXT
        );

        CREATE TABLE IF NOT EXISTS scans (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            scan_id      TEXT NOT NULL UNIQUE,
            status       TEXT NOT NULL DEFAULT 'pending',
            shelf_ids    TEXT NOT NULL DEFAULT '[]',
            created_at   TEXT NOT NULL DEFAULT (datetime('now')),
            completed_at TEXT
        );

        CREATE TABLE IF NOT EXISTS scan_results (
            id               INTEGER PRIMARY KEY AUTOINCREMENT,
            scan_id          TEXT NOT NULL,
            shelf_id         TEXT NOT NULL,
            expected_items   TEXT NOT NULL DEFAULT '[]',
            detected_items   TEXT NOT NULL DEFAULT '[]',
            missing_items    TEXT NOT NULL DEFAULT '[]',
            unexpected_items TEXT NOT NULL DEFAULT '[]',
            match            INTEGER NOT NULL DEFAULT 0,
            scanned_at       TEXT NOT NULL DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS shelf_inventory (
            id             INTEGER PRIMARY KEY AUTOINCREMENT,
            shelf_id       TEXT NOT NULL UNIQUE,
            expected_items TEXT NOT NULL DEFAULT '[]'
        );
    """)

    # Seed shelf_inventory (ignore if rows already exist)
    seed = [
        ("shelf_1", json.dumps(["bottle", "mouse"])),
        ("shelf_2", json.dumps(["cup", "book"])),
        ("shelf_3", json.dumps(["keyboard", "mouse"])),
        ("shelf_4", json.dumps(["bottle", "book"])),
    ]
    cur.executemany(
        "INSERT OR IGNORE INTO shelf_inventory (shelf_id, expected_items) VALUES (?, ?)",
        seed,
    )
    conn.commit()
    logger.info("SQLite database initialised")


def _row_to_dict(row: sqlite3.Row) -> dict:
    """Convert a sqlite3.Row to a plain dict."""
    return dict(row)


# ---------------------------------------------------------------------------
# Command logs
# ---------------------------------------------------------------------------

def save_command(command: str, status: str):
    """Save command log."""
    conn = _get_connection()
    try:
        conn.execute(
            "INSERT INTO command_logs (command, status) VALUES (?, ?)",
            (command, status),
        )
        conn.commit()
        logger.info(f"Command saved: {command} - {status}")
    except Exception as e:
        logger.error(f"Error saving command: {e}")
        raise
    finally:
        conn.close()


def get_command_logs(limit: int = 50):
    """Retrieve command logs."""
    conn = _get_connection()
    try:
        cur = conn.execute(
            "SELECT * FROM command_logs ORDER BY timestamp DESC LIMIT ?",
            (limit,),
        )
        return [_row_to_dict(r) for r in cur.fetchall()]
    except Exception as e:
        logger.error(f"Error fetching command logs: {e}")
        raise
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# Status logs
# ---------------------------------------------------------------------------

def save_status(state: str, battery: float, current_task: Optional[str]):
    """Save status log."""
    conn = _get_connection()
    try:
        conn.execute(
            "INSERT INTO status_logs (state, battery, current_task) VALUES (?, ?, ?)",
            (state, battery, current_task),
        )
        conn.commit()
        logger.info(f"Status saved: {state} - Battery: {battery}%")
    except Exception as e:
        logger.error(f"Error saving status: {e}")
        raise
    finally:
        conn.close()


def get_status_logs(limit: int = 50):
    """Retrieve status logs."""
    conn = _get_connection()
    try:
        cur = conn.execute(
            "SELECT * FROM status_logs ORDER BY timestamp DESC LIMIT ?",
            (limit,),
        )
        return [_row_to_dict(r) for r in cur.fetchall()]
    except Exception as e:
        logger.error(f"Error fetching status logs: {e}")
        raise
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# Detections
# ---------------------------------------------------------------------------

def save_detected_object(object_name: str, confidence: float, x: float, y: float):
    """Save detected object."""
    conn = _get_connection()
    try:
        conn.execute(
            "INSERT INTO detections (object_name, confidence, x, y) VALUES (?, ?, ?, ?)",
            (object_name, confidence, x, y),
        )
        conn.commit()
        logger.debug(f"Detection saved: {object_name} ({confidence:.2f})")
    except Exception as e:
        logger.error(f"Error saving detection: {e}")
        raise
    finally:
        conn.close()


def get_detection_logs(limit: int = 50):
    """Retrieve detection logs."""
    conn = _get_connection()
    try:
        cur = conn.execute(
            "SELECT * FROM detections ORDER BY detected_at DESC LIMIT ?",
            (limit,),
        )
        return [_row_to_dict(r) for r in cur.fetchall()]
    except Exception as e:
        logger.error(f"Error fetching detection logs: {e}")
        raise
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# Inventory
# ---------------------------------------------------------------------------

def get_inventory_item(item_id: str):
    """Get inventory item by item_id. Returns dict or None."""
    conn = _get_connection()
    try:
        cur = conn.execute(
            "SELECT * FROM inventory WHERE item_id = ?", (item_id,)
        )
        row = cur.fetchone()
        return _row_to_dict(row) if row else None
    except Exception as e:
        logger.error(f"Error fetching inventory item {item_id}: {e}")
        return None
    finally:
        conn.close()


def create_inventory_item(item_id: str, name: str, quantity: int, description: Optional[str] = None):
    """Insert a new inventory item. Returns the created row as dict."""
    conn = _get_connection()
    try:
        conn.execute(
            "INSERT INTO inventory (item_id, name, quantity, description) VALUES (?, ?, ?, ?)",
            (item_id, name, quantity, description),
        )
        conn.commit()
        cur = conn.execute(
            "SELECT * FROM inventory WHERE item_id = ?", (item_id,)
        )
        row = cur.fetchone()
        return _row_to_dict(row) if row else None
    except Exception as e:
        logger.error(f"Error creating inventory item: {e}")
        raise
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# Scan management
# ---------------------------------------------------------------------------

def create_scan(scan_id: str, shelf_ids: list) -> Optional[dict]:
    """Create a new scan record."""
    conn = _get_connection()
    try:
        conn.execute(
            "INSERT INTO scans (scan_id, status, shelf_ids) VALUES (?, 'pending', ?)",
            (scan_id, json.dumps(shelf_ids)),
        )
        conn.commit()
        logger.info(f"Scan created: {scan_id} for shelves {shelf_ids}")
        cur = conn.execute("SELECT * FROM scans WHERE scan_id = ?", (scan_id,))
        row = cur.fetchone()
        if row:
            d = _row_to_dict(row)
            d["shelf_ids"] = json.loads(d["shelf_ids"])
            return d
        return None
    except Exception as e:
        logger.error(f"Error creating scan {scan_id}: {e}")
        raise
    finally:
        conn.close()


def update_scan_status(scan_id: str, status: str):
    """Update scan status."""
    conn = _get_connection()
    try:
        if status in ("completed", "failed"):
            completed_at = datetime.now().isoformat()
            conn.execute(
                "UPDATE scans SET status = ?, completed_at = ? WHERE scan_id = ?",
                (status, completed_at, scan_id),
            )
        else:
            conn.execute(
                "UPDATE scans SET status = ? WHERE scan_id = ?",
                (status, scan_id),
            )
        conn.commit()
        logger.info(f"Scan {scan_id} status updated to {status}")
    except Exception as e:
        logger.error(f"Error updating scan {scan_id} status: {e}")
        raise
    finally:
        conn.close()


def get_scan(scan_id: str) -> Optional[dict]:
    """Get scan by ID."""
    conn = _get_connection()
    try:
        cur = conn.execute("SELECT * FROM scans WHERE scan_id = ?", (scan_id,))
        row = cur.fetchone()
        if row:
            d = _row_to_dict(row)
            d["shelf_ids"] = json.loads(d["shelf_ids"])
            return d
        return None
    except Exception as e:
        logger.error(f"Error fetching scan {scan_id}: {e}")
        return None
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# Shelf inventory
# ---------------------------------------------------------------------------

def get_shelf_inventory(shelf_id: str) -> list:
    """Get expected inventory items for a shelf."""
    conn = _get_connection()
    try:
        cur = conn.execute(
            "SELECT expected_items FROM shelf_inventory WHERE shelf_id = ?",
            (shelf_id,),
        )
        row = cur.fetchone()
        if row:
            return json.loads(row["expected_items"])
        logger.warning(f"No inventory found for shelf {shelf_id}")
        return []
    except Exception as e:
        logger.error(f"Error fetching shelf inventory for {shelf_id}: {e}")
        return []
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# Scan results
# ---------------------------------------------------------------------------

def save_scan_results(data: dict):
    """Save per-shelf scan results."""
    conn = _get_connection()
    try:
        conn.execute(
            """INSERT INTO scan_results
               (scan_id, shelf_id, expected_items, detected_items,
                missing_items, unexpected_items, match)
               VALUES (?, ?, ?, ?, ?, ?, ?)""",
            (
                data.get("scan_id"),
                data.get("shelf_id"),
                json.dumps(data.get("expected_items", [])),
                json.dumps(data.get("detected_items", [])),
                json.dumps(data.get("missing_items", [])),
                json.dumps(data.get("unexpected_items", [])),
                1 if data.get("match", False) else 0,
            ),
        )
        conn.commit()
        logger.info(f"Scan results saved: {data.get('scan_id')} / {data.get('shelf_id')}")
    except Exception as e:
        logger.error(f"Error saving scan results: {e}")
        raise
    finally:
        conn.close()


def get_scan_results(scan_id: str) -> list:
    """Get all results for a scan."""
    conn = _get_connection()
    try:
        cur = conn.execute(
            "SELECT * FROM scan_results WHERE scan_id = ? ORDER BY scanned_at",
            (scan_id,),
        )
        rows = []
        for row in cur.fetchall():
            d = _row_to_dict(row)
            d["expected_items"] = json.loads(d["expected_items"])
            d["detected_items"] = json.loads(d["detected_items"])
            d["missing_items"] = json.loads(d["missing_items"])
            d["unexpected_items"] = json.loads(d["unexpected_items"])
            d["match"] = bool(d["match"])
            rows.append(d)
        return rows
    except Exception as e:
        logger.error(f"Error fetching scan results for {scan_id}: {e}")
        return []
    finally:
        conn.close()
