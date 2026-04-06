"""
Automatic scan scheduler.

Uses APScheduler to trigger periodic scans.  The schedule can be configured
at runtime through the /schedule API endpoints without restarting the server.
"""

import asyncio
import uuid
import logging
from typing import List, Optional

from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.interval import IntervalTrigger

from app.database import create_scan
from app.ros_handlers import start_scan
from app.rosbridge import rosbridge_client

logger = logging.getLogger(__name__)

JOB_ID = "auto_scan"

# In-memory state — persists for the life of the server process
_scheduler: Optional[AsyncIOScheduler] = None
_current_config: dict = {
    "enabled": False,
    "interval_minutes": 10,
    "shelf_ids": ["shelf_1", "shelf_2", "shelf_3"],
}


def get_scheduler() -> AsyncIOScheduler:
    global _scheduler
    if _scheduler is None:
        _scheduler = AsyncIOScheduler()
    return _scheduler


def get_schedule_config() -> dict:
    """Return a copy of the current schedule configuration."""
    scheduler = get_scheduler()
    job = scheduler.get_job(JOB_ID)
    config = dict(_current_config)
    config["next_run"] = job.next_run_time.isoformat() if job else None
    return config


async def _run_scheduled_scan():
    """Called by the scheduler — triggers a scan exactly like the /command/ endpoint."""
    if not rosbridge_client.is_connected:
        logger.warning("Scheduled scan skipped: rosbridge not connected")
        return

    scan_id = f"scan_{uuid.uuid4().hex[:8]}"
    shelf_ids: List[str] = _current_config["shelf_ids"]

    logger.info(f"Auto-scan starting: scan_id={scan_id}, shelves={shelf_ids}")

    # Write DB record in a thread to avoid blocking the event loop
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, lambda: _safe_create_scan(scan_id, shelf_ids))

    await start_scan(scan_id, shelf_ids)


def _safe_create_scan(scan_id: str, shelf_ids: List[str]):
    try:
        create_scan(scan_id, shelf_ids)
    except Exception as e:
        logger.warning(f"Scheduled scan DB write failed: {e}")


# ---------------------------------------------------------------------------
# Public control functions
# ---------------------------------------------------------------------------

def start_scheduler():
    """Start the APScheduler (call once at app startup)."""
    scheduler = get_scheduler()
    if not scheduler.running:
        scheduler.start()
        logger.info("Scheduler started")


def stop_scheduler():
    """Shut down the APScheduler (call at app shutdown)."""
    scheduler = get_scheduler()
    if scheduler.running:
        scheduler.shutdown(wait=False)
        logger.info("Scheduler stopped")


def enable_schedule(interval_minutes: int, shelf_ids: List[str]):
    """Add or replace the recurring scan job."""
    global _current_config
    _current_config.update(
        enabled=True,
        interval_minutes=interval_minutes,
        shelf_ids=shelf_ids,
    )

    scheduler = get_scheduler()
    # Remove existing job if present
    if scheduler.get_job(JOB_ID):
        scheduler.remove_job(JOB_ID)

    scheduler.add_job(
        _run_scheduled_scan,
        trigger=IntervalTrigger(minutes=interval_minutes),
        id=JOB_ID,
        name="Automatic inventory scan",
        replace_existing=True,
    )
    logger.info(f"Auto-scan scheduled every {interval_minutes} min on {shelf_ids}")


def disable_schedule():
    """Remove the recurring scan job."""
    global _current_config
    _current_config["enabled"] = False

    scheduler = get_scheduler()
    if scheduler.get_job(JOB_ID):
        scheduler.remove_job(JOB_ID)
        logger.info("Auto-scan schedule disabled")