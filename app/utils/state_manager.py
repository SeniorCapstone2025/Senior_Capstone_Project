"""
Simple in-memory state manager for rover status
"""
from datetime import datetime
from typing import Optional


class RoverStateManager:
    def __init__(self):
        self._state = "idle"
        self._battery_level = 83.0
        self._current_task: Optional[str] = None
        self._last_update = datetime.now()

        # Caching attributes for tracking last saved state
        self._last_saved_state: Optional[str] = None
        self._last_saved_battery: Optional[float] = None
        self._last_saved_task: Optional[str] = None
        self._last_saved_timestamp: Optional[datetime] = None

    def update_state(self, command: str):
        """Update rover state based on command"""
        command_state_map = {
            "start": "scanning",
            "pause": "paused",
            "cancel": "idle",
            "search": "searching",
            "reboot": "rebooting",
            "return": "returning"
        }

        new_state = command_state_map.get(command.lower(), self._state)
        self._state = new_state
        self._last_update = datetime.now()

        # Update current task based on state
        if new_state == "idle":
            self._current_task = None
        elif new_state == "scanning":
            self._current_task = "Inventory scan in progress"
        elif new_state == "searching":
            self._current_task = "Searching for item"
        elif new_state == "returning":
            self._current_task = "Returning to base"

    def get_state(self) -> str:
        return self._state

    def get_battery_level(self) -> float:
        return self._battery_level

    def set_battery_level(self, level: float):
        self._battery_level = max(0.0, min(100.0, level))

    def get_current_task(self) -> Optional[str]:
        return self._current_task

    def get_status_dict(self) -> dict:
        return {
            "state": self._state,
            "battery_level": self._battery_level,
            "current_task": self._current_task,
            "last_update": self._last_update.isoformat()
        }

    def should_save_to_database(self, battery_threshold: float = 5.0, heartbeat_seconds: int = 300) -> bool:
        """
        Determine if current state should be saved to database.

        Returns True if:
        - State has changed
        - Current task has changed
        - Battery changed by >= battery_threshold percent
        - heartbeat_seconds have elapsed since last save
        - Never saved before (first run)

        Args:
            battery_threshold: Percentage change in battery to trigger save (default: 5.0)
            heartbeat_seconds: Seconds elapsed since last save to trigger save (default: 300)

        Returns:
            bool: True if status should be saved to database
        """
        # First run - never saved before
        if self._last_saved_timestamp is None:
            return True

        # Check if state has changed
        if self._state != self._last_saved_state:
            return True

        # Check if current task has changed
        if self._current_task != self._last_saved_task:
            return True

        # Check if battery has changed significantly
        if self._last_saved_battery is not None:
            battery_change = abs(self._battery_level - self._last_saved_battery)
            if battery_change >= battery_threshold:
                return True

        # Check if heartbeat interval has elapsed
        time_since_last_save = (datetime.now() - self._last_saved_timestamp).total_seconds()
        if time_since_last_save >= heartbeat_seconds:
            return True

        # No significant changes detected
        return False

    def mark_as_saved(self):
        """
        Record current state as saved to database.
        Updates all last_saved_* attributes with current values.
        """
        self._last_saved_state = self._state
        self._last_saved_battery = self._battery_level
        self._last_saved_task = self._current_task
        self._last_saved_timestamp = datetime.now()


# Global state manager instance
rover_state = RoverStateManager()
