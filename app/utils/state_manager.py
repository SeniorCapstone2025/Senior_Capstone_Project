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


# Global state manager instance
rover_state = RoverStateManager()
