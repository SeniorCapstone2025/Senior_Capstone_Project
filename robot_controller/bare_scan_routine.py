from statemachine import StateMachine, State
import json


class BareScanRoutine(StateMachine):
    # Define states
    IDLE = State('idle', initial=True)
    FIND_LOCATION = State('find')
    SCAN_QR_CODE = State('scanQR')
    VERIFY_OBJECT = State('verifyObject')
    LOG_RESULT = State('logResult')
    COMPLETE = State('complete')
    ERROR = State('error')

    # Define state transitions
    start_scan = IDLE.to(FIND_LOCATION)
    location_found = FIND_LOCATION.to(SCAN_QR_CODE)
    qr_scanned = SCAN_QR_CODE.to(VERIFY_OBJECT)
    object_verified = VERIFY_OBJECT.to(LOG_RESULT)
    log_complete = LOG_RESULT.to(COMPLETE)
    finish = COMPLETE.to(IDLE)
    handle_error = (
        FIND_LOCATION.to(ERROR) |
        SCAN_QR_CODE.to(ERROR) |
        VERIFY_OBJECT.to(ERROR) |
        LOG_RESULT.to(ERROR)
    )
    reset = ERROR.to(IDLE)

    def __init__(self):
        StateMachine.__init__(self)

        # State data
        self.target_position = None
        self.qr_data = None
        self.error_msg = None

        print("BareScanRoutine initialized in IDLE state")

    def start_scan_for_position(self, position):
        """Start a scan for the given position."""
        if self.current_state == self.IDLE:
            self.target_position = position
            print(f"Starting scan for position: {self.target_position}")
            self.start_scan()
        else:
            print(f"Cannot start scan - current state is {self.current_state}")

    def set_nav_result(self, result):
        """Simulate receiving navigation result."""
        if self.current_state == self.FIND_LOCATION:
            if result == "success":
                self.location_found()
            else:
                self.error_msg = f"Navigation failed: {result}"
                self.handle_error()

    def set_qr_result(self, qr_data):
        """Simulate receiving QR scan result."""
        if self.current_state == self.SCAN_QR_CODE:
            self.qr_data = qr_data
            if self.qr_data:
                self.qr_scanned()
            else:
                self.error_msg = "QR scan failed - no data"
                self.handle_error()

    # Transition functions for each state
    def on_enter_FIND_LOCATION(self):
        """Called when entering FIND_LOCATION state."""
        try:
            print(f"STATE: FIND_LOCATION - Navigating to {self.target_position}")
            # Simulate navigation command
            nav_cmd = {
                "goal": self.target_position,
                "action": "navigate"
            }
            print(f"Navigation command: {json.dumps(nav_cmd)}")
        except Exception as e:
            print(f"Error in on_enter_FIND_LOCATION: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def on_enter_SCAN_QR_CODE(self):
        """Called when entering SCAN_QR_CODE state."""
        try:
            print("STATE: SCAN_QR_CODE - Starting QR scan")
            # Simulate QR scan command
            qr_cmd = {
                "action": "scan_qr",
                "position": self.target_position
            }
            print(f"QR scan command: {json.dumps(qr_cmd)}")
        except Exception as e:
            print(f"Error in on_enter_SCAN_QR_CODE: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def on_enter_VERIFY_OBJECT(self):
        """Called when entering VERIFY_OBJECT state."""
        try:
            print(f"STATE: VERIFY_OBJECT - Verifying {self.qr_data}")

            is_valid = self.verify_qr_data()

            if is_valid:
                print("Object verified successfully")
                self.object_verified()
            else:
                self.error_msg = f"Verification failed for {self.target_position}"
                self.handle_error()
        except Exception as e:
            print(f"Error in on_enter_VERIFY_OBJECT: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def verify_qr_data(self):
        """Verify QR data matches expected format."""
        if not self.qr_data:
            return False
        return self.target_position in self.qr_data

    def on_enter_LOG_RESULT(self):
        """Called when entering LOG_RESULT state."""
        try:
            print(f"STATE: LOG_RESULT - Logging result for {self.target_position}")

            log_data = {
                "position": self.target_position,
                "qr_data": self.qr_data,
                "status": "success"
            }

            print(f"Log data: {json.dumps(log_data)}")
            self.log_complete()
        except Exception as e:
            print(f"Error in on_enter_LOG_RESULT: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def on_enter_COMPLETE(self):
        """Called when entering COMPLETE state."""
        try:
            print(f"STATE: COMPLETE - Scan completed for {self.target_position}")

            completion_msg = {
                "status": "complete",
                "position": self.target_position,
                "qr_data": self.qr_data
            }
            print(f"Completion: {json.dumps(completion_msg)}")

            # Reset and return to IDLE
            self.finish()
        except Exception as e:
            print(f"Error in on_enter_COMPLETE: {e}")

    def on_enter_ERROR(self):
        """Called when entering ERROR state."""
        try:
            print(f"STATE: ERROR - {self.error_msg}")

            error_data = {
                "status": "error",
                "position": self.target_position,
                "error": self.error_msg
            }
            print(f"Error data: {json.dumps(error_data)}")

            # Reset to IDLE
            self.reset()
        except Exception as e:
            print(f"Critical error in on_enter_ERROR: {e}")

    def on_enter_IDLE(self):
        """Called when entering IDLE state."""
        self.target_position = None
        self.qr_data = None
        self.error_msg = None
        print("STATE: IDLE - Ready for next command")


if __name__ == '__main__':
    # Example usage
    machine = BareScanRoutine()

    # Start a scan
    machine.start_scan_for_position("A1")

    # Simulate navigation success
    machine.set_nav_result("success")

    # Simulate QR scan result
    machine.set_qr_result("QR_A1_12345")
