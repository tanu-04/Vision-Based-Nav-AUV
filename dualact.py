import Jetson.GPIO as GPIO
import time
from pymavlink import mavutil

class SafetyKillSwitch:
    def __init__(self, master):
        self.master = master
        self.armed = False  # Flag to track whether the motors are armed or not

    def flick_switch(self):
        if not self.armed:
            print("arm_motors()")
        else:
            self.disarm_motors()

    def arm_motors(self):
        print("Arming the vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('Armed!')
        self.armed = True

    def disarm_motors(self):
        print("Disarming the vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        print("Waiting for the vehicle to disarm")
        self.master.motors_disarmed_wait()
        print('Disarmed!')
        self.armed = False

# Define GPIO pin
BUTTON_PIN = 15  # GPIO 15 for the button input

# Set up GPIO configuration
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN)  # Use BOARD numbering and set up the button pin

# Create the connection to MAVLink
master = mavutil.mavlink_connection('udp:192.168.2.49:14550')
master.wait_heartbeat()  # Wait for a heartbeat before sending commands
if not master.wait_heartbeat():
    print("Heartbeat not received")
else:
    print("Heartbeat received")

# Initialize SafetyKillSwitch instance
kill_switch = SafetyKillSwitch(master)

# Track the button state
prev_button_state = GPIO.input(BUTTON_PIN)

# Main loop for button state detection
try:
    while True:
        # Read the current state of the button
        button_state = GPIO.input(BUTTON_PIN)

        # If the button is flicked, flick the switch accordingly
        if button_state != prev_button_state:
            kill_switch.flick_switch()

        # Update the previous button state
        prev_button_state = button_state

        time.sleep(0.01)  # Debounce period

except KeyboardInterrupt:
    pass  # Handle Ctrl+C gracefully
