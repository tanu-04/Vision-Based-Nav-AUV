import math
import rospy
from std_msgs.msg import Float32, UInt16
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import os
import csv
import time
import random
from mavros_msgs.msg import OverrideRCIn
from scam_quali import missionstart
import armros


# Global variables for roll and distance values
roll_value = 320
distance_value = 0.3
roll_value_received = False
distance_value_received = False

# Callback function to update roll value
def roll_callback(msg):
    global roll_value
    global roll_value_received
    roll_value_received = True
    roll_value = msg.data
    print("Roll value updated:", roll_value)
    pass

# Callback function to update distance value
def distance_callback(msg):
    global distance_value
    global distance_value_received
    distance_value_received = True
    distance_value = msg.data
    print("Distance value updated:", distance_value)
    pass
class PIDController:
    def __init__(self, Kp_roll, Ki_roll, Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch, setpoint_roll, setpoint_pitch, sample_time, log_folder):
        # PID parameters for roll axis
        self.Kp_roll = Kp_roll
        self.Ki_roll = Ki_roll
        self.Kd_roll = Kd_roll
        self.Kp_pitch = Kp_pitch
        self.Ki_pitch = Ki_pitch
        self.Kd_pitch = Kd_pitch

        self.setpoint_roll = setpoint_roll
        self.setpoint_pitch = setpoint_pitch
        self.sample_time = sample_time
        self.prev_time = time.time()
        self.log_folder = log_folder  # Log folder path

        # Initialize previous error and integral terms for roll
        self.prev_error_roll = 0
        self.integral_roll = 0
        self.count = 0
        self.prev_error_pitch = 0
        self.integral_pitch = 0
        # Initialize output for roll
        self.output_roll = 0  # Initial output set to 1500
        self.output_pitch = 0
        # Create indexed CSV file for logging
        self.create_log_file()

    def create_log_file(self):
        # Create folder if it doesn't exist
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

        # Generate log file name in the format log_dd_mm_yy_hh_mm_ss.csv
        log_file = 'log_' + time.strftime("%d_%m_%y_%H_%M_%S") + '.csv'
        self.log_file_path = os.path.join(self.log_folder, log_file)

        # Create or open the log file in append mode
        self.log_file = open(self.log_file_path, 'a', newline='')
        self.log_writer = csv.writer(self.log_file)

    def update(self, rollinput):
        self.count += 1
        # Compute time difference since last update
        current_time = time.time()
        dt = current_time - self.prev_time
        print("Current time:", current_time, "Previous time:", self.prev_time)

        # Check if enough time has passed for a new update
        if dt >= self.sample_time:
            # Compute error terms for roll axis
            error_roll = self.setpoint_roll - rollinput[0]
            self.integral_roll += error_roll * dt
            derivative_roll = (error_roll - self.prev_error_roll) / dt
            error_pitch = self.setpoint_pitch - rollinput[1]
            self.integral_pitch += error_roll * dt
            derivative_pitch = (error_roll - self.prev_error_pitch) / dt

            # Compute PID output for roll axis
            self.output_roll = (
                self.Kp_roll * error_roll +
                self.Ki_roll * self.integral_roll +
                self.Kd_roll * derivative_roll
            )
            self.output_pitch = (
                self.Kp_pitch * error_pitch +
                self.Ki_pitch * self.integral_pitch +
                self.Kd_pitch * derivative_pitch
            )
            self.output_pitch = int(max(1000, min(2000, 1500+ self.output_pitch)))
            # Adjust output based on rollinput
            if rollinput[0] == self.setpoint_roll:
                print("Case 2: Roll input is 320", "Roll input:", rollinput, "Output roll:", self.output_roll)
                self.output_roll = 1500
            else:
                print("Case 1: Roll input is not 320", "Roll input:", rollinput, "Output roll:", self.output_roll)
                self.output_roll = int(max(1000, min(2000, 1500 + self.output_roll)))

            # Log the data for roll
            print("Count:", self.count)
            self.log_data(current_time, [self.output_roll, self.output_pitch], rollinput)
            time.sleep(0.095)
            # Update previous time and error for next iteration
            self.prev_time = current_time
            self.prev_error_roll = error_roll
            self.prev_error_pitch = error_pitch

        return [self.output_roll, self.output_pitch]

    def log_data(self, timestamp, output, input):
        # Write data to the log file
        self.log_writer.writerow([timestamp, output[0], input[0], output[1], input[1]])

def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=( # ignore everything except z position
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

def set_rc_channel_pwm(user_input, value):
    if user_input == 1:
        x = value  # Example value for x
        y = 0      # Example value for y
        z = 0      # Example value for z
        r = 0      # Example value for r
        button = 0 # Example value for button
        master.mav.manual_control_send(master.target_system, x, y, z, r, button)
    elif user_input == 2:
        x = 0      # Example value for x
        y = value  # Example value for y
        z = 0      # Example value for z
        r = 0      # Example value for r
        button = 0 # Example value for button
        master.mav.manual_control_send(master.target_system, x, y, z, r, button)
    else:
        print("Invalid input. Please enter either 1 or 2.")

master = mavutil.mavlink_connection('udp:192.168.2.49:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
#master.wait_heartbeat()

def main():
    # Initialize ROS node
    rospy.init_node('pid_controller_node', anonymous=True)
    time.sleep(10)
    armros.arm_vehicle()

    # Initialize PID parameters
    Kp_roll = 0.50
    Ki_roll = 0.50
    Kd_roll = 0.10
    Kp_pitch = 0.50
    Ki_pitch = 0.50
    Kd_pitch = 0.10

    setpoint_roll = 640
    setpoint_pitch = 0.8
    sample_time = 0.1
    log_folder = '/home/vyana/logs/dive_logs/'

    # Initialize PID controller
    pid = PIDController(
        Kp_roll, Ki_roll, Kd_roll,
        Kp_pitch, Ki_pitch, Kd_pitch,
        setpoint_roll, setpoint_pitch, sample_time,
        log_folder=log_folder
    )

    # Subscribe to roll and distance topics
    rospy.Subscriber('/custom_node/pixel_x', UInt16, roll_callback)
    rospy.Subscriber('/custom_node/distance', Float32, distance_callback)
    print("1")
    i = 0
    while not rospy.is_shutdown():
        if not roll_value_received and i == 0:
            #missionstart()
            print("mission start")
            i = 1
            
        elif roll_value_received:
            
            # Process data and apply PID controller
            timerandom = random.uniform(1, 2)
            print("Roll value:", roll_value, "Distance value:", distance_value)
            time.sleep(0.7)
            pid_output = pid.update([roll_value, distance_value])

            # Apply PID output to your system (e.g., set PWM value)
            # For this example, we're just printing out the PID output
            print("Roll PID Output:", pid_output[0])
            print("Pitch PID Output:", pid_output[1])
            rc = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

            # Create a message object
            rc_msg = OverrideRCIn()

            # Set RC channel values
            rc_msg.channels = [1500, 1500, 1500, 1500,1500 , pid_output[0], 0, 0, 0, 0, 0, 0, 1100, 1100, 0, 1500, 0, 0]
            # Publish the message repeatedly at 10 Hz
            rc.publish(rc_msg)

            # Log data if needed
            pid.log_data(time.time(), pid_output, [roll_value, distance_value])

        # Add a delay or condition to control loop frequency
        time.sleep(0.1)

if __name__ == '__main__':
    print("PID CONTROLLER")
    main()
