"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

print("Activate MANUAL mode")
# set the desired operating mode
MANUAL = 'MANUAL'
MANUAL_MODE = master.mode_mapping()[MANUAL]
while not master.wait_heartbeat().custom_mode == MANUAL_MODE:
    master.set_mode(MANUAL)

# Arm the vehicle
master.arducopter_arm()
master.motors_armed_wait()
print("Motors armed")


# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].
# Send a positive x value for 5 seconds.
# i = 0
# while i < 2:
#         master.mav.manual_control_send(
#             master.target_system,
#             500,
#             0,
#             0,
#             0,
#             0)
#         time.sleep(0.5)
#         i += 1
master.mav.manual_control_send(
            master.target_system,
            500,
            0,
            0,
            0,
            0)
input("Press enter to continue...")
# Disarm on exit
print("Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("Motors disarmed.")

# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC
# buttons = 1 + 1 << 3 + 1 << 7
# master.mav.manual_control_send(
#     master.target_system,
#     0,
#     0,
#     500, # 500 means neutral throttle
#     0,
#     buttons)