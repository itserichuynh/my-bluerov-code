import time
from pymavlink import mavutil
from pynput import keyboard

# Connect to the ROV
connection = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
connection.wait_heartbeat()
print("Connected to system", connection.target_system)

print("Activate MANUAL mode")
# set the desired operating mode
MANUAL = 'MANUAL'
MANUAL_MODE = connection.mode_mapping()[MANUAL]
while not connection.wait_heartbeat().custom_mode == MANUAL_MODE:
    connection.set_mode(MANUAL)

# Arm the vehicle
connection.arducopter_arm()
connection.motors_armed_wait()
print("Motors armed")


# Manual control values
x = 0       # forward/backward
y = 0       # left/right
z = 500     # up/down (500 = neutral)
r = 0       # yaw
buttons = 0

# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].
def send_control():
    connection.mav.manual_control_send(
        connection.target_system,
        x, y, z, r, buttons
    )

def on_press(key):
    global x, y, z, r
    try:
        if key.char == 'w':
            x = 700
        elif key.char == 's':
            x = -700
        elif key.char == 'a':
            y = -700
        elif key.char == 'd':
            y = 700
        elif key.char == 'q':
            r = -700
        elif key.char == 'e':
            r = 700
        elif key.char == 'f':
            z = 200  # go down
        elif key.char == 'r':
            z = 800  # go up
    except AttributeError:
        pass

def on_release(key):
    global x, y, z, r
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    try:
        if key.char in ['w', 's']:
            x = 0
        elif key.char in ['a', 'd']:
            y = 0
        elif key.char in ['q', 'e']:
            r = 0
        elif key.char in ['r', 'f']:
            z = 500
    except AttributeError:
        pass

# Start a thread to listen to key events
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

print("Use keys: w/s (forward/back), a/d (left/right), q/e (yaw), r/f (up/down), ESC to stop")
try:
    while listener.running:
        send_control()
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

# Disarm on exit
print("Disarming...")
connection.arducopter_disarm()
connection.motors_disarmed_wait()