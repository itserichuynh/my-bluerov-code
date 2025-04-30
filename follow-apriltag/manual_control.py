from pymavlink import mavutil
import time

# Connect to the ROV
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
master.wait_heartbeat()
print("Connected to system", master.target_system)

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()

# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].

for _ in range(50):  # send for ~2 seconds
    master.mav.manual_control_send(
        master.target_system,
        500,  # forward
        0,
        500,   # neutral vertical thrust
        0,
        0
    )
    time.sleep(0.1)


# clean up (disarm) at the end
master.arducopter_disarm()
master.motors_disarmed_wait()