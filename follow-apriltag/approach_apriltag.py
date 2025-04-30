import cv2
import time
import numpy as np
import apriltag
from pymavlink import mavutil
from camera_stream import Video

# ----------------------------
# PID Controller Class
# ----------------------------
class PID:
    def __init__(self, kp, ki, kd, output_limits=(-1000, 1000)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limits = output_limits
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return max(self.output_limits[0], min(self.output_limits[1], round(output)))

# ----------------------------
# MAVLink Connection and Setup
# ----------------------------
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
master.wait_heartbeat()
print("Connected to system:", master.target_system)

# Arm the ROV (Button 7 = ArduSub arm)
def arm():
    print("Arming ROV...")
    buttons = 1 << 7
    for _ in range(5):
        master.mav.manual_control_send(master.target_system, 0, 0, 500, 0, buttons)
        time.sleep(0.1)

# Send manual control command (X = forward, Z = vertical thrust, R = yaw)
def move_rov(x=0, y=0, z=500, r=0):
    master.mav.manual_control_send(master.target_system, x, y, z, r, 0)

# Request regular VFR_HUD messages (contains altitude)
def request_vfr_hud(frequency_hz=10):
    msg_id = mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD
    interval_us = 1e6 / frequency_hz
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, interval_us, 0, 0, 0, 0, 0
    )
    print("Requested VFR_HUD messages at", frequency_hz, "Hz")

# ----------------------------
# Camera Stream and AprilTag Detector
# ----------------------------
video = Video(port=5601)
while not video.frame_available():
    time.sleep(0.1)

detector = apriltag.Detector()
TAG_SIZE_METERS = 0.18  # size of your AprilTag
FOCAL_LENGTH_PIXELS = 600  # approximate camera focal length

# ----------------------------
# PID Controllers Setup
# ----------------------------
forward_pid = PID(kp=1000.0, ki=0.0, kd=0.0, output_limits=(0, 1000))
yaw_pid = PID(kp=1.0, ki=0.0, kd=0.2, output_limits=(-1000, 1000))
depth_pid = PID(kp=300.0, ki=0.0, kd=20.0, output_limits=(-500, 500))  # note: offset to 500 later

desired_distance = 0.6  # desired tag-following distance in meters
desired_depth = 1.5     # desired depth in meters (altitude from surface)

# Request altitude data
request_vfr_hud(5)
# arm()
last_time = time.time()

print("Starting tag-following loop. Press 'q' to quit.")
while True:
    # --------------------
    # Get camera frame
    # --------------------
    raw = video.frame()
    if raw is None:
        continue
    frame = raw.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dt = time.time() - last_time
    last_time = time.time()

    # --------------------
    # Read altitude (depth)
    # --------------------
    z_command = 500  # default neutral vertical thrust
    alt_msg = master.recv_match(type= "AHRS2", blocking=True, timeout=0.1).to_dict()
    if alt_msg:
        print(f"[DEBUG] Current altitude (VFR_HUD.alt): {alt_msg['altitude']}")
        current_depth = alt_msg['altitude']  # ArduSub gives altitude above seabed in meters
        depth_error = current_depth - desired_depth
        z_offset = depth_pid.compute(depth_error, dt)
        z_command = int(500 + z_offset)
        z_command = max(0, min(1000, z_command))

    # --------------------
    # AprilTag detection and control
    # --------------------
    tags = detector.detect(gray)
    if tags:
        tag = tags[0]  # use the first detected tag
        cx, cy = tag.center
        width = frame.shape[1]
        error_x = cx - width / 2  # left/right offset

        # Estimate distance from tag width
        tag_width = np.linalg.norm(np.array(tag.corners[0]) - np.array(tag.corners[1]))
        distance_m = (TAG_SIZE_METERS * FOCAL_LENGTH_PIXELS) / tag_width

        # Forward motion
        forward_output = forward_pid.compute(distance_m - desired_distance, dt)
        forward_command = max(0, forward_output)
        if 0 < forward_command < 100:
            forward_command = 100

        # Yaw rotation
        yaw_output = yaw_pid.compute(-error_x, dt)
        yaw_command = int(yaw_output)
        if -100 < yaw_command < 100:
            yaw_command = 0

        print(f"Tag seen. Distance: {distance_m:.2f} m | Forward: {forward_command}, Yaw: {yaw_command}, Z: {z_command}")
        # move_rov(x=forward_command, z=z_command, r=yaw_command)

        # Visual feedback
        for corner in tag.corners:
            cv2.circle(frame, tuple(map(int, corner)), 5, (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)
        cv2.putText(frame, f"{distance_m:.2f}m", (int(cx), int(cy - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        print("No tag detected — holding position")
        # move_rov(0, 0, z_command, 0)

    # --------------------
    # Show video feed
    # --------------------
    cv2.namedWindow('AprilTag Following with Depth Hold', cv2.WINDOW_NORMAL)  # allow manual resizing
    cv2.resizeWindow('AprilTag Following with Depth Hold', 640, 480) 
    cv2.imshow('AprilTag Following with Depth Hold', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
