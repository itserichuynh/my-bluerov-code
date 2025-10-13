import cv2
import time
import numpy as np
import apriltag
from pymavlink import mavutil
from camera_stream import Video
import csv
from datetime import datetime

from pynput import keyboard

import matplotlib.pyplot as plt
from collections import deque
from pathlib import Path

# history_len = 300  # number of frames to display

# x_errs = deque(maxlen=history_len)
# y_errs = deque(maxlen=history_len)
# z_errs = deque(maxlen=history_len)

# x_cmds = deque(maxlen=history_len)
# y_cmds = deque(maxlen=history_len)
# z_cmds = deque(maxlen=history_len)

# x_pos = deque(maxlen=history_len)
# y_pos = deque(maxlen=history_len)
# z_pos = deque(maxlen=history_len)

# def request_message_interval(connection, message, frequency_hz: float):
    
#     """
#     Request MAVLink message in a desired frequency,
#     documentation for SET_MESSAGE_INTERVAL:
#         https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

#     Args:
#         message_id (int): MAVLink message ID
#         frequency_hz (float): Desired frequency in Hz
#     """

#     message_name = "MAVLINK_MSG_ID_" + message

#     message_id = getattr(mavutil.mavlink, message_name)

#     connection.mav.command_long_send(
#         connection.target_system, connection.target_component,
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
#         message_id, # The MAVLink message ID
#         1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
#         0, 0, 0, 0, # Unused parameters
#         0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
#     )

#     print("Requested the message successfully.")

# def set_target_depth(master, depth, boot_time):
#     """ Sets the target depth while in depth-hold mode.

#     Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

#     'depth' is technically an altitude, so set as negative meters below the surface
#         -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

#     """
#     master.mav.set_position_target_global_int_send(
#         int(1e3 * (time.time() - boot_time)), # ms since boot
#         master.target_system, master.target_component,
#         coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
#         type_mask=( # ignore everything except z position
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
#             # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
#             # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
#         ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
#         vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
#         afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
#         # accelerations in NED frame [N], yaw, yaw_rate
#         #  (all not supported yet, ignored in GCS Mavlink)
#     )

# def get_alt_data(connection, message_name: str):
#     try:
#         dict = connection.recv_match(type= message_name, blocking=True, timeout=0.1).to_dict()
        
#         alt = dict['altitude']

#         print(f'Current altitude: {alt}')

#         return alt
#     except:
#         pass

# # ----------------------------
# # PID Controller Class
# # ----------------------------
# class PID:
#     def __init__(self, kp, ki, kd, output_limits=(-1000, 1000)):
#         self.kp, self.ki, self.kd = kp, ki, kd
#         self.output_limits = output_limits
#         self.integral = 0
#         self.prev_error = 0

#     def compute(self, error, dt):
#         self.integral += error * dt
#         derivative = (error - self.prev_error) / dt if dt > 0 else 0
#         output = self.kp * error + self.ki * self.integral + self.kd * derivative
#         self.prev_error = error
#         return max(self.output_limits[0], min(self.output_limits[1], round(output)))

# # ----------------------------
# # Load Camera Intrinsics
# # ----------------------------
cal_filename = "camera_calibration_brov_pool.npz"
with np.load(cal_filename) as data:
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

# print("Loaded camera matrix:\n", camera_matrix)
# print("Loaded distortion coefficients:\n", dist_coeffs)

# # ----------------------------
# # MAVLink Connection and Setup
# # ----------------------------
# connection = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
# connection.wait_heartbeat()
# print("Connected to system:", connection.target_system)
# boot_time = time.time()

# print("Activate MANUAL mode")
# # set the desired operating mode
# MANUAL = 'MANUAL'
# MANUAL_MODE = connection.mode_mapping()[MANUAL]
# while not connection.wait_heartbeat().custom_mode == MANUAL_MODE:
#     connection.set_mode(MANUAL)

# print("Switched to DEPTH HOLD mode.")
# DEPTH_HOLD = 'ALT_HOLD'  # MANUAL or ATL_HOLD
# DEPTH_HOLD_MODE = connection.mode_mapping()[DEPTH_HOLD]
# while not connection.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
#     connection.set_mode(DEPTH_HOLD)

# # # Arm the vehicle
# connection.arducopter_arm()
# connection.motors_armed_wait()
# print("Motors armed")

# set_target_depth(connection, -0.5, boot_time)

# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].
# def send_control(x=0, y=0, z=500, r=0):
#     connection.mav.manual_control_send(connection.target_system, x, y, z, r, 0)

# ----------------------------
# Camera Stream and AprilTag Detector
# ----------------------------
# video = Video(port=5601)
# while not video.frame_available():
#     print("no frame")
#     time.sleep(0.1)

detector = apriltag.Detector()
TAG_SIZE_METERS = 0.093  # side length of AprilTag in meters

# Define tag corner positions in tag frame (tag center at origin)
half = TAG_SIZE_METERS / 2.0
object_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)

# ----------------------------
# PID Controllers Setup
# ----------------------------
# forward_pid = PID(kp=400.0, ki=0.0, kd=50.0, output_limits=(-1000, 1000))
# lateral_pid = PID(kp=500.0, ki=0.0, kd=20.0, output_limits=(-1000, 1000))
# vertical_pid = PID(kp=950.0, ki=0.0, kd=20.0, output_limits=(-500, 500))

# Desired offset from the tag in tag's frame
desired_position_in_tag = np.array([0.45, 0.0, 0.0])  # 45cm in front

# Main Loop
last_time = time.time()
plt.ion()  # interactive mode for live updates
plt.figure(figsize=(10, 9))

parked = False # bool to hold whether the robot is parked

# Add a timestamp string for this scan
datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
# Open CSV files
# ahrs_log = open(f"record_data/raw_data_logs/ahrs_log_{datetime_str}.csv", "w", newline='')
# ahrs_writer = csv.writer(ahrs_log)
# ahrs_writer.writerow(["timestamp", "roll", "pitch", "yaw", "altitude", "lat", "lng"])

# tag_log = open(f"record_data/raw_data_logs/tag_pos_{datetime_str}.csv", "w", newline='')
# tag_writer = csv.writer(tag_log)
# tag_writer.writerow(["timestamp", "x_cam", "y_cam", "z_cam"])

last_seen_yaw_dir = 1

# Set the folder path
folder_path = Path("record_data/camera")

png_files = sorted(folder_path.glob("*.png"), key=lambda f: float(f.stem))

# Iterate through all PNG files

print("Starting tag-following loop (PnP mode). Press 'q' to quit.")
try:
    # while True:
    for idx, png_file in enumerate(png_files):
        # timestamp = time.time()
        # raw = video.frame()
        # if raw is None:
        #     continue
        # frame = raw.copy()
        # # save frames
        # cv2.imwrite(f"record_data/camera/{timestamp}.png", frame)
        img = cv2.imread(str(png_file))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # dt = time.time() - last_time
        # last_time = time.time()

        # ahrs_msg = connection.recv_match(type="AHRS2", blocking=True, timeout=1)
        # if ahrs_msg is None:
        #     print("AHRS2 message timeout!")
        #     continue

        # ahrs_data = ahrs_msg.to_dict()
        # roll_rad = ahrs_data.get('roll', 0.0)
        # pitch_rad = ahrs_data.get('pitch', 0.0)
        # yaw_rad = ahrs_data.get('yaw', 0.0)
        # altitude = ahrs_data.get('altitude', 0.0)
        # lat = ahrs_data.get('lat', 0)
        # lng = ahrs_data.get('lng', 0)

        # ahrs_writer.writerow([timestamp, roll_rad, pitch_rad, yaw_rad, altitude, lat, lng])

        tags = detector.detect(gray)
        if tags:
            tag = tags[0]  # pick first tag
            image_points = np.array(tag.corners, dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(object_points, image_points,
                                               camera_matrix, dist_coeffs)

            if success:
                # Get tag position in camera frame
                tvec = tvec.flatten()
                tag_position_cam = np.array([tvec[2], -tvec[0], -tvec[1]])
                # desired_position_in_tag[0] = tag_position_cam[0]

                # tag_writer.writerow([timestamp] + tag_position_cam.tolist())

                # store the last direction an apriltag was detected
                if tag_position_cam[1] > 0: # left
                    last_seen_yaw_dir = -1
                else: # right
                    last_seen_yaw_dir = 1


                # Get rotation matrix and compute desired target in camera frame
                R, _ = cv2.Rodrigues(rvec)
                desired_cam = R @ desired_position_in_tag + tag_position_cam
                # print(f"{tag_position_cam.shape}, {R.shape}, {desired_position_in_tag.shape}, {desired_cam.shape}")


                # Errors in camera frame
                x_err, y_err, z_err = desired_cam

                # x_cmd = int(forward_pid.compute(float(x_err), dt))
                # y_cmd = int(lateral_pid.compute(float(-y_err), dt))
                # z_cmd = int(500 + vertical_pid.compute(float(z_err), dt))

                # x_errs.append(x_err)
                # y_errs.append(-y_err)
                # z_errs.append(z_err)

                # x_cmds.append(x_cmd)
                # y_cmds.append(y_cmd)
                # z_cmds.append(z_cmd)

                # x_pos.append(tag_position_cam[0])
                # y_pos.append(tag_position_cam[1])
                # z_pos.append(tag_position_cam[2])

                distance_error = np.linalg.norm([x_err, y_err, z_err])
                # if distance_error < 0.08:
                #     # switch to depth hold mode
                #     print(f"Switching to depth hold mode {distance_error}")
                #     parked = True
                #     break

                # print(f"[cur pos] x={tag_position_cam[0]:.2f}, y={tag_position_cam[1]:.2f}, z={tag_position_cam[2]:.2f} | [error] x={-x_err:.2f}, y={-y_err:.2f}, z={-z_err:.2f} | x_cmd={x_cmd}, y_cmd={y_cmd}, z_cmd={z_cmd}")
                # send_control(x=x_cmd, y=y_cmd, z=z_cmd, r=0)

                print("detected")

                # Visual feedback
                for corner in tag.corners:
                    cv2.circle(img, tuple(map(int, corner)), 5, (0, 255, 0), 2)
                cx, cy = tag.center
                cv2.circle(img, (int(cx), int(cy)), 6, (0, 0, 255), -1)
                cv2.putText(img, f"{tag_position_cam[0]:.2f}m", (int(cx), int(cy - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                print("PnP failed — holding position")
                # send_control(0, 0, 500, 0)
        else:
            print("No tag detected — recovering now")
            # send_control(0, 0, 500, 200 * last_seen_yaw_dir)

        # Display frame
        cv2.namedWindow('AprilTag PnP Following', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('AprilTag PnP Following', 1280, 960)
        # cv2.imshow('AprilTag PnP Following', img)
        cv2.imwrite(f"record_data/images_with_tag/{idx}.png", img)

        # Optional: plot every N frames (e.g. every 10)
        # if len(x_errs) % 10 == 0:
        #     plt.clf()
        #     plt.subplot(3, 1, 1)
        #     plt.plot(x_pos, label='x pos')
        #     plt.plot(y_pos, label='y pos')
        #     plt.plot(z_pos, label='z pos')
        #     plt.legend()
        #     plt.title("Apriltag X, Y, Z pos in robot's frame")

        #     plt.subplot(3, 1, 2)
        #     plt.plot(x_errs, label='x error')
        #     plt.plot(y_errs, label='y error')
        #     plt.plot(z_errs, label='z error')
        #     plt.legend()
        #     plt.title("Position error")

        #     plt.subplot(3, 1, 3)
        #     plt.plot(x_cmds, label='x cmd')
        #     plt.plot(y_cmds, label='y cmd')
        #     plt.plot(z_cmds, label='z cmd')
        #     plt.legend()
        #     plt.title("PID Commands")

            # plt.pause(0.001)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass


plt.ioff()
cv2.destroyAllWindows()

if parked:
    print("Switched to DEPTH HOLD mode.")
    # DEPTH_HOLD = 'ALT_HOLD'  # MANUAL or ATL_HOLD
    # DEPTH_HOLD_MODE = connection.mode_mapping()[DEPTH_HOLD]
    # while not connection.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    #     connection.set_mode(DEPTH_HOLD)

# request_message_interval("AHRS2", 10)
# try:
#     alt = get_alt_data(connection, "AHRS2")
# except:
#     pass

input("Press enter to exit...")

# ahrs_log.close()
# tag_log.close()

print("Activate MANUAL mode")
# # set the desired operating mode
# MANUAL = 'MANUAL'
# MANUAL_MODE = connection.mode_mapping()[MANUAL]
# while not connection.wait_heartbeat().custom_mode == MANUAL_MODE:
#     connection.set_mode(MANUAL)

# # Disarm
# print("Disarming...")
# connection.arducopter_disarm()
# connection.motors_disarmed_wait()
# print("Motors disarmed.")
