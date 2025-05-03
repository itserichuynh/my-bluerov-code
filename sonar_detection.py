import time
import numpy as np
import csv
from pymavlink import mavutil
import os
import math
import cv2
import gi
from bluerov_ping.brping import Ping360
import matplotlib.pyplot as plt
from datetime import datetime
import argparse

parser = argparse.ArgumentParser(description="BlueROV stuff lol")
parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 192.168.2.2:9092")
parser.add_argument('--mavlink', action="store", required=False, type=str, help="Mavlink UDP. E.g: udp:192.168.2.1:14551")
parser.add_argument('--camera', action="store", required=False, type=str, help="Camera UDP port. E.g: 5601")


args = parser.parse_args()
if args.udp is None or args.mavlink is None or args.camera is None:
    parser.print_help()
    exit(1)

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# -----------------------------
# Gstreamer class to access camera stream
# -----------------------------

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5602):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK

# -----------------------------
# Helper Functions
# -----------------------------

def correct_sonar_scan_with_rp(sonar_data, desired_range_meters, num_samples, roll_rad_list, pitch_rad_list, angle_start):
    corrected_points = []
    num_angles = sonar_data.shape[0]
    angles_rad = (np.arange(num_angles) + angle_start) * (2 * np.pi / 400)
    radii = np.linspace(0, desired_range_meters, num_samples)

    for angle_idx, angle_rad in enumerate(angles_rad):
        roll = roll_rad_list[angle_idx]
        pitch = pitch_rad_list[angle_idx]

        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        R_pitch = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R = np.dot(R_pitch, R_roll)

        for sample_idx, radius in enumerate(radii):
            intensity = sonar_data[angle_idx, sample_idx]
            if intensity > 20:
                x = radius * np.cos(angle_rad)
                y = radius * np.sin(angle_rad)
                z = 0.0
                rotated_point = np.dot(R, np.array([x, y, z]))
                corrected_points.append((rotated_point[0], rotated_point[1]))
    return corrected_points

def correct_sonar_scan_with_rpy(sonar_data, desired_range_meters, num_samples, roll_rad_list, pitch_rad_list, yaw_rad_list, angle_start):
    corrected_points = []
    num_angles = sonar_data.shape[0]
    angles_rad = (np.arange(num_angles) + angle_start)* (2 * np.pi / 400)
    radii = np.linspace(0, desired_range_meters, num_samples)

    for angle_idx, angle_rad in enumerate(angles_rad):
        roll = roll_rad_list[angle_idx]
        pitch = pitch_rad_list[angle_idx]
        yaw = yaw_rad_list[angle_idx]

        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        R_pitch = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_rp = np.dot(R_pitch, R_roll)

        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])

        for sample_idx, radius in enumerate(radii):
            intensity = sonar_data[angle_idx, sample_idx]
            if intensity > 20:
                x = radius * np.cos(angle_rad)
                y = radius * np.sin(angle_rad)
                z = 0.0
                rotated_point = np.dot(R_rp, np.array([x, y, z]))
                rotated_2d = np.dot(R_yaw, rotated_point[:2])
                corrected_points.append((rotated_2d[0], rotated_2d[1]))
    return corrected_points

def sonar_scan_without_correction(sonar_data, desired_range_meters, num_samples, angle_start):
    raw_points = []
    num_angles = sonar_data.shape[0]
    angles_rad = (np.arange(num_angles) + angle_start) * (2 * np.pi / 400)
    radii = np.linspace(0, desired_range_meters, num_samples)

    for angle_idx, angle_rad in enumerate(angles_rad):
        for sample_idx, radius in enumerate(radii):
            intensity = sonar_data[angle_idx, sample_idx]
            if intensity > 20:
                x = radius * np.cos(angle_rad)
                y = radius * np.sin(angle_rad)
                raw_points.append((x, y))
    return raw_points

class OccupancyGrid2D:
    def __init__(self, width_m, height_m, resolution_m):
        self.width = int(width_m / resolution_m)
        self.height = int(height_m / resolution_m)
        self.resolution = resolution_m
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

    def world_to_grid(self, x, y):
        ix = int((x + (self.width * self.resolution) / 2) / self.resolution)
        iy = int((y + (self.height * self.resolution) / 2) / self.resolution)
        return ix, iy

    def add_points(self, points):
        for (x, y) in points:
            ix, iy = self.world_to_grid(x, y)
            if 0 <= ix < self.width and 0 <= iy < self.height:
                self.grid[iy, ix] = 1

def request_message_interval(message, frequency_hz: float):
    message_name = "MAVLINK_MSG_ID_" + message
    message_id = getattr(mavutil.mavlink, message_name)
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        1e6 / frequency_hz,
        0, 0, 0, 0, 0
    )
    print(f"Requested {message} message at {frequency_hz} Hz")

def arm_brov(connection):
    connection.arducopter_arm()
    connection.motors_armed_wait()
    print("Motors armed")

def disarm_brov(connection):
    print("Disarming...")
    connection.arducopter_disarm()
    connection.motors_disarmed_wait()

def depth_hold_mode(connection):
    print("Depth hold mode")
    DEPTH_HOLD = 'ALT_HOLD'  # MANUAL or ATL_HOLD
    DEPTH_HOLD_MODE = connection.mode_mapping()[DEPTH_HOLD]
    while not connection.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
        connection.set_mode(DEPTH_HOLD)

def manual_mode(connection):
    print("Manual mode")
    DEPTH_HOLD = 'MANUAL'  # MANUAL or ATL_HOLD
    DEPTH_HOLD_MODE = connection.mode_mapping()[DEPTH_HOLD]
    while not connection.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
        connection.set_mode(DEPTH_HOLD)

def query_current_state(connection):
    # Get the latest heartbeat message
    heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
    is_armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    return is_armed

def set_target_depth(connection, depth, boot_time):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    connection.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        connection.target_system, connection.target_component,
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

def set_servo_pwm(connection, servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n + 8,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )

def stop(connection):
    print("Stop all thrusters...")
    # Send command to ROV
    connection.mav.manual_control_send(
        connection.target_system,
        0, 0, 500, 0,
        0  # button mask
    )

    print("Disarming...")
    connection.arducopter_disarm()
    connection.motors_disarmed_wait()

def request_message_interval(connection, message, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """

    message_name = "MAVLINK_MSG_ID_" + message

    message_id = getattr(mavutil.mavlink, message_name)

    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

    print(f"Requested {message} successfully.")

def get_altitude(connection, message_name: str):
    try:
        dict = connection.recv_match(type= message_name, blocking=True, timeout=0.1).to_dict()
        alt = dict['altitude']
        print(f'Current altitude: {alt}')
        # return alt
    except:
        pass

def cv2_keyboard_control(connection, video):
    print("""
        Use WASD to move horizontally
        Use R/F to ascend/descend
        Use Q/E to yaw
        Press X to stop all motion
        Press ESC to quit control

        Make sure the OpenCV window is focused.
        """)

    x = y = r = 0
    z = 500  # neutral vertical throttle

    while True:
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = video.frame()
            cv2.namedWindow('camera frame', cv2.WINDOW_NORMAL)  # allow manual resizing
            cv2.resizeWindow('camera frame', 640, 480) 

            cv2.imshow('camera frame', frame)
            # cv2.waitKey(1)
        key = cv2.waitKey(100) & 0xFF  # 100ms delay between key polls

        if key == ord('w'):
            x = 500
        elif key == ord('s'):
            x = -500
        else:
            x = 0

        if key == ord('a'):
            y = -500
        elif key == ord('d'):
            y = 500
        else:
            y = 0

        if key == ord('r'):
            z = 800
        elif key == ord('f'):
            z = 200
        else:
            z = 500

        if key == ord('q'):
            r = -500
        elif key == ord('e'):
            r = 500
        else:
            r = 0

        if key == ord('x'):
            x = y = r = 0
            z = 500

        if key == 27:  # ESC key
            print("Exiting control mode.")
            break

        # Send command to ROV
        connection.mav.manual_control_send(
            connection.target_system,
            x, y, z, r,
            0  # button mask
        )

        print(f"[SEND] x={x}, y={y}, z={z}, r={r}")

    cv2.destroyWindow("camera frame")

def sonar_detection(connection, video, ping_sonar, angle_start, angle_end, angle_range, num_samples, desired_range_meters):
    # Add a timestamp string for this scan
    datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Open CSV files
    ahrs_log = open(f"record_data/raw_data_logs/ahrs_log_{datetime_str}.csv", "w", newline='')
    sonar_log = open(f"record_data/raw_data_logs/sonar_log_{datetime_str}.csv", "w", newline='')
    ahrs_writer = csv.writer(ahrs_log)
    sonar_writer = csv.writer(sonar_log)
    ahrs_writer.writerow(["timestamp", "roll", "pitch", "yaw", "altitude", "lat", "lng"])
    sonar_writer.writerow(["timestamp", "angle"] + [f"sample_{i}" for i in range(num_samples)])

    # -----------------------------
    # Data Collection
    # -----------------------------

    sonar_data = np.zeros((angle_range, num_samples), dtype=np.uint8)
    roll_rad_list, pitch_rad_list, yaw_rad_list = [], [], []

    print("Starting synchronized data collection...")

    t_start = time.time()
    for angle in range(angle_start, angle_end + 1):
        timestamp = time.time()
        response = ping_sonar.transmitAngle(angle)

        ahrs_msg = connection.recv_match(type="AHRS2", blocking=True, timeout=1)
        if ahrs_msg is None:
            print("AHRS2 message timeout!")
            continue

        ahrs_data = ahrs_msg.to_dict()
        roll_rad = ahrs_data.get('roll', 0.0)
        pitch_rad = ahrs_data.get('pitch', 0.0)
        yaw_rad = ahrs_data.get('yaw', 0.0)
        altitude = ahrs_data.get('altitude', 0.0)
        lat = ahrs_data.get('lat', 0)
        lng = ahrs_data.get('lng', 0)

        roll_rad_list.append(roll_rad)
        pitch_rad_list.append(pitch_rad)
        yaw_rad_list.append(yaw_rad)

        ahrs_writer.writerow([timestamp, roll_rad, pitch_rad, yaw_rad, altitude, lat, lng])

        if response:
            data = np.frombuffer(response.data, dtype=np.uint8)
            sonar_data[angle - angle_start] = data
            sonar_writer.writerow([timestamp, angle] + data.tolist())
            print(f"[Angle {angle}] Collected sonar + AHRS2")
        else:
            print(f"[Angle {angle}] Sonar data missing")
        
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = video.frame()
            cv2.namedWindow('frame', cv2.WINDOW_NORMAL)  # allow manual resizing
            cv2.resizeWindow('frame', 640, 480) 

            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            cv2.imwrite(f"record_data/camera/{timestamp}.png", frame)
            print("Camera frame successfully captured.")
        else:
            print("Frame not available :(")

    t_end = time.time()
    print(f"Full scan in {t_end - t_start:.2f}s")

    ahrs_log.close()
    sonar_log.close()

    # Task box detection starts
    angles = [i for i in range(angle_range)]
    height, width = angle_range, num_samples
    cart_height, cart_width = num_samples, num_samples
    cartesian_image = np.zeros((cart_height, cart_width), dtype=np.uint8)

    max_radius = num_samples

    x = np.linspace(-max_radius, max_radius, cart_width)
    y = np.linspace(max_radius, -max_radius, cart_height)
    X, Y = np.meshgrid(x, y)

    # Convert Cartesian coordinates to polar (r, theta)
    r = np.sqrt(X**2 + Y**2)
    theta = np.arctan2(Y, X)
    
    # Normalize theta to match sonar angles
    theta = np.mod(theta, 2 * np.pi)  # Keep angles in [0, 2π]
    theta_index = np.searchsorted(angles, theta)  # Map to row indices
    
    # Normalize radius
    r_index = (r / max_radius * width).astype(int)
    
    # Ensure indices are within bounds
    valid_indices = (r_index < width) & (theta_index < height)
    cartesian_image[valid_indices] = sonar_data[theta_index[valid_indices], r_index[valid_indices]]
    
    # Flip the image vertically to correct orientation
    cartesian_image = np.flipud(cartesian_image)
    
    # Resize Cartesian image to 512x512 for YOLO
    resized_cartesian_image = cv2.resize(cartesian_image, (512, 512), interpolation=cv2.INTER_AREA)
    resized_cartesian_image = cv2.flip(resized_cartesian_image, 0)  # Flip horizontally
    
    # Save output image
    cv2.imwrite(f"record_data/sonar_cart/{timestamp}.png", resized_cartesian_image)
    
    # Show the resized Cartesian image
    plt.ion()
    fig1 = plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.imshow(resized_cartesian_image, cmap='gray')
    plt.title("Resized Cartesian Sonar Image (512x512)")
    plt.axis("off")
    
    # Plot the sonar image in Polar coordinates
    plt.subplot(1, 2, 2, polar=True)
    theta_grid, r_grid = np.meshgrid(angles, np.linspace(0, max_radius, width))
    plt.pcolormesh(theta_grid, r_grid, sonar_data.T, shading='auto', cmap='gray')
    plt.title("Polar Sonar Image")
    plt.colorbar(label='Intensity')
    
    plt.show()
    input("Press Enter to close the plot...") # Wait for user input
    plt.close(fig1)
    # -----------------------------
    # Correction and Mapping
    # -----------------------------

    print("Checking how much the robot rotated during scanning...")

    # Convert yaw list from radians to degrees for easier interpretation
    yaw_deg_list = np.degrees(yaw_rad_list)

    # Unwrap angles to avoid jumps from +180 to -180
    yaw_deg_list_unwrapped = np.unwrap(np.radians(yaw_deg_list)) * 180/np.pi

    # Compute differences between consecutive yaw values
    yaw_deltas = np.diff(yaw_deg_list_unwrapped)

    # Compute average rotation speed per ping
    average_yaw_rate_deg_per_ping = np.mean(np.abs(yaw_deltas))

    # Estimate time per ping (approximate)
    full_scan_time_sec = t_end - t_start
    time_per_ping_sec = full_scan_time_sec / len(yaw_rad_list)

    # Convert to degrees per second
    average_yaw_rate_deg_per_sec = average_yaw_rate_deg_per_ping / time_per_ping_sec

    print(f"Estimated average yaw rotation per ping: {average_yaw_rate_deg_per_ping:.2f} deg/ping")
    print(f"Estimated average yaw rotation speed: {average_yaw_rate_deg_per_sec:.2f} deg/sec")

    input("Press enter to continue...")

    print("Correcting sonar data...")

    raw_points = sonar_scan_without_correction(sonar_data, desired_range_meters, num_samples, angle_start)
    corrected_points_rp = correct_sonar_scan_with_rp(sonar_data, desired_range_meters, num_samples, roll_rad_list, pitch_rad_list, angle_start)
    corrected_points_rpy = correct_sonar_scan_with_rpy(sonar_data, desired_range_meters, num_samples, roll_rad_list, pitch_rad_list, yaw_rad_list, angle_start)

    # Save uncorrected points
    with open(f"record_data/sonar_raw_points/raw_points_{datetime_str}.csv", "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for (x, y) in raw_points:
            writer.writerow([x, y])

    print(f"Saved {len(raw_points)} raw sonar points!")

    # Save corrected points
    with open(f"record_data/sonar_corrected_points/corrected_points_rp_{datetime_str}.csv", "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for (x, y) in corrected_points_rp:
            writer.writerow([x, y])

    print(f"Saved {len(corrected_points_rp)} corrected sonar points with rp!")

    # Save corrected points
    with open(f"record_data/sonar_corrected_points/corrected_points_rpy_{datetime_str}.csv", "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for (x, y) in corrected_points_rpy:
            writer.writerow([x, y])

    print(f"Saved {len(corrected_points_rpy)} corrected sonar points with rpy!")

    # Build Grids
    grid_raw = OccupancyGrid2D(width_m=desired_range_meters*2, height_m=desired_range_meters*2, resolution_m=0.05)
    grid_raw.add_points(raw_points)

    grid_rp = OccupancyGrid2D(width_m=desired_range_meters*2, height_m=desired_range_meters*2, resolution_m=0.05)
    grid_rp.add_points(corrected_points_rp)

    # grid_rpy = OccupancyGrid2D(width_m=int(args.sonar_range)*2, height_m=int(args.sonar_range)*2, resolution_m=0.05)
    # grid_rpy.add_points(corrected_points_rpy)

    # -----------------------------
    # Plot all three maps
    # -----------------------------

    print("Plotting maps...")

    plt.ion()
    fig2, axs = plt.subplots(1, 2, figsize=(10, 5))

    axs[0].imshow(grid_raw.grid, cmap='gray', origin='lower')
    axs[0].set_title('Raw Sonar (No Correction)')
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    axs[0].grid(False)

    axs[1].imshow(grid_rp.grid, cmap='gray', origin='lower')
    axs[1].set_title('Roll-Pitch Corrected')
    axs[1].set_xlabel('X')
    axs[1].set_ylabel('Y')
    axs[1].grid(False)

    # axs[2].imshow(grid_rpy.grid, cmap='gray', origin='lower')
    # axs[2].set_title('Roll-Pitch-Yaw Corrected')
    # axs[2].set_xlabel('X')
    # axs[2].set_ylabel('Y')
    # axs[2].grid(False)

    # # Draw robot heading arrow
    # center_x = grid_rp.width // 2
    # center_y = grid_rp.height // 2

    # # Use last yaw angle recorded
    # # robot_yaw = yaw_rad_list[-1] # in radians

    # # Arrow parameters
    # arrow_length = 20  # in pixels (adjust as needed)

    # # print(f"YAWWWW is {robot_yaw}")

    # # Calculate end of arrow
    # arrow_dx = arrow_length * np.cos(np.pi)
    # arrow_dy = arrow_length * np.sin(np.pi)

    # print(f'{arrow_dx}, {arrow_dy}')

    # # Plot on the last corrected grid (axs[2])
    # # axs[2].arrow(
    # #     center_x, center_y,
    # #     arrow_dx, arrow_dy,
    # #     head_width=5, head_length=10, fc='red', ec='red'
    # # )

    # # Optionally for other plots (you can comment if you want only one)
    # axs[0].arrow(center_x, center_y, arrow_dx, arrow_dy, head_width=5, head_length=10, fc='red', ec='red')
    # axs[1].arrow(center_x, center_y, arrow_dx, arrow_dy, head_width=5, head_length=10, fc='red', ec='red')


    plt.tight_layout()
    plt.savefig(f"record_data/plots/imu_sonar_{datetime_str}.png")
    plt.show()

    input("Press Enter to close the map plot and continue...")
    plt.close(fig2)

    print("Finished full mapping and plotting!")
    cv2.destroyWindow('frame')
# -----------------------------
# Initialize Sensors
# -----------------------------

connection = mavutil.mavlink_connection(args.mavlink)
connection.wait_heartbeat()
boot_time = time.time()
print("Connected to BlueROV2 AHRS2")

request_message_interval(connection, "AHRS2", 10)

ping_sonar = Ping360()
(host, port) = args.udp.split(':')
ping_sonar.connect_udp(host, int(port))
ping_sonar.initialize()
print("Ping360 Sonar Initialized")

video = Video(int(args.camera))

print("Camera Stream Initialized")

os.makedirs("record_data/raw_data_logs", exist_ok=True)
os.makedirs("record_data/sonar_raw_points", exist_ok=True)
os.makedirs("record_data/sonar_corrected_points", exist_ok=True)
os.makedirs("record_data/plots", exist_ok=True)
os.makedirs("record_data/camera", exist_ok=True)
os.makedirs("record_data/sonar_cart/", exist_ok=True)


print("\nReady for commands:")
print("  - Type 'state' to print current states of bluerov")
print("  - Type 'arm' to arm BlueROV")
print("  - Type 'disarm' to disarm BlueROV")
print("  - Type 'wasd' to control BlueROV")
print("  - Type 'manual' to activate manual mode on BlueROV")
print("  - Type 'hold' to activate depth hold mode on BlueROV")
print("  - Type 'scan <start_angle> <end_angle> <range>' to start sonar detection")
print("  - Type 'depth -X' to set a target depth (in meters, negative)")
print("  - Type 'alt' to query altitude info")
print("  - Type 'light <status>' where status is on or off to control light")
print("  - Type 'stop' to imediately halt")
print("  - Type 'options' to print all available control options")
print("  - Type 'quit' to exit\n")

while True:
    cmd = input(">>> ").strip()

    if cmd == "arm":
        arm_brov(connection)
    elif cmd == "disarm":
        disarm_brov(connection)
    elif cmd == "manual":
        manual_mode(connection)
    elif cmd == "hold":
        depth_hold_mode(connection)
    elif cmd.startswith("scan"):
        _, value1, value2, value3 = cmd.split()
        angle_start, angle_end = int(value1), int(value2)
        angle_range = angle_end - angle_start + 1
        num_samples = 1200
        desired_range_meters = int(value3)
        speed_of_sound = 1500
        sample_period = int((2 * desired_range_meters) / (num_samples * speed_of_sound * 25e-9))
        ping_sonar.set_sample_period(sample_period)
        ping_sonar.set_number_of_samples(num_samples)

        print(f"Sonar params: angle_start {angle_start}, angle_end {angle_end}, range {desired_range_meters} m")

        sonar_detection(connection, video, ping_sonar, angle_start, angle_end, angle_range, num_samples, desired_range_meters)
    elif cmd.startswith("depth"):
        try:
            _, value = cmd.split()
            depth_val = float(value)
            print(f"Setting target depth to {depth_val} meters...")
            set_target_depth(connection, depth_val, boot_time)
        except Exception as e:
            print(f"Invalid depth command. Use: depth -X\nError: {e}")
    elif cmd == "alt":
        get_altitude(connection, "AHRS2")
    elif cmd == "wasd":
        cv2_keyboard_control(connection, video)
    elif cmd == "quit":
        print("Exiting.")
        disarm_brov(connection)
        break
    elif cmd.startswith("light"):
        _, stat = cmd.split()
        if stat == "on":
            set_servo_pwm(connection, 1, 1900)
        else:
            set_servo_pwm(connection, 1, 1100)
    elif cmd == "stop":
        stop(connection)
    elif cmd == "options":
        print("\nReady for commands:")
        print("  - Type 'arm' to arm BlueROV")
        print("  - Type 'disarm' to disarm BlueROV")
        print("  - Type 'wasd' to control BlueROV")
        print("  - Type 'manual' to activate manual mode on BlueROV")
        print("  - Type 'hold' to activate depth hold mode on BlueROV")
        print("  - Type 'scan <start_angle> <end_angle> <range>' to start sonar detection")
        print("  - Type 'depth -X' to set a target depth (in meters, negative)")
        print("  - Type 'alt' to query altitude info")
        print("  - Type 'light <status>' where status is on or off to control light")
        print("  - Type 'stop' to imediately halt")
        print("  - Type 'quit' to exit\n")
    elif cmd == "state":
        is_armed = query_current_state(connection)
        state = "armed" if is_armed else "disarmed"
        print(f"BlueROV is currently {state}")
    else:
        print("Unknown command. Try 'scan', 'depth -X', or 'quit'.")
