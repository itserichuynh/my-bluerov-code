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

parser = argparse.ArgumentParser(description="Collecting imu, camera and sonar. RPY correction on sonar.")
parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 192.168.2.2:9092")
parser.add_argument('--mavlink', action="store", required=False, type=str, help="Mavlink UDP. E.g: udp:192.168.2.1:14551")
parser.add_argument('--camera', action="store", required=False, type=str, help="Camera UDP port. E.g: 5601")
parser.add_argument('--sonar_range', action="store", required=False, type=str, help="Sonar range in meters. E.g: 10")


args = parser.parse_args()
if args.udp is None or args.mavlink is None or args.camera is None or args.sonar_range is None:
    parser.print_help()
    exit(1)

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Add a timestamp string for this scan
datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")


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

# -----------------------------
# Initialize Sensors
# -----------------------------

connection = mavutil.mavlink_connection(args.mavlink)
connection.wait_heartbeat()
print("Connected to BlueROV2 AHRS2")

request_message_interval("AHRS2", 13)

p = Ping360()
(host, port) = args.udp.split(':')
p.connect_udp(host, int(port))
p.initialize()
print("Ping360 Sonar Initialized")

video = Video(int(args.camera))

print("Camera Stream Initialized")

# Set Sonar Parameters
# num_angles = 400
angle_start = 140
angle_end = 260
angle_range = angle_end - angle_start + 1
num_samples = 200
desired_range_meters = int(args.sonar_range)
speed_of_sound = 1500
sample_period = int((2 * desired_range_meters) / (num_samples * speed_of_sound * 25e-9))
p.set_sample_period(sample_period)
p.set_number_of_samples(num_samples)

os.makedirs("raw_data_logs", exist_ok=True)
os.makedirs("sonar_raw_points", exist_ok=True)
os.makedirs("sonar_corrected_points", exist_ok=True)
os.makedirs("plots", exist_ok=True)
os.makedirs("camera", exist_ok=True)

# Open CSV files
ahrs_log = open(f"raw_data_logs/ahrs_log_{datetime_str}.csv", "w", newline='')
sonar_log = open(f"raw_data_logs/sonar_log_{datetime_str}.csv", "w", newline='')
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
    response = p.transmitAngle(angle)

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
        sonar_data[angle] = data
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
        cv2.imwrite(f"camera/{timestamp}.png", frame)
        print("Camera frame successfully captured.")
    else:
        print("Frame not available :(")

t_end = time.time()
print(f"Full scan in {t_end - t_start:.2f}s")

ahrs_log.close()
sonar_log.close()

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

raw_points = sonar_scan_without_correction(sonar_data, desired_range_meters, num_samples)
corrected_points_rp = correct_sonar_scan_with_rp(sonar_data, desired_range_meters, num_samples, roll_rad_list, pitch_rad_list)
corrected_points_rpy = correct_sonar_scan_with_rpy(sonar_data, desired_range_meters, num_samples, roll_rad_list, pitch_rad_list, yaw_rad_list)

# Save uncorrected points
with open(f"sonar_raw_points/raw_points_{datetime_str}.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    for (x, y) in raw_points:
        writer.writerow([x, y])

print(f"Saved {len(raw_points)} raw sonar points!")

# Save corrected points
with open(f"sonar_corrected_points/corrected_points_rp_{datetime_str}.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    for (x, y) in corrected_points_rp:
        writer.writerow([x, y])

print(f"Saved {len(corrected_points_rp)} corrected sonar points with rp!")

# Save corrected points
with open(f"sonar_corrected_points/corrected_points_rpy_{datetime_str}.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    for (x, y) in corrected_points_rpy:
        writer.writerow([x, y])

print(f"Saved {len(corrected_points_rpy)} corrected sonar points with rpy!")

# Build Grids
grid_raw = OccupancyGrid2D(width_m=int(args.sonar_range)*2, height_m=int(args.sonar_range)*2, resolution_m=0.05)
grid_raw.add_points(raw_points)

grid_rp = OccupancyGrid2D(width_m=int(args.sonar_range)*2, height_m=int(args.sonar_range)*2, resolution_m=0.05)
grid_rp.add_points(corrected_points_rp)

grid_rpy = OccupancyGrid2D(width_m=int(args.sonar_range)*2, height_m=int(args.sonar_range)*2, resolution_m=0.05)
grid_rpy.add_points(corrected_points_rpy)

# -----------------------------
# Plot all three maps
# -----------------------------

print("Plotting maps...")

fig, axs = plt.subplots(1, 3, figsize=(24, 8))

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

axs[2].imshow(grid_rpy.grid, cmap='gray', origin='lower')
axs[2].set_title('Roll-Pitch-Yaw Corrected')
axs[2].set_xlabel('X')
axs[2].set_ylabel('Y')
axs[2].grid(False)

# Draw robot heading arrow
center_x = grid_rpy.width // 2
center_y = grid_rpy.height // 2

# Use last yaw angle recorded
robot_yaw = np.pi - yaw_rad_list[-1] # in radians

# Arrow parameters
arrow_length = 20  # in pixels (adjust as needed)

print(f"YAWWWW is {robot_yaw}")

# Calculate end of arrow
arrow_dx = arrow_length * np.cos(robot_yaw)
arrow_dy = arrow_length * np.sin(robot_yaw)

print(f'{arrow_dx}, {arrow_dy}')

# Plot on the last corrected grid (axs[2])
axs[2].arrow(
    center_x, center_y,
    arrow_dx, arrow_dy,
    head_width=5, head_length=10, fc='red', ec='red'
)

# Optionally for other plots (you can comment if you want only one)
axs[0].arrow(center_x, center_y, arrow_dx, arrow_dy, head_width=5, head_length=10, fc='red', ec='red')
axs[1].arrow(center_x, center_y, arrow_dx, arrow_dy, head_width=5, head_length=10, fc='red', ec='red')


plt.tight_layout()
plt.savefig(f"plots/imu_sonar_{datetime_str}.png")
plt.show()

print("Finished full mapping and plotting!")
