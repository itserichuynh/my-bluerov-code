import time
import numpy as np
import csv
from pymavlink import mavutil
import os
import math
from bluerov_ping.brping import Ping360
import matplotlib.pyplot as plt

from datetime import datetime

# Add a timestamp string for this scan
datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")


# -----------------------------
# Helper Functions
# -----------------------------

def correct_sonar_scan_with_rpy(sonar_data, desired_range_meters, num_samples, roll_deg, pitch_deg):
    corrected_points = []
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)

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

    num_angles = sonar_data.shape[0]
    angles_rad = np.arange(num_angles) * (2 * np.pi / 400)
    radii = np.linspace(0, desired_range_meters, num_samples)

    for angle_idx, angle_rad in enumerate(angles_rad):
        for sample_idx, radius in enumerate(radii):
            intensity = sonar_data[angle_idx, sample_idx]
            if intensity > 20:
                x = radius * np.cos(angle_rad)
                y = radius * np.sin(angle_rad)
                z = 0.0
                rotated_point = np.dot(R, np.array([x, y, z]))
                corrected_points.append((rotated_point[0], rotated_point[1]))

    return corrected_points

def sonar_scan_without_correction(sonar_data, desired_range_meters, num_samples):
    raw_points = []
    num_angles = sonar_data.shape[0]
    angles_rad = np.arange(num_angles) * (2 * np.pi / 400)
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

    def plot(self, title="Occupancy Grid Map"):
        plt.figure(figsize=(8, 8))
        plt.imshow(self.grid, cmap='gray', origin='lower')
        plt.title(title)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(False)
        plt.show()

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

request_message_interval("AHRS2", 13)


# -----------------------------
# Initialize Sensors
# -----------------------------

# MAVLink Connection for AHRS2
connection = mavutil.mavlink_connection("udp:192.168.2.1:14551")
connection.wait_heartbeat()
print("Connected to BlueROV2 AHRS2")

# Sonar Connection
p = Ping360()
p.connect_udp("192.168.2.2", 9092)
p.initialize()
print("Ping360 Sonar Initialized")

# Set Sonar Parameters
num_angles = 400
num_samples = 200
desired_range_meters = 2
speed_of_sound = 1500
sample_period = int((2 * desired_range_meters) / (num_samples * speed_of_sound * 25e-9))
p.set_sample_period(sample_period)
p.set_number_of_samples(num_samples)

# Create folder to save results
os.makedirs("sonar_corrected_points", exist_ok=True)
os.makedirs("sonar_uncorrected_points", exist_ok=True)
os.makedirs("raw_data_logs", exist_ok=True)

# Open CSV files for logging
ahrs_log = open(f"raw_data_logs/ahrs_log_{datetime_str}.csv", "w", newline='')
sonar_log = open(f"raw_data_logs/sonar_log_{datetime_str}.csv", "w", newline='')
ahrs_writer = csv.writer(ahrs_log)
sonar_writer = csv.writer(sonar_log)

ahrs_writer.writerow(["timestamp", "roll", "pitch", "yaw", "altitude", "lat", "lng"])
sonar_writer.writerow(["timestamp", "angle"] + [f"sample_{i}" for i in range(num_samples)])

# -----------------------------
# Data Collection Loop
# -----------------------------

sonar_data = np.zeros((num_angles, num_samples), dtype=np.uint8)

print("Starting synchronized data collection...")

t_start = time.time()
for angle in range(num_angles):
    timestamp = time.time()

    # 1. Sonar ping
    response = p.transmitAngle(angle)

    # 2. AHRS2 data fetch
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

    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)

    # Log AHRS2 data
    ahrs_writer.writerow([timestamp, roll_rad, pitch_rad, yaw_rad, altitude, lat, lng])

    # 3. Save sonar data for this angle
    if response:
        data = np.frombuffer(response.data, dtype=np.uint8)
        sonar_data[angle] = data
        sonar_writer.writerow([timestamp, angle] + data.tolist())
        print(f"[Angle {angle}] Collected sonar + AHRS2")
    else:
        print(f"[Angle {angle}] Sonar data missing")
t_end = time.time()
print(f"Full scan in {t_end - t_start}s, {400/(t_end - t_start)}Hz")

# Close log files
ahrs_log.close()
sonar_log.close()

# After full scan (0-400 angles), correct and save
print("Correcting full sonar scan...")

corrected_points = correct_sonar_scan_with_rpy(
    sonar_data, desired_range_meters, num_samples, roll_deg, pitch_deg
)
raw_points = sonar_scan_without_correction(
    sonar_data, desired_range_meters, num_samples
)

# Save corrected points
with open(f"sonar_corrected_points/corrected_points_{datetime_str}.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    for (x, y) in corrected_points:
        writer.writerow([x, y])

print(f"Saved {len(corrected_points)} corrected sonar points!")

# Save uncorrected points
with open(f"sonar_uncorrected_points/uncorrected_points_{datetime_str}.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    for (x, y) in raw_points:
        writer.writerow([x, y])

print(f"Saved {len(corrected_points)} corrected sonar points!")

# Build Occupancy Grids
print("Building occupancy grid maps...")

grid_corrected = OccupancyGrid2D(width_m=10, height_m=10, resolution_m=0.05)
grid_corrected.add_points(corrected_points)

grid_raw = OccupancyGrid2D(width_m=10, height_m=10, resolution_m=0.05)
grid_raw.add_points(raw_points)

# Plot both grids
# grid_corrected.plot(title="Corrected Sonar Occupancy Grid")
# grid_raw.plot(title="Raw Sonar Occupancy Grid (No Tilt Correction)")

# # Plot both grids in the same window
# print("Plotting both corrected and raw sonar maps together...")

fig, axs = plt.subplots(1, 2, figsize=(16, 8))

axs[0].imshow(grid_corrected.grid, cmap='gray', origin='lower')
axs[0].set_title('Corrected Sonar Occupancy Grid')
axs[0].set_xlabel('X')
axs[0].set_ylabel('Y')
axs[0].grid(False)

axs[1].imshow(grid_raw.grid, cmap='gray', origin='lower')
axs[1].set_title('Raw Sonar Occupancy Grid (No Tilt Correction)')
axs[1].set_xlabel('X')
axs[1].set_ylabel('Y')
axs[1].grid(False)

plt.tight_layout()
plt.show()

# -----------------------------
# Done
# -----------------------------
print("Finished data collection, correction, and mapping!")


# -----------------------------
# Done
# -----------------------------
print("Finished data collection, correction, and mapping!")
