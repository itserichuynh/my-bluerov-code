# import Ping360 class
from bluerov_ping.brping import Ping360
import argparse
import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import time

# Create Ping360 instance
p = Ping360()

... # Connect to, initialize, and set up Ping360 settings

parser = argparse.ArgumentParser(description="Collecting Ping360 sonar data.")
parser.add_argument('--device', action="store", required=False, type=str, help="Ping device port. E.g: /dev/ttyUSB0")
parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 192.168.2.2:9092")
parser.add_argument('--file', action="store", required=False, type=str, help="Save to file. E.g: sonar_log.csv")

args = parser.parse_args()
if args.device is None and args.udp is None:
    parser.print_help()
    exit(1)

p = Ping360()
if args.device is not None:
    p.connect_serial(args.device, args.baudrate)
elif args.udp is not None:
    (host, port) = args.udp.split(':')
    p.connect_udp(host, int(port))

print("Initialized: %s" % p.initialize())

num_angles = 400
num_samples = 200
sonar_data = np.zeros((num_angles, num_samples), dtype=np.uint8)

desired_range_meters = 2 # range in meters
speed_of_sound = 1500  # in water

# Rearranged formula to find sample_period:
sample_period = int((2 * desired_range_meters) / (num_samples * speed_of_sound * 25e-9))

p.set_sample_period(sample_period)
p.set_number_of_samples(num_samples)

file_name = args.file
with open(file_name, "w", newline='') as f:
    writer = csv.writer(f)
    # Create header: ["timestamp", "angle", "sample_0", ..., "sample_199"]
    header = ["timestamp", "angle"] + [f"sample_{i}" for i in range(num_samples)]
    writer.writerow(header)

    # Loop through a full circle, one gradian at a time
    t_start = time.time()
    for angle in range(num_angles):
        response = p.transmitAngle(angle)
        ts = time.time()
        # print(response)
        if response:
            data = np.frombuffer(response.data, dtype=np.uint8)
            sonar_data[angle] = data
            print(f"Collected data at angle {angle}")
        else:
            print(f"Missing data at angle {angle}")
            sonar_data[angle] = np.zeros(num_samples, dtype=np.uint8)
        row = [ts, angle] + data.tolist()
        writer.writerow(row)
    t_end = time.time()
print(f"Full scan in {t_end - t_start}s, {400/(t_end - t_start)}Hz")

# Polar plot
angles_rad = np.arange(num_angles) * (math.pi / 200)
radii = np.linspace(0, desired_range_meters, num_samples)
theta, r = np.meshgrid(angles_rad, radii, indexing='ij')

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
c = ax.pcolormesh(theta, r, sonar_data / 255.0, shading='auto', cmap='viridis')
ax.set_title("Ping360 Sonar Scan")
fig.colorbar(c, ax=ax, label='Echo Intensity')
plt.show()