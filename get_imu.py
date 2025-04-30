from pymavlink import mavutil
import time
import csv
import argparse

parser = argparse.ArgumentParser(description="Collecting imu data.")
parser.add_argument('--file', action="store", required=False, type=str, help="Save to file. E.g: imu_log.csv")
args = parser.parse_args()
if args.file is None:
    parser.print_help()
    exit(1)

# Connect to BlueROV2 via MAVLink (UDP)
connection = mavutil.mavlink_connection("udp:192.168.2.1:14551")

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to BlueROV2")

# Request raw and scaled IMU data streams
connection.mav.request_data_stream_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # Data stream type
    500,  # Update rate (Hz)
    1    # Start streaming
)
print("Requested RAW_SENSORS data stream")

file_name = args.file
# Open CSV file to log both IMU types
with open(file_name, "w", newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow([
        "timestamp", "imu_type",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z",
        "mag_x", "mag_y", "mag_z"
    ])

    # Collect for a fixed time, e.g., 60 seconds
    time_start = time.time()
    duration = 20  # seconds
    while time.time() - time_start < duration:
        msg = connection.recv_match(type=['RAW_IMU', 'SCALED_IMU2'], blocking=True)
        if msg:
            ts = time.time()
            if msg.get_type() == "RAW_IMU":
                writer.writerow([
                    ts, "RAW_IMU",
                    msg.xacc, msg.yacc, msg.zacc,
                    msg.xgyro, msg.ygyro, msg.zgyro,
                    msg.xmag, msg.ymag, msg.zmag
                ])
                print(f"RAW_IMU - Time: {msg.time_usec}, Accel: ({msg.xacc}, {msg.yacc}, {msg.zacc}), Gyro: ({msg.xgyro}, {msg.ygyro}, {msg.zgyro}), Mag: ({msg.xmag}, {msg.ymag}, {msg.zmag})")
            elif msg.get_type() == "SCALED_IMU2":
                writer.writerow([
                    ts, "SCALED_IMU2",
                    msg.xacc, msg.yacc, msg.zacc,
                    msg.xgyro, msg.ygyro, msg.zgyro,
                    msg.xmag, msg.ymag, msg.zmag
                ])
                print(f"SCALED_IMU2 - Time: {msg.time_boot_ms}, Accel: ({msg.xacc}, {msg.yacc}, {msg.zacc}), Gyro: ({msg.xgyro}, {msg.ygyro}, {msg.zgyro}), Mag: ({msg.xmag}, {msg.ymag}, {msg.zmag})")
