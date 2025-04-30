from pymavlink import mavutil
import time
import csv
import argparse
import os

parser = argparse.ArgumentParser(description="Collecting imu data.")
parser.add_argument('--file', action="store", required=False, type=str, help="Save to file. E.g: imu_log.csv")
args = parser.parse_args()
if args.file is None:
    parser.print_help()
    exit(1)

file_name = args.file

os.makedirs("roll_pitch_yaw_logs", exist_ok=True)

# Connect to BlueROV2 via MAVLink (UDP)
connection = mavutil.mavlink_connection("udp:192.168.2.1:14551")

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to BlueROV2")

# Open CSV files for logging
rpy_log = open("roll_pitch_yaw_logs/" + file_name, "w", newline='')
rpy_writer = csv.writer(rpy_log)
rpy_writer.writerow(["timestamp", "roll", "pitch", "yaw", "altitude", "lat", "lng"])

def request_message_interval(message, frequency_hz: float):
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

    print("Requested the message successfully.")

   

def get_ahrs2_data(connection, message_name: str):
    try:
        dict = connection.recv_match(type= message_name, blocking=True, timeout=0.1).to_dict()
        
        roll = dict['roll']
        pitch = dict['pitch']
        yaw = dict['yaw']
        altitude = dict['altitude']
        lat = dict['lat']
        lng = dict['lng']

        print(f'roll: {roll}, pitch: {pitch}, yaw: {yaw}')

        return roll, pitch, yaw, altitude, lat, lng
    except:
        pass

def get_alt_data(connection, message_name: str):
    try:
        dict = connection.recv_match(type= message_name, blocking=True, timeout=0.1).to_dict()
        
        alt = dict['alt']

        print(f'altitude: {alt}')

        return alt
    except:
        pass

request_message_interval("VFR_HUD", 1)
save_name = 'test.csv'

while True:
    try:
        # roll, pitch, yaw, altitude, lat, lng = get_ahrs2_data(connection, "AHRS2")
        # ts = time.time()
        # rpy_writer.writerow([ts, roll, pitch, yaw, altitude, lat, lng])
        alt = get_alt_data(connection, "VFR_HUD")
    except:
        pass
    time.sleep(0.1)