import time
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

class MavlinkSender(): 
    def __init__(self, ip='udp:127.0.0.1:14551'):
        self.ip = ip

    def connect(self):
        # Connect to the vehicle
        connection = mavutil.mavlink_connection(self.ip)
        # Wait for the first heartbeat, so we know the system IDs
        connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
        self.connection = connection

    def set_home(self, home_lat=47.397742 * 1e7, home_lon=8.545594 * 1e7, home_alt=488.0):
        # Set home position relative to current position (0 for global, 1 for current)
        use_current_position = 0

        # Send the MAV_CMD_DO_SET_HOME command
        connection = self.connection
        connection.mav.command_long_send(
            connection.target_system,    # Target system (0 for broadcast)
            connection.target_component, # Target component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # Command
            0,  # Confirmation (0: First transmission)
            use_current_position,  # Param 1 (1 to use current position, 0 to use specified position)
            home_lat,  # Param 2 (Latitude)
            home_lon,  # Param 3 (Longitude)
            home_alt,  # Param 4 (Altitude)
            0, 0, 0    # Param 5, 6, 7 (unused)
        )

        # Confirm the command was received
        ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(ack_msg)
        time.sleep(0.1)

    def set_ekf_home(self, home_lat=47.397742 * 1e7, home_lon=8.545594 * 1e7, home_alt=488.0):
        # Send the SET_GPS_GLOBAL_ORIGIN message
        connection = self.connection
        connection.mav.set_gps_global_origin_send(
            connection.target_system,  # Target system (0 for broadcast)
            int(home_lat),             # Latitude in int32 (degrees * 1e7)
            int(home_lon),             # Longitude in int32 (degrees * 1e7)
            int(home_alt * 1000)       # Altitude in int32 (millimeters)
        )

        # Confirm the command was received (OPTIONAL) only support be some FCU
        ack_msg = connection.recv_match(type='STATUSTEXT', blocking=True)
        print(ack_msg)

    def send_pose(self, poses): #only support single tag in a frame
        connection = self.connection
        pose = poses[0]
        # Define the vision position estimate data
        usec = int(time.time() * 1e6)  # Current time in microseconds

        # Quaternion components representing the vehicle's attitude
        quat = pose.orientation
        q = QuaternionBase([quat.x, quat.y, quat.z, quat.w])

        x = pose.position.x  # X position in meters (forward)
        y = pose.position.y  # Y position in meters (right)
        z = pose.position.z # Z position in meters (down)

        covariance = [0] * 21  # Covariance matrix (not used here)

        # Send the ATT_POS_MOCAP message
        connection.mav.att_pos_mocap_send(
            usec,      # Time since system boot in microseconds
            q,         # Quaternion components [w, x, y, z]
            x,         # X position in meters
            y,         # Y position in meters
            z,         # Z position in meters
            covariance # Pose covariance matrix (optional)
        )

        # Print a status message
        print(f"Sent ATT_POS_MOCAP: x={x}, y={y}, z={z}, q={q}")

    def send_pose_dummy(self):
        connection = self.connection
        # Define the vision position estimate data
        usec = int(time.time() * 1e6)  # Current time in microseconds

        # Quaternion components representing the vehicle's attitude
        q = QuaternionBase([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z] where w is the scalar component

        x = 1.0  # X position in meters (forward)
        y = 2.0  # Y position in meters (right)
        z = -1.0 # Z position in meters (down)

        covariance = [0] * 21  # Covariance matrix (not used here)

        # Send the VISION_POSITION_ESTIMATE message
        connection.mav.att_pos_mocap_send(
            usec,      # Time since system boot in microseconds
            q,         # Quaternion components [w, x, y, z]
            x,         # X position in meters
            y,         # Y position in meters
            z,         # Z position in meters
            covariance # Pose covariance matrix (optional)
        )

        # Print a status message
        print(f"Sent ATT_POS_MOCAP: x={x}, y={y}, z={z}, q={q}")

    def close(self):
        self.connection.close()

if __name__ == '__main__':
    sender = MavlinkSender()
    time.sleep(0.1)
    sender.set_home()
    sender.set_ekf_home()
    time.sleep(0.5)

    try:
        while(True):
            sender.send_pose_dummy()
    except KeyboardInterrupt:
        # Graceful exit on Ctrl+C
        print("Exiting...")
    finally:
        sender.close()