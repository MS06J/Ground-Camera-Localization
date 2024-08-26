import time
from pymavlink import mavutil

connection = mavutil.mavlink_connection('udp:192.168.1.142:14550')
print('waiting for heartbeat...')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))