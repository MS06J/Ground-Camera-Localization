To run Ground Control Station (GCS) paraelle with pymavlink script, use mavproxy as router.
`mavproxy.py --master=udp:your-GCS-ip:14650 --baudrate 115200 --out udp:your-GCS-ip:14551 --out udp:your-GCS-ip:14550
