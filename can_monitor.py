# LEARNING GUIDE: inspect raw traffic on the Linux can0 interface.
# This script listens and prints frames; it does not enable or command motors.
# The interface must already exist and be configured. This is a top-level loop.
# python-can supplies the Bus object and CAN Message objects.
import can

# Open the existing SocketCAN interface; this code does not configure bitrate.
bus = can.interface.Bus(
    channel="can0",
    interface="socketcan"
)

# ID is the arbitration identifier in hexadecimal; EXT indicates the extended
# identifier format; DATA is the payload as hexadecimal bytes. This output
# does not convert the encoded fields into angles, velocities, or torques.
print("Listening on CAN...")

# Keep listening until the process is interrupted.
while True:
    # With no timeout supplied, recv() blocks until a frame is available.
    msg = bus.recv()

    # Print only when a Message object was returned.
    if msg:
        # ID is the arbitration identifier in hexadecimal; EXT indicates the extended
        # identifier format; DATA is the payload as hexadecimal bytes. This output
        # does not convert the encoded fields into angles, velocities, or torques.
        print(
            f"ID: {hex(msg.arbitration_id)} "
            f"EXT:{msg.is_extended_id} "
            f"DATA:{msg.data.hex()}"
        )