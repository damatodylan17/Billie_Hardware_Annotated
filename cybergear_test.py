# LEARNING GUIDE: send probe frames and inspect the replies on can0.
# The commented block at the start is an older experiment and does not execute.
# The active code probes addresses 1 through 4, without commanding joint motion.
# This is a diagnostic script, not a unit test or confirmed motor discovery tool.
# import can
# import time

# # figures out what the motor ID is

# bus = can.interface.Bus(
#     channel="can0",
#     interface="socketcan"
# )

# print("Scanning...")

# for motor_id in range(1, 128):

#     # MIT read parameter command
#     can_id = (0x11 << 24) | (0 << 8) | motor_id

#     msg = can.Message(
#         arbitration_id=can_id,
#         is_extended_id=True,
#         data=[
#             0x20, 0x10,
#             0,0,0,0,0,0
#         ]
#     )

#     try:
#         bus.send(msg)
#     except:
#         pass

#     rx = bus.recv(timeout=0.01)

#     if rx:
#         print(
#             "FOUND:",
#             hex(rx.arbitration_id),
#             rx.data.hex()
#         )

# print("Done")

# python-can opens SocketCAN; time controls each response collection window.
import can
import time

# Use an already-configured Linux can0 interface; no driver setup occurs here.
bus = can.interface.Bus(
    channel="can0",
    interface="socketcan"
)

# DLC is the payload length; Data is a raw hex dump, not decoded motor state.
print("=== CyberGear CAN Scanner ===")

# Flush any old frames
# Drain queued frames until one 0.01-second receive interval returns nothing.
# With continuous incoming traffic, this loop may keep waiting for a quiet gap.
while True:
    msg = bus.recv(timeout=0.01)
    # None means no frame arrived before the timeout, so the flush is complete.
    if msg is None:
        break

# DLC is the payload length; Data is a raw hex dump, not decoded motor state.
print("Bus cleared.\n")

# range excludes its upper bound: this tries motor IDs 1, 2, 3, and 4.
for motor_id in range(1, 5):

    # Bit shifts place the read-parameter command (0x11) in the high field,
    # master ID 0 in the middle, and the target motor address in the low byte.
    can_id = (0x11 << 24) | (0 << 8) | motor_id

    # Build an extended CAN frame. The first two data bytes encode parameter
    # index 0x1020, which is not in cybergear_driver.py's VALID_PARAMS table.
    # The probe is preserved as written; a response is not guaranteed.
    tx = can.Message(
        arbitration_id=can_id,
        is_extended_id=True,
        data=[
            0x20, 0x10,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ]
    )

    # DLC is the payload length; Data is a raw hex dump, not decoded motor state.
    print(f"Trying motor ID {motor_id:3d}...", end=" ")

    try:
        # Transmit the probe. Success here only means sending did not raise an error;
        # it does not establish that a specific motor received or accepted it.
        bus.send(tx)
        # DLC is the payload length; Data is a raw hex dump, not decoded motor state.
        print("TX OK", end="")
    # Print the send error and move to the next ID if transmission fails.
    except Exception as e:
        # DLC is the payload length; Data is a raw hex dump, not decoded motor state.
        print(f"TX FAILED: {e}")
        continue

    # Track whether any frame is received during this probe window.
    found = False
    start = time.time()

    # Collect frames for roughly 100 ms, with receive calls waiting at most 10 ms.
    while time.time() - start < 0.1:
        rx = bus.recv(timeout=0.01)

        if rx is None:
            continue

        # Existing behavior: any received frame counts here. The code does not match
        # its sender or command type against the current probe.
        found = True

        # DLC is the payload length; Data is a raw hex dump, not decoded motor state.
        print(
            f"\n    RX:"
            f"\n      ID       = {hex(rx.arbitration_id)}"
            f"\n      Extended= {rx.is_extended_id}"
            f"\n      DLC      = {rx.dlc}"
            f"\n      Data     = {rx.data.hex()}"
        )

    # Report a quiet response window; this alone does not prove that the ID is absent.
    if not found:
        # DLC is the payload length; Data is a raw hex dump, not decoded motor state.
        print("  --> No response")

# DLC is the payload length; Data is a raw hex dump, not decoded motor state.
print("\nFinished.")