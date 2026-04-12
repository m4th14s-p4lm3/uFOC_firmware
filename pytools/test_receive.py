import can
import time

bus = can.Bus(
    interface="slcan",
    # channel="/dev/cu.usbmodem206F327555481",
    channel="/dev/tty.usbmodem206F327555481",
    bitrate=500000,
)

print("opened")
time.sleep(0.5)

last_msg_time = time.perf_counter()

while True:
    msg = bus.recv(timeout=1.0)
    if msg is None:
        print("timeout")
    else:
        print(
            f"ID=0x{msg.arbitration_id:X} EXT={msg.is_extended_id} "
            f"DLC={msg.dlc} DATA={msg.data.hex(' ')}"
        )
        # new_time = time.perf_counter()

        # print(1/(new_time-last_msg_time ))
        # last_msg_time = new_time