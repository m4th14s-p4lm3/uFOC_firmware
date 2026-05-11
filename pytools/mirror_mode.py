import time
from ufoc_client import uFOCclient

client_master = uFOCclient(
    channel="/dev/tty.usbmodem206F327555481",
    arbitration_id=0x123,
    device_id = 0x1
)

client_slave = uFOCclient(
    channel="/dev/tty.usbmodem206F327555481",
    arbitration_id=0x123,
    device_id = 0x0
)

client_slave.set_control_state(3)
client_master.set_control_state(0)


while True:
    master_pos = client_master.get_position()
    print(master_pos)
    time.sleep(0.0001)
    client_slave.set_position_target(round(master_pos, 3))
    time.sleep(0.0001)
    
    # print(client_master.get_position())
    # print(client_slave.get_position())
