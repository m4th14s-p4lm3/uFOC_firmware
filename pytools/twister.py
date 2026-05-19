import time
from ufoc_client import uFOCclient

client_slave = uFOCclient(
    channel="/dev/tty.usbmodem206F327555481",
    arbitration_id=0x123,
    device_id = 0x0
)

client_slave.set_control_state(2)
time.sleep(0.0001)
client_slave.set_angular_velocity_target(0)


while True:
    client_slave_pos = client_slave.get_position()
    print(client_slave_pos)
    time.sleep(0.001)
    client_slave.set_torque_current_soft_limit(abs(client_slave_pos/4))
    time.sleep(0.001)
