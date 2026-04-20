import time
from ufoc_client import uFOCclient

client = uFOCclient(
    channel="/dev/tty.usbmodem206F327555481",
    arbitration_id=0x123,
)

client.set_control_state(3)
time.sleep(0.01)
client.set_angular_velocity_target(0)
time.sleep(0.01)

delay = 0.005

def simulate_ridge(current_pos ,center, width, force):
    if current_pos > center - width and current_pos < center + width:
        client.set_torque_current_soft_limit(force) 
        time.sleep(delay)
    else:
        client.set_torque_current_soft_limit(0) 

while True:
    pos = client.get_position()
    time.sleep(delay)

    # simulate_ridge(pos%1, 0.0, 0.05, 0.5)
    simulate_ridge(pos%1, 0.2, 0.05, 0.5)
    # simulate_ridge(pos%1, 0.4, 0.05, 0.5)
    simulate_ridge(pos%1, 0.6, 0.05, 0.5)
    # simulate_ridge(pos%1, 0.8, 0.05, 0.5)
    print(pos%1)
    time.sleep(delay)

    client.set_position_target(pos)
    time.sleep(delay)

    # target_torue = abs(pos/10)
    # print(pos)
    # time.sleep(0.01)
    # client.set_torque_current_soft_limit(target_torue)
    # time.sleep(0.01)
