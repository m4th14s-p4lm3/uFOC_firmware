from ufoc_client import uFOCclient
import time


if __name__ == "__main__":
    client = uFOCclient(
        channel="/dev/tty.usbmodem206F327555481",
        arbitration_id=0x123,
    )

    # client.set_electrical_offset(647086.0)
    # time.sleep(0.05)

    client.set_control_state(1)
    time.sleep(0.05)
    client.set_torque_current_target(0.7)
    time.sleep(3)
    client.set_torque_current_target(-0.7)
    time.sleep(3)
    client.set_torque_current_target(0)
    time.sleep(0.05)


    # client.set_control_state(3)

    # time.sleep(0.05)

    # client.set_position_target(15)
    # time.sleep(2)
    # client.set_position_target(-15)
    # time.sleep(1)

    # print(client.get_electrical_offset())
