import can
import struct
from typing import Optional


class MotorClient:
    # Device
    CAN_COMMUNICATION_DEVICE_ID = 0x00

    # General
    SET_CONTROL_STATE = 0x00

    # Current control
    SET_TORQUE_CURRENT_TARGET = 0x06
    SET_TORQUE_CURRENT_SOFT_LIMIT = 0x07
    GET_TORQUE_CURRENT_SOFT_LIMIT = 0x08
    SET_CURRENT_KP = 0x09
    SET_CURRENT_KI = 0x0A

    # Velocity control
    SET_ANGULAR_VELOCITY_TARGET = 0x10
    SET_ANGULAR_VELOCITY_SOFT_LIMIT = 0x11
    GET_ANGULAR_VELOCITY_SOFT_LIMIT = 0x12
    GET_ANGULAR_VELOCITY = 0x13
    SET_ANGULAR_VELOCITY_KP = 0x14
    SET_ANGULAR_VELOCITY_KI = 0x15

    # Position control
    SET_POSITION_TARGET = 0x1A
    GET_POSITION_TARGET = 0x1B
    GET_POSITION = 0x1C
    GET_RELATIVE_POSITION = 0x1D
    SET_POSITION_KP = 0x1E
    SET_POSITION_KI = 0x1F
    SET_POSITION_KD = 0x20

    # Encoder
    GET_ELECTRICAL_OFFSET = 0x26
    SET_ELECTRICAL_OFFSET = 0x27
    CALIBRATE_ELECTRICAL_OFFSET = 0x28

    def __init__(
        self,
        channel: str,
        arbitration_id: int = 0x123,
        bitrate: int = 500000,
        interface: str = "slcan",
        device_id: int = 0x00,
    ):
        self.arbitration_id = arbitration_id
        self.device_id = device_id

        self.bus = can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
        )

    @staticmethod
    def _float_to_bytes(value: float) -> list[int]:
        return list(struct.pack("<f", value))  # little-endian float

    def _send(self, command: int, payload: Optional[list[int]] = None) -> None:
        if payload is None:
            payload = []

        data = [self.device_id, command] + payload

        msg = can.Message(
            arbitration_id=self.arbitration_id,
            data=data,
            is_extended_id=False,
        )

        self.bus.send(msg)

    @staticmethod
    def _bytes_to_float(data: bytes) -> float:
        return struct.unpack("<f", data)[0]  # little-endian

    def close(self) -> None:
        self.bus.shutdown()

    # ===== Write commands =====

    def set_control_state(self, state: int) -> None:
        self._send(self.SET_CONTROL_STATE, [state])

    def set_torque_current_target(self, value: float) -> None:
        self._send(self.SET_TORQUE_CURRENT_TARGET, self._float_to_bytes(value))

    def set_torque_current_soft_limit(self, value: float) -> None:
        self._send(self.SET_TORQUE_CURRENT_SOFT_LIMIT, self._float_to_bytes(value))

    def set_current_kp(self, value: float) -> None:
        self._send(self.SET_CURRENT_KP, self._float_to_bytes(value))

    def set_current_ki(self, value: float) -> None:
        self._send(self.SET_CURRENT_KI, self._float_to_bytes(value))

    def set_angular_velocity_target(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_TARGET, self._float_to_bytes(value))

    def set_angular_velocity_soft_limit(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_SOFT_LIMIT, self._float_to_bytes(value))

    def set_angular_velocity_kp(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_KP, self._float_to_bytes(value))

    def set_angular_velocity_ki(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_KI, self._float_to_bytes(value))

    def set_position_target(self, value: float) -> None:
        self._send(self.SET_POSITION_TARGET, self._float_to_bytes(value))

    def set_position_kp(self, value: float) -> None:
        self._send(self.SET_POSITION_KP, self._float_to_bytes(value))

    def set_position_ki(self, value: float) -> None:
        self._send(self.SET_POSITION_KI, self._float_to_bytes(value))

    def set_position_kd(self, value: float) -> None:
        self._send(self.SET_POSITION_KD, self._float_to_bytes(value))

    def set_electrical_offset(self, value: float) -> None:
        self._send(self.SET_ELECTRICAL_OFFSET, self._float_to_bytes(value))

    def calibrate_electrical_offset(self) -> None:
        self._send(self.CALIBRATE_ELECTRICAL_OFFSET)

    # ===== Read-like commands =====
    # Tohle jen odešle request. Odpověď zatím neposloucháme.

    def get_torque_current_soft_limit(self) -> None:
        self._send(self.GET_TORQUE_CURRENT_SOFT_LIMIT)

    def get_angular_velocity_soft_limit(self) -> None:
        self._send(self.GET_ANGULAR_VELOCITY_SOFT_LIMIT)

    def get_angular_velocity(self) -> None:
        self._send(self.GET_ANGULAR_VELOCITY)

    def get_position_target(self) -> None:
        self._send(self.GET_POSITION_TARGET)

    # def get_position(self) -> None:
    #     self._send(self.GET_POSITION)


    def get_position(self, timeout: float = 1.0) -> float | None:
        # pošli request
        self._send(self.GET_POSITION)

        # čekej na odpověď
        msg = self.bus.recv(timeout)

        if msg is None:
            print("Timeout waiting for position")
            return None

        data = msg.data

        # validace
        if len(data) < 6:
            print("Invalid message length")
            return None

        if data[0] != self.device_id:
            print("Wrong device id")
            return None

        if data[1] != self.GET_POSITION:
            print("Unexpected command")
            return None

        # decode float z data[2:6]
        position = self._bytes_to_float(bytes(data[2:6]))
        return position

    def get_relative_position(self) -> None:
        self._send(self.GET_RELATIVE_POSITION)

    def get_electrical_offset(self) -> None:
        self._send(self.GET_ELECTRICAL_OFFSET)


if __name__ == "__main__":
    import time
    client = MotorClient(
        channel="/dev/tty.usbmodem206F327555481",
        arbitration_id=0x123,
    )



    client.set_control_state(0)
    time.sleep(0.2)
    client.set_angular_velocity_target(500)

    # time.sleep(0.5)
    # print("opened")
    # time.sleep(0.5)

    # client.set_control_state(3)
    # time.sleep(0.2)

    # # client.set_torque_current_target(0.9)
    # # time.sleep(2)

    # client.set_angular_velocity_soft_limit(1000.0)
    # time.sleep(0.1)

    # # client.set_position_target(10.0)
    # # for i in range(40):
    # #     time.sleep(0.1)
    # #     print(client.get_position())

    # client.set_torque_current_soft_limit(1)
    # time.sleep(0.01)

    # client.set_position_target(2)

    # pos = client.get_position()
    # print(pos)

    # while True:
    #     pos = client.get_position()
    #     print(pos)
    #     if pos > 1:
    #         time.sleep(0.01)
    #         client.set_control_state(3)
    #         time.sleep(0.01)
    #         client.set_position_target(1)
    #     if pos < 0:
    #         time.sleep(0.01)
    #         client.set_control_state(3)
    #         time.sleep(0.01)
    #         client.set_position_target(0.00)
    #     else:
    #         time.sleep(0.01)
    #         client.set_control_state(0)
            

    