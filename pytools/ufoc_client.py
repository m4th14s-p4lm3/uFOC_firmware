import can
import struct
from enum import IntEnum
from typing import Optional


class ControlState(IntEnum):
    NO_CONTROL       = 0
    CURRENT_CONTROL  = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3


class AngleUnits(IntEnum):
    RADIANS   = 0
    ROTATIONS = 1
    DEGREES   = 2


class uFOCclient:
    # Device
    CAN_COMMUNICATION_DEVICE_ID = 0x00

    # General
    SET_CONTROL_STATE    = 0x00
    SET_USER_ANGLE_UNITS = 0x01
    GET_USER_ANGLE_UNITS = 0x02

    # Current control
    SET_TORQUE_CURRENT_TARGET     = 0x06
    SET_TORQUE_CURRENT_SOFT_LIMIT = 0x07
    GET_TORQUE_CURRENT_SOFT_LIMIT = 0x08
    SET_CURRENT_KP                = 0x09
    SET_CURRENT_KI                = 0x0A

    # Velocity control
    SET_ANGULAR_VELOCITY_TARGET     = 0x10
    SET_ANGULAR_VELOCITY_SOFT_LIMIT = 0x11
    GET_ANGULAR_VELOCITY_SOFT_LIMIT = 0x12
    GET_ANGULAR_VELOCITY            = 0x13
    SET_ANGULAR_VELOCITY_KP         = 0x14
    SET_ANGULAR_VELOCITY_KI         = 0x15

    # Position control
    SET_POSITION_TARGET   = 0x1A
    GET_POSITION_TARGET   = 0x1B
    GET_POSITION          = 0x1C
    GET_RELATIVE_POSITION = 0x1D
    SET_POSITION_KP       = 0x1E
    SET_POSITION_KI       = 0x1F
    SET_POSITION_KD       = 0x20

    # Encoder
    GET_ELECTRICAL_OFFSET       = 0x26
    SET_ELECTRICAL_OFFSET       = 0x27
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
        return list(struct.pack("<f", value))

    @staticmethod
    def _bytes_to_float(data: bytes) -> float:
        return struct.unpack("<f", data)[0]

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

    def _request_float(self, command: int, timeout: float = 1.0) -> float | None:
        self._send(command)

        while True:
            msg = self.bus.recv(timeout)
            if msg is None:
                return None

            if msg.arbitration_id != self.arbitration_id:
                continue

            data = msg.data
            if len(data) < 6:
                continue

            if data[0] != self.device_id:
                continue

            if data[1] != command:
                continue

            return self._bytes_to_float(bytes(data[2:6]))

    def _request_byte(self, command: int, timeout: float = 1.0) -> int | None:
        # GET_USER_ANGLE_UNITS: firmware writes value to data[3] (note: firmware
        # currently missing communication_send() in this branch – will timeout)
        self._send(command)

        while True:
            msg = self.bus.recv(timeout)
            if msg is None:
                return None

            if msg.arbitration_id != self.arbitration_id:
                continue

            data = msg.data
            if len(data) < 3:
                continue

            if data[0] != self.device_id:
                continue

            if data[1] != command:
                continue

            return int(data[2])

    def close(self) -> None:
        self.bus.shutdown()

    # ----- General -----

    def set_control_state(self, state: ControlState | int) -> None:
        self._send(self.SET_CONTROL_STATE, [int(state)])

    def set_user_angle_units(self, units: AngleUnits | int) -> None:
        self._send(self.SET_USER_ANGLE_UNITS, [int(units)])

    def get_user_angle_units(self, timeout: float = 1.0) -> AngleUnits | None:
        val = self._request_byte(self.GET_USER_ANGLE_UNITS, timeout)
        if val is None:
            return None
        return AngleUnits(val)

    # ----- Current control – write -----

    def set_torque_current_target(self, value: float) -> None:
        self._send(self.SET_TORQUE_CURRENT_TARGET, self._float_to_bytes(value))

    def set_torque_current_soft_limit(self, value: float) -> None:
        self._send(self.SET_TORQUE_CURRENT_SOFT_LIMIT, self._float_to_bytes(value))

    def set_current_kp(self, value: float) -> None:
        self._send(self.SET_CURRENT_KP, self._float_to_bytes(value))

    def set_current_ki(self, value: float) -> None:
        self._send(self.SET_CURRENT_KI, self._float_to_bytes(value))

    # ----- Current control – read -----

    def get_torque_current_soft_limit(self, timeout: float = 1.0) -> float | None:
        return self._request_float(self.GET_TORQUE_CURRENT_SOFT_LIMIT, timeout)

    # ----- Velocity control – write -----

    def set_angular_velocity_target(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_TARGET, self._float_to_bytes(value))

    def set_angular_velocity_soft_limit(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_SOFT_LIMIT, self._float_to_bytes(value))

    def set_angular_velocity_kp(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_KP, self._float_to_bytes(value))

    def set_angular_velocity_ki(self, value: float) -> None:
        self._send(self.SET_ANGULAR_VELOCITY_KI, self._float_to_bytes(value))

    # ----- Velocity control – read -----

    def get_angular_velocity_soft_limit(self, timeout: float = 1.0) -> float | None:
        return self._request_float(self.GET_ANGULAR_VELOCITY_SOFT_LIMIT, timeout)

    def get_angular_velocity(self, timeout: float = 1.0) -> float | None:
        return self._request_float(self.GET_ANGULAR_VELOCITY, timeout)

    # ----- Position control – write -----

    def set_position_target(self, value: float) -> None:
        self._send(self.SET_POSITION_TARGET, self._float_to_bytes(value))

    def set_position_kp(self, value: float) -> None:
        self._send(self.SET_POSITION_KP, self._float_to_bytes(value))

    def set_position_ki(self, value: float) -> None:
        self._send(self.SET_POSITION_KI, self._float_to_bytes(value))

    def set_position_kd(self, value: float) -> None:
        self._send(self.SET_POSITION_KD, self._float_to_bytes(value))

    # ----- Position control – read -----

    def get_position_target(self, timeout: float = 1.0) -> float | None:
        return self._request_float(self.GET_POSITION_TARGET, timeout)

    def get_position(self, timeout: float = 1.0) -> float | None:
        return self._request_float(self.GET_POSITION, timeout)

    def get_relative_position(self, timeout: float = 1.0) -> float | None:
        return self._request_float(self.GET_RELATIVE_POSITION, timeout)

    # ----- Encoder -----

    def get_electrical_offset(self, timeout: float = 1.0) -> float | None:
        return self._request_float(self.GET_ELECTRICAL_OFFSET, timeout)

    def set_electrical_offset(self, value: float) -> None:
        self._send(self.SET_ELECTRICAL_OFFSET, self._float_to_bytes(value))

    def calibrate_electrical_offset(self) -> None:
        self._send(self.CALIBRATE_ELECTRICAL_OFFSET)


if __name__ == "__main__":
    import time
    client = uFOCclient(
        channel="/dev/tty.usbmodem206F327555481",
        arbitration_id=0x123,
    )

    print(client.get_user_angle_units())

    # client.set_control_state(ControlState.CURRENT_CONTROL)
    # time.sleep(0.1)
    # client.set_torque_current_target(1)
    # time.sleep(0.1)
    # for i in range(50):
    #     print(client.get_angular_velocity())
    #     time.sleep(0.1)