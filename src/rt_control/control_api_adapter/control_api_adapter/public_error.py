from __future__ import annotations

import enum
from dataclasses import dataclass


class PublicErrorCode(enum.IntEnum):
    """RT-Control subset of the contract-owned DREE error-code table."""

    SUCCESS = 0
    INVALID_GOAL = 4

    RT_ENABLE_FAILED = 1021
    RT_PUMP_UNAVAILABLE = 1022
    RT_RESET_FAILED = 1023
    RT_DISABLE_FAILED = 1024
    RT_PUMP_COMMAND_REJECTED = 1025
    RT_POSSIBLE_LOAD_HELD = 1026
    RT_RESTART_REQUIRED = 1027
    RT_INTERNAL_ERROR = 1090

    RT_SERVICE_UNAVAILABLE = 1100
    RT_ENABLE_MANAGER_NOT_READY = 1101
    RT_OPERATION_IN_PROGRESS = 1102
    RT_PLC_UNAVAILABLE = 1121
    RT_VACUUM_NOT_ESTABLISHED = 1160


def is_retryable(code: int | PublicErrorCode) -> bool:
    """Derive ErrorInfo.retryable from the DREE recovery digit."""

    value = int(code)
    if value == int(PublicErrorCode.SUCCESS):
        return False
    if not 0 < value < 10000:
        raise ValueError(f"error code is outside the DREE range: {value}")
    return (value // 100) % 10 == 1


@dataclass(frozen=True)
class ErrorInfoData:
    code: PublicErrorCode
    message: str
    origin: str = "rt_control"

    @property
    def retryable(self) -> bool:
        return is_retryable(self.code)


def error_info(code: PublicErrorCode, message: str) -> ErrorInfoData:
    return ErrorInfoData(code=code, message=str(message))


def assign_error_info(message, value: ErrorInfoData) -> None:
    """Populate a generated robot_system_interfaces/ErrorInfo message."""

    message.code = int(value.code)
    message.retryable = value.retryable
    message.message = value.message
    message.origin = value.origin
