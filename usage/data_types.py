import enum
from datetime import datetime

import attr
from pyrsistent import pmap
from pyrsistent.typing import PMap, PVector


@attr.s(kw_only=True, frozen=True)
class AxisConfig():
    """The single axis configurations (motion unit, io) the motion control card."""

    axis_name: int = attr.ib()
    """The axes that we need to use. You must configure the axis_units, axis_max_speed,
    axis_max_accel, default_axis_home_search_speed and axis_enable_outputs. While the axis_error_rst_outputs and
    axis_error_inputs is optional."""

    max_pulse_frequency: int = attr.ib()
    """The max frequency of the output pulse. In some motion control cards, it has a default frequency, but it might
    not meet the requirements. For instance, the default value of ZMotion ECI3600 is 1000000 (1M), since max is 10M.
    Be careful to configure it by the motor driver."""

    units: float = attr.ib()
    """The Pulse Equivalent. Here, it mean the plus number that motion control card needs to send to motor leading a
    1mm linear motion (or 1-degree rotation motion). It supports .5f precision, with Dict[axis_number, units_number]."""

    pos_limit: tuple[float, float] = attr.ib()
    """The position limit of the axis, with [min_position, max_position]."""

    max_speed: float = attr.ib()
    """The max speed that every-axis motor supports, with Dict[axis_number, max_speed]. The unit is units/s."""

    max_accel: float = attr.ib()
    """The max acceleration/deceleration that every-axis motor supports, with Dict[axis_number, max_accel].
    The unit is units/s/s."""

    default_home_search_speed: float = attr.ib()
    """The search speed of homing with the format Dict[axis_number, speed] used for forward search signal,
    e.g. limit, home. The unit of speed is units/s."""

    default_home_search_accel: float = attr.ib()
    """The search acceleration of homing with the format Dict[axis_number, acceleration]. The unit is units/s^2."""

    default_home_search_decel: float = attr.ib()
    """The search deceleration of homing with the format Dict[axis_number, deceleration]. The unit is units/s^2."""

    default_home_creep_speed: float = attr.ib()
    """The creep speed of axis homing used to left the home, and stop immediately when the digital input of the home
    sensor is False."""

    home_di_address: int = attr.ib()
    """The digital inputs for the home sensor of every axis. If you use the default configured digital inputs, just
    skip this."""

    reverse_limit_di_address: int = attr.ib()
    """The digital inputs for the reverse-limit sensor of every axis. If you use the default configured digital inputs,
    just skip this."""

    forward_limit_di_address: int = attr.ib()
    """The digital inputs for the forward-limit sensor of every axis. If you use the default configured digital inputs,
    just skip this. You must ensure the direction from reverse-limit sensor to forward-limit sensor is same with the
    positive step direction of every original axis motion."""

    need_invert_direction: bool = attr.ib()
    """Invert the axis direction for external application. In the case that the original positive step direction of
    the motor is contract with what you want, you can use this to invert the direction.
    Dict[axis_number, whether_need_invert_direction]."""

    enable_second_after_first_power_on: float = attr.ib(default=0.0, )
    """Some motors (e.g., the linear motors used in Apparel) need enable several seconds (10 second for Apparel) after
    the power on. Only finish this, the motion control card can command the motor to move."""

    enable_do_address: int = attr.ib()
    """The digital outputs for enable the axes, with Dict[axis_number, do_number]."""

    error_di_address: int = attr.ib()
    """The digital inputs for reading the error status of the motor driver, with Dict[axis_number, di_number]."""

    no_limit_sensor: bool = attr.ib(default=False, )
    """There is no limit sensor for this axis."""

    encoder_type: int = attr.ib(default=1, )
    """The encoder type of this axis."""

    encoder_units: float = attr.ib(default=0.0, )
    """The units for encoder pulse."""


@attr.s(kw_only=True, frozen=True)
class MotionControlCardConfig:
    """Configurations (axes, motion unit, io) for the motion control card."""

    ip_address: str = attr.ib()
    """The IP address of the motion control card."""

    axis_configs: PMap[int, AxisConfig] = attr.ib()
    """The configurations of the motion axes, with Dict[axis_number, di_number] format."""

    invert_di_addresses: PVector[int] = attr.ib()
    """The digital inputs that need to invert its status. The inverts of the digital inputs in
    [monitor_di_min_address, monitor_di_max_address] but not in invert_di_addresses will be off."""

    emergency_stop_di_address: int = attr.ib()
    """Use this to monitor the emergency stop press to reject the motion command from the external application."""

    monitor_di_min_address: int = attr.ib()
    """We monitor the digital inputs [monitor_di_min_address, monitor_di_max_address] in the background."""

    monitor_di_max_address: int = attr.ib()
    """We monitor the digital inputs [monitor_di_min_address, monitor_di_max_address] in the background."""

    monitor_do_min_address: int = attr.ib()
    """We monitor the digital inputs [monitor_di_min_address, monitor_di_max_address] in the background."""

    monitor_do_max_address: int = attr.ib()
    """We monitor the digital inputs [monitor_di_min_address, monitor_di_max_address] in the background."""


@attr.s(kw_only=True, frozen=True)
class AxisState:
    date_time: datetime = attr.ib()
    axis_name: int = attr.ib()
    position: float = attr.ib()
    axis_status: int = attr.ib()
    is_moving: bool = attr.ib()
    is_trigger_reverse_limit: bool = attr.ib()
    is_trigger_forward_limit: bool = attr.ib()
    is_enabled: bool = attr.ib()
    is_finding_home: bool = attr.ib()


@attr.s(kw_only=True, frozen=True)
class MotionControlCardState:
    date_time: datetime = attr.ib()
    digital_inputs: PMap[int, bool] = attr.ib(default=pmap({}))
    digital_outputs: PMap[int, bool] = attr.ib(default=pmap({}))
    axis_states: PMap[int, AxisState] = attr.ib(default=pmap({}))


@enum.unique
class AxisStopMode(enum.Enum):
    UNSPECIFIED = 0
    CANCEL_CURRENT_MOTION = 1
    CANCEL_MOTION_COMMAND_IN_BUFFER = 2
    CANCEL_CURRENT_MOTION_AND_COMMAND_IN_BUFFER = 3
    STOP_SENDING_PULSE_IMMEDIATELY = 4
