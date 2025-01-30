import enum
import gc
import logging
import threading
import time
from collections.abc import Generator, Sequence
from datetime import datetime
from typing import Any

from pyrsistent import pmap
from pyrsistent.typing import PMap
from requests.exceptions import ConnectionError

from usage.data_types import (
    AxisState,
    AxisStopMode,
    MotionControlCardConfig,
    MotionControlCardState,
)
from runtime_compilation.zmotion_py import (
    Handle_Wrapper,
    zmotion_close,
    zmotion_direct,
    zmotion_execute,
    zmotion_init,
    zmotion_open_ethernet,
)
from usage.stream_utils import SubscribableValue
from usage.utils import get_current_datetime

READ_CARD_STATE_PERIOD_S = 0.0125
ERROR_FOR_POSITION_EQUAL_MM = 2
NORMAL_WAITING_TIME_BETWEEN_DISABLE_AND_ENABLE_S = 1.5
GC_DURATION_THRESHOLD = 0.001
MAX_READ_STATE_FAILURE_COUNT = 3
RECONNECT_SLEEP_TIME_S = 5


@enum.unique
class _IOReadType(enum.Enum):
    """The io type for reading."""

    UNSPECIFIED = 0
    """Status not specified."""

    READ_DIGITAL_INPUT = 1
    """Read the digital input."""

    READ_DIGITAL_OUTPUT = 2
    """Read the digital output."""


@enum.unique
class _SendCommandInterface(enum.Enum):
    """The io type for reading."""

    UNSPECIFIED = 0
    """Status not specified."""

    ZMC_DIRECT = 1
    """The interface using the `ZMC_DirectCommand` of ZMotion library. It only supports motion functions, parameters
    and array variables configuration."""

    ZMC_EXECUTE = 2
    """The universal command execution interface using the `ZMC_Execute` of ZMotion library. It includes the function
    of ZMC_DIRECT. It will get block when the controller is not buffered"""


def _decollate_interval_with_32_length(min_number: int, max_number: int) -> list[tuple[int, int]]:
    """Decollate the interval [min_number, max_number] to a multi intervals with a constant length (32).

    Some examples:
    >>> _decollate_interval_with_32_length(0, 25) -> [(0, 25)]
    >>> _decollate_interval_with_32_length(0, 65) -> [(0, 31), (32, 63), (64, 65)]
    >>> _decollate_interval_with_32_length(5, 40) -> [(5, 36), (37, 40)]
    """
    interval_number = int((max_number - min_number) / 32) + 1
    decollated_interval: list[tuple[int, int]] = [
        (32 * interval_order + min_number, 32 * interval_order + 31 + min_number)
        for interval_order in range(interval_number)
    ]
    last_interval = decollated_interval.pop()
    decollated_interval.append((last_interval[0], max_number))
    return decollated_interval


class ZMotionControlCard:
    """ZMotion control card.
    
    Had been tested on the ECI2000/ECI3000 series.
    If you want to use this class to connect to zmoiton control card
    practicality and test it, please refer to ./example.py.
    """

    def __init__(self, config: MotionControlCardConfig):
        # monitor gc
        gc.set_threshold(300, 10, 10)
        self._gc_start_time = time.perf_counter()
        gc.callbacks.append(self._gc_callback)

        self._ip_address = config.ip_address
        self._cmd_lock = threading.Lock()
        self._config: MotionControlCardConfig = config
        # TODO: get the service name from json config
        self._card_name = "MotionControlCard " + self._ip_address
        self._axis_enable_do_addresses = [axis_config.enable_do_address for axis_config in config.axis_configs.values()]

        # init zmotion controller practicality.
        zmotion_init()
        self.controller_handle = Handle_Wrapper()
        success = zmotion_open_ethernet(self._ip_address, self.controller_handle)
        if not success == 0:
            raise ConnectionError(
                f"Failed to connect to motion controller at ip: {self._ip_address} with error code {success}."
            )
        logging.info(f"Connected to motion control card {self._ip_address}")

        self._thread_running = True

        self._state: SubscribableValue[MotionControlCardState] = SubscribableValue(
            initial_value=MotionControlCardState(date_time=get_current_datetime())
        )
        self._has_latest_state = threading.Event()
        self._monitor_state_worker = threading.Thread(target=self._read_card_state_loop, daemon=True)
        self._monitor_state_worker.start()

        self._zmotion_init()
        logging.info("Finished the initial of the ZMotion control card.")

    def _gc_callback(self, phase: str, info: Any) -> None:
        if phase == "start":
            self._gc_start_time = time.perf_counter()
        elif phase == "stop":
            duration = time.perf_counter() - self._gc_start_time
            if duration > GC_DURATION_THRESHOLD:
                logging.info(f"gc collection time: {duration}")

    def __del__(self) -> None:
        self.emergency_stop()
        self._thread_running = False
        self._monitor_state_worker.join()
        zmotion_close(self.controller_handle)
        gc.callbacks.remove(self._gc_callback)

    def _send_cmd(self, cmd: str, cmd_interface: _SendCommandInterface = _SendCommandInterface.ZMC_DIRECT) -> str:
        """Send a command to the motion command card practicality.

        Now, the duration of sending one command to the motion control card is about 0.15ms - 0.35ms.

        Parameters
        ----------
        cmd: str
            The send command.
        cmd_interface: _SendCommandInterface
            The interface of the zmotion library that this sending uses.

        Returns
        -------
        response : str
            The response detail.
        """
        with self._cmd_lock:
            if cmd_interface == _SendCommandInterface.ZMC_DIRECT:
                success, response = zmotion_direct(self.controller_handle, cmd)
            elif cmd_interface == _SendCommandInterface.ZMC_EXECUTE:
                success, response = zmotion_execute(self.controller_handle, cmd)
            else:
                raise ValueError(
                    f"A invalid enum interface type {cmd_interface} for sending command to motion control card."
                )
        if not success == 0:
            # TODO get the detailed reason of the failure. JIRA ticket https://covariant.atlassian.net/browse/PUT-1066
            raise ConnectionError(
                f"{self._card_name} {cmd_interface} failed to send {cmd} to motion control card with {success} status."
                f"Its function response is {response}."
            )
        return response

    def _zmotion_init(self) -> None:
        # Information output settings: output warning and error to the ZDevelop (A ZMotion software).
        self._send_cmd(cmd="ERRSWITCH=2")
        # All axes stop immediately: cancel current and buffer exercise.
        self._send_cmd(cmd="RAPIDSTOP(2)")
        for axis, axis_config in self._config.axis_configs.items():
            # Configure the axis control type: pulse-direction step or servo control.
            # to be compatible with the old configuration, don't support encoder_type=0 which means virtual axis.
            self._send_cmd(cmd=f"ATYPE({axis})={1 if axis_config.encoder_type == 0 else axis_config.encoder_type}")
            # Configure the Pulse Equivalent
            self._send_cmd(cmd=f"UNITS({axis})={axis_config.units:.5f}")
            # Configure the pulse mode: pulse/direction (pulse positive logic) (Forward)
            self._send_cmd(cmd=f"INVERT_STEP({axis})=0")
            self._send_cmd(cmd=f"MAX_SPEED AXIS({axis})={axis_config.max_pulse_frequency}")
            if not axis_config.no_limit_sensor:
                self._send_cmd(cmd=f"DATUM_IN({axis})={axis_config.home_di_address}")
                self._send_cmd(cmd=f"REV_IN({axis})={axis_config.reverse_limit_di_address}")
                self._send_cmd(cmd=f"FWD_IN({axis})={axis_config.forward_limit_di_address}")
            else:
                self._send_cmd(cmd=f"DATUM_IN({axis})={-1}")
                self._send_cmd(cmd=f"REV_IN({axis})={-1}")
                self._send_cmd(cmd=f"FWD_IN({axis})={-1}")
            self._send_cmd(cmd=f"ALM_IN({axis})={axis_config.error_di_address}")
            # The decel for CANCEL axis movement, RAPIDSTOP, trigger limit sensor or other exception stop. 20m/s/s.
            self._send_cmd(cmd=f"FASTDEC({axis})=20000")
            if not axis_config.no_limit_sensor:
                # Configure the creep speed, accel, decel.
                self._send_cmd(cmd=f"CREEP({axis})={axis_config.default_home_creep_speed}")
            self._set_axis_speed_and_accel(
                axis=axis, speed=axis_config.max_speed, accel=axis_config.max_accel, decel=axis_config.max_accel
            )
        for invert_input in self._config.invert_di_addresses:
            self._send_cmd(cmd=f"INVERT_IN({invert_input},ON)")
        for di in range(self._config.monitor_di_min_address, self._config.monitor_di_max_address + 1):
            if di in self._config.invert_di_addresses:
                continue
            self._send_cmd(cmd=f"INVERT_IN({di},OFF)")

    def _read_io(self, io_type: _IOReadType, min_address: int, max_address: int) -> PMap[int, bool]:
        """Read the digital input or output values.

        Parameters
        ----------
        io_type: _IOReadType
            The input/output to be read.
        min_address: int
            Min address.
        max_address: int
            Max address.

        Returns
        -------
        result: PMap[int, bool]
            PMap[io address, value].
        """
        if max_address <= min_address:
            raise ValueError(f"The min address {min_address} must smaller {max_address}")
        if io_type == _IOReadType.READ_DIGITAL_OUTPUT:
            read_cmd = "OP"
        elif io_type == _IOReadType.READ_DIGITAL_INPUT:
            read_cmd = "IN"
        else:
            raise ValueError(f"A invalid enum io type {io_type} for reading io status")
        io_values: dict[int, bool] = {}
        for interval_left, interval_right in _decollate_interval_with_32_length(min_address, max_address):
            io_binary_result = int(self._send_cmd(cmd=f"?{read_cmd}({interval_left},{interval_right})"))
            # input_result is a binary result for [interval_right, ..., interval_left] mask. For instance, if interval
            # is [0,2], the io_binary_result 6 (0b110) means io_0 = False, io_1 = True, io_2 = True.
            for io_address in range(interval_left, interval_right + 1):
                io_values[io_address] = bool(io_binary_result & 0x01)
                io_binary_result >>= 1
        return pmap(io_values)

    def _read_card_state(self) -> MotionControlCardState:
        with self._read_input_io_perf:
            digital_input_values: PMap[int, bool] = self._read_io(
                io_type=_IOReadType.READ_DIGITAL_INPUT,
                min_address=self._config.monitor_di_min_address,
                max_address=self._config.monitor_di_max_address,
            )
        with self._read_output_io_perf:
            digital_output_values: PMap[int, bool] = self._read_io(
                io_type=_IOReadType.READ_DIGITAL_OUTPUT,
                min_address=self._config.monitor_do_min_address,
                max_address=self._config.monitor_do_max_address,
            )
        axis_states: dict[int, AxisState] = {}
        date_time = datetime.utcnow()
        for axis in self._config.axis_configs:
            with self._read_status_perf:
                axis_status = int(self._send_cmd(cmd=f"?AXISSTATUS({axis})"))
            with self._read_idle_perf:
                is_moving: bool = int(self._send_cmd(cmd=f"?IDLE({axis})")) == 0
            axis_states[axis] = AxisState(
                date_time=date_time,
                axis_name=axis,
                position=self._get_position(axis=axis),
                axis_status=axis_status,
                is_moving=is_moving,
                is_trigger_reverse_limit=bool(axis_status & 0b00100000),
                is_trigger_forward_limit=bool(axis_status & 0b00010000),
                is_enabled=digital_output_values[self._config.axis_configs[axis].enable_do_address],
                is_finding_home=bool(axis_status & 0b01000000),
            )
        return MotionControlCardState(
            date_time=date_time,
            digital_inputs=digital_input_values,
            digital_outputs=digital_output_values,
            axis_states=pmap(axis_states),
        )

    def _read_card_state_loop(self) -> None:
        read_state_error_cnt = 0
        while self._thread_running:
            start_time = time.perf_counter()
            try:
                motion_control_card_state: MotionControlCardState = self._read_card_state()
                self._state.set_value(new_value=motion_control_card_state)
                read_state_error_cnt = 0
            except Exception as err:
                self._state.set_error(error=err)
                logging.error(f"Get a exception while read the motion control card state. {err}")
                read_state_error_cnt += 1
                if read_state_error_cnt >= MAX_READ_STATE_FAILURE_COUNT:
                    logging.warning("reconnect to motion card")
                    time.sleep(RECONNECT_SLEEP_TIME_S)
                    try:
                        self.reconnect()
                        read_state_error_cnt = 0
                    except Exception:
                        pass
                    continue

            self._has_latest_state.set()
            # The cycle time is depending on how many commands we send to motion control card practicality, since a
            # command costs about 0.4ms now. For Apparel station using four axes, the cycle costs about 8ms, including
            # communication time and decode time.
            time.sleep(max(0.0, (READ_CARD_STATE_PERIOD_S - (time.perf_counter() - start_time))))

    def _verify_speed_and_accel(self, axis: int, speed: float, accel: float, decel: float) -> None:
        max_speed = self._config.axis_configs[axis].max_speed
        if (speed > max_speed) or (speed < 0):
            raise ValueError(
                f"{self._card_name}: the target speed {speed} must be in [0, {max_speed}], use a smaller speed."
            )
        max_accel = self._config.axis_configs[axis].max_accel
        if (accel > max_accel) or (accel < 0):
            raise ValueError(
                f"{self._card_name}: the target acceleration {accel} must be in [0, {max_accel}], use a smaller "
                "acceleration."
            )
        if (decel > max_accel) or (accel < 0):
            raise ValueError(
                f"{self._card_name}: the target deceleration {decel} must be in [0, {max_accel}], use a smaller"
                "deceleration."
            )

    def _set_axis_speed_and_accel(self, axis: int, speed: float, accel: float, decel: float) -> None:
        """Set speed, accel and decel.

        Since it takes effect immediately, be careful to use it.

        Parameters
        ----------
        axis: int
            The target axis.
        speed: float
            float: the input target speed.
        accel: float
            The target accel.
        decel: float
            The target deceleration.
        """
        self._verify_speed_and_accel(axis=axis, speed=speed, accel=accel, decel=accel)
        self._send_cmd(cmd=f"SPEED({axis})={speed}")
        self._send_cmd(cmd=f"ACCEL({axis})={accel}")
        self._send_cmd(cmd=f"DECEL({axis})={decel}")

    def _set_out(self, do_address: int, value: bool | int) -> None:
        self._send_cmd(cmd=f"op({do_address},{'1' if value else '0'})")

    def _set_outs(self, target_values: dict[int, bool]) -> None:
        # TODO operate all requested outputs meantime, to save the time of communication latency.
        # JIRA ticket https://covariant.atlassian.net/browse/PUT-1057
        for do_address, value in target_values.items():
            self._set_out(do_address=do_address, value=value)

    def _wait_until_idle(self, axes: list[int], timeout_s: float | None = 60) -> None:
        self._has_latest_state.wait()
        start_time = time.perf_counter()
        for latest_state in self._state.subscribe():
            if latest_state is not None:
                is_moving = [latest_state.axis_states[axis].is_moving for axis in axes]
                if not any(is_moving):
                    return
            if timeout_s is not None and (time.perf_counter() - start_time) > timeout_s:
                raise TimeoutError(f"{self._card_name}: wait for axis {axes} is idle with timeout_s {timeout_s}.")

    def _wait_until_found_home(self, axes: list[int], timeout_s: float | None = 60) -> None:
        self._has_latest_state.wait()
        start_time = time.perf_counter()
        for latest_state in self._state.subscribe():
            if latest_state is not None:
                is_moving = [latest_state.axis_states[axis].is_finding_home for axis in axes]
                has_alarm = [bool(latest_state.axis_states[axis].axis_status & 0x400000) for axis in axes]
                if any(has_alarm):
                    self.stop_axes(axes=axes, mode=AxisStopMode.CANCEL_CURRENT_MOTION)
                    raise RuntimeError(f"{self._card_name}: axis {axes} find home error, alarm = {has_alarm}")
                if not any(is_moving):
                    return
            if timeout_s is not None and (time.perf_counter() - start_time) > timeout_s:
                self.stop_axes(axes=axes, mode=AxisStopMode.CANCEL_CURRENT_MOTION)
                raise TimeoutError(f"{self._card_name}: wait for axis {axes} is idle with timeout_s {timeout_s}.")

    def _get_position(self, axis: int) -> float:
        """Get the current position of the target axis.

        If there is an encoder, use cmd=f"?MPOS({axis})". Otherwise, use cmd=f"?DPOS({axis})".

        Parameters
        ----------
        axis: int
            The target axis.

        Returns
        -------
        float
            The current position, unit is mm (or degree).
        """
        position = float(self._send_cmd(cmd=f"?MPOS({axis})"))
        axis_config = self._config.axis_configs[axis]
        if axis_config.encoder_units != 0.0:
            position = position * axis_config.units / axis_config.encoder_units

        return -position if axis_config.need_invert_direction else position

    def subscribe_control_card_state(self) -> Generator[MotionControlCardState, None, None]:
        self._has_latest_state.wait()
        yield from self._state.subscribe()

    def reconnect(self) -> None:
        """Reconnect to the saved ip address.

        This is usually called after powering cycling the motion control card, such as after opening and closing the safety door
        or triggering the tote buffer line conveyor light curtain.
        Note that the running threads and subscribable values of this instance are not affected. Only the ZMotion connection is reset
        and re-initialized.

        Raises
        ------
        ConnectionError
            Any error during the reconnect.
        """
        self.emergency_stop()
        zmotion_close(self.controller_handle)
        zmotion_init()
        self.controller_handle = Handle_Wrapper()
        success = zmotion_open_ethernet(self._ip_address, self.controller_handle)
        if not success == 0:
            raise ConnectionError(
                f"Failed to reconnect to motion controller at ip: {self._ip_address} with error code {success}."
            )
        logging.info(f"Reconnected to motion control card {self._ip_address}")

        self._init_perf()

        self._zmotion_init()

    def axes_enable(self, axes: Sequence[int]) -> None:
        def get_input_error_signal(axis: int) -> bool:
            state = self._state.get_value()
            return bool(state.axis_states[axis].axis_status & 0x400000)

        for axis in axes:
            if axis not in self._config.axis_configs:
                raise ValueError(
                    f"{self._card_name}: just can enable the axes {self._config.axis_configs.keys()}, "
                    f"not the target axis {axis}."
                )

        # Some motor driver errors can be killed by enable the axis several second, and some motors need a
        # several-second enable after the power-on. Ensure at least 0.5 second for enabling.
        wait_times_after_axis_enable: list[float] = [NORMAL_WAITING_TIME_BETWEEN_DISABLE_AND_ENABLE_S]
        for axis in axes:
            if get_input_error_signal(axis=axis):
                self.axes_disable(axes=[axis])
                wait_times_after_axis_enable.append(self._config.axis_configs[axis].enable_second_after_first_power_on)

        if len(wait_times_after_axis_enable) > 1:
            time.sleep(max(wait_times_after_axis_enable))

        self._set_outs(target_values={self._config.axis_configs[axis].enable_do_address: True for axis in axes})

        start_time = time.perf_counter()
        while True:
            time.sleep(0.5)
            failed_axes: list[int] = [axis for axis in axes if get_input_error_signal(axis=axis)]
            if len(failed_axes) == 0:
                logging.info(f"enable axes {axes} ok, duration = {(time.perf_counter()-start_time):.4f}")
                return
            if time.perf_counter() - start_time > NORMAL_WAITING_TIME_BETWEEN_DISABLE_AND_ENABLE_S:
                raise AssertionError(
                    f"{self._card_name}: Can't enable the axis {failed_axes}, because the motion control card gets a error"
                    f"from the external. The motor driver has a error, please check the motor driver to find the reason"
                    "and solve it."
                )

    def axes_disable(self, axes: Sequence[int]) -> None:
        for axis in axes:
            if axis not in self._config.axis_configs:
                raise ValueError(
                    f"{self._card_name}: Just can disable the axes {self._config.axis_configs.keys()}, "
                    f"not the target axis {axis}."
                )
        self._set_outs(target_values={self._config.axis_configs[axis].enable_do_address: False for axis in axes})

    def subscribe_position(self, axis: int) -> Generator[float, None, None]:
        """Get the current position of the target axis.

        Parameters
        ----------
        axis: int
            The target axis.

        Yield
        -----
        float
            The current position, unit is mm (or degree).
        """
        for card_state in self._state.subscribe():
            yield card_state.axis_states[axis].position

    def get_position(self, axis: int) -> float:
        """Get the current position of the target axis.

        Parameters
        ----------
        axis: int
            The target axis.

        Returns
        -------
        float
            The current position, unit is mm (or degree).
        """
        position = next(self.subscribe_position(axis=axis))
        return position

    def _wait_speed(
        self, axis: int, speed: float, min_error: float = 1.0e-3, interval: float = 0.05, timeout: float = 5.0
    ) -> None:
        """Wait the axis's speed to reach the target speed."""
        error = max(speed / 1000, min_error)
        start_time = time.perf_counter()
        mspeed: float | None = None
        while time.perf_counter() - start_time < timeout:
            mspeed = abs(float(self._send_cmd(f"?MSPEED({axis})")))
            if abs(mspeed - speed) < error:
                return
            time.sleep(interval)
        raise TimeoutError(f"waiting axis {axis}, speed = {speed}, mspeed = {mspeed} timeout.")

    def rotate(self, axis: int, speed: float, accel: float, direction: bool, wait: bool) -> None:
        """Rotate a roller forever, stop it if speed is zero."""
        if axis not in self._config.axis_configs:
            raise ValueError(
                f"{self._card_name}: Just support to move the axes {self._config.axis_configs.keys()}, "
                f"not the target axis {axis}."
            )

        if abs(speed) < 1e-5:
            self.stop_axes([axis], AxisStopMode.CANCEL_CURRENT_MOTION_AND_COMMAND_IN_BUFFER)
            if wait:
                self._wait_speed(axis, speed)
            return

        self._send_cmd(cmd=f"DPOS({axis})=0")
        self._set_axis_speed_and_accel(axis=axis, speed=speed, accel=accel, decel=accel)
        self._send_cmd(cmd=f"VMOVE({1 if direction else -1}) AXIS({axis})")
        if wait:
            self._wait_speed(axis, speed)

    def move_abs(
        self, axis: int, position: float, wait: bool, speed: float, accel: float, timeout_s: float = 10
    ) -> None:
        """Move the target axis to an absolute position.

        Move to a position:
        >>> self.move_abs(axis=1, position=1.0)
        >>> self.move_abs(axis=1, position=10.0)
        It will arrive to the absoluteed position 1.0 mm first, and then move position 10 mm.

        Parameters
        ----------
        axis: int
            The axis that need to move.
        position: float
            The absolute position. The unit is mm for linear motion (degree for rotation motion).
        wait: bool
            The application code waits for that it finishes the axis movement.
        speed: float
            The target speed. The unit is mm for linear motion (degree for rotation motion).
        accel: float
            The target acceleration and deceleration. The unit is mm for linear motion (degree for rotation motion).
        timeout_s: float
            If waits, the max time for waiting.
        """
        if axis not in self._config.axis_configs:
            raise ValueError(
                f"{self._card_name}: Just support to move the axes {self._config.axis_configs.keys()}, "
                f"not the target axis {axis}."
            )
        state = self._state.get_value()
        if state.axis_states[axis].is_finding_home:
            raise RuntimeError(
                f"{self._card_name}: The target axis {axis} is finding home, reject to move to {position}."
            )

        self._set_axis_speed_and_accel(axis=axis, speed=speed, accel=accel, decel=accel)
        self._send_cmd(
            cmd="MOVEABS"
            f"({-position if self._config.axis_configs[axis].need_invert_direction else position}) AXIS({axis})"
        )
        if wait:
            start_time = time.perf_counter()
            for latest_position in self.subscribe_position(axis=axis):
                if abs(position - latest_position) < ERROR_FOR_POSITION_EQUAL_MM:
                    logging.info(f"axis[{axis}] success, mpos = {latest_position}, target = {position}")
                    return
                if (time.perf_counter() - start_time) > timeout_s:
                    logging.info(f"axis[{axis}] fail, mpos = {latest_position}, target = {position}")
                    self.stop_axes(axes=[axis], mode=AxisStopMode.CANCEL_CURRENT_MOTION_AND_COMMAND_IN_BUFFER)
                    raise TimeoutError(
                        f"{self._card_name}: Axis {axis} plans to move to target position {position}_mm (or degree), "
                        f"but failed and arrived position {latest_position} with timeout {timeout_s}_s."
                    )

    def move_l(self, axes: Sequence[int], positions: Sequence[float], speed: float, accel: float, decel: float) -> None:
        """Move multiple axes to absolute position in linear interpolation motion.

        Examples
        --------
        An interpolation movement of axes (0, 1) to target positions (10.0, 10.0) with the last speed parameters:
        >>> self.move_l(axis=[0, 1], position=[10.0, 10.0])
        An interpolation movement of axes (0, 1) to target positions (10.0, 10.0) with specified speed parameters:
        >>> self.move_l(axis=[0, 1], position=[10.0, 10.0], speed=10.0, accel=100.0, decel=100.0)

        Parameters
        ----------
        axes: Sequence[int]
            The axes of motion that needs to be interpolated.
        positions: Sequence[float]
            The absolute target positions of the motion. The unit is mm for linear motion (degree for rotation motion).
        speed: float
            The speed of the interpolation motion. The unit is mm/s for linear motion (degree/s for rotation motion).
            Valid if speed > 0.
        accel: float
            The acceleration of the interpolation motion. The unit is mm/s^2 for linear motion (degree/s^2 for rotation
            motion). Valid if accel > 0.
        decel: float
            The deceleration of the interpolation motion. The unit is mm/s^2 for linear motion (degree/s^2 for rotation
            motion). Valid if accel > 0.
        """
        if len(axes) == 0:
            raise ValueError("The size of axes for command `move_l` is 0.")
        for axis in axes:
            if axis not in self._config.axis_configs:
                raise ValueError(
                    f"Just support to move the axes {self._config.axis_configs.keys()}, not the target axis {axis}."
                )
        if len(axes) != len(positions):
            raise ValueError(
                f"Invalid parameter for `move_l`. The size of axes: {len(axes)} != position: {len(positions)}."
            )
        state = self._state.get_value()
        for axis in axes:
            if state.axis_states[axis].is_finding_home:
                raise RuntimeError(f"{self._card_name}: The target axis {axis} is finding home, reject to move it.")

        target_positions = [
            -positions[idx] if self._config.axis_configs[axis].need_invert_direction else positions[idx]
            for idx, axis in enumerate(axes)
        ]
        target_positions_str = ", ".join([f"{position}" for position in target_positions])
        target_axes_str = ", ".join([f"{axis}" for axis in axes])

        self._send_cmd(cmd=f"BASE({target_axes_str})")
        for axis in axes:
            self._verify_speed_and_accel(axis=axis, speed=speed, accel=accel, decel=accel)
        self._send_cmd(cmd=f"SPEED={speed}")
        self._send_cmd(cmd=f"ACCEL={accel}")
        self._send_cmd(cmd=f"DECEL={decel}")
        self._send_cmd(cmd=f"MOVEABS({target_positions_str})")

    def emergency_stop(self) -> None:
        # Cancel the current motion and the motion command in buffer of all axes.
        self._send_cmd(cmd="RAPIDSTOP(2)")
        self.axes_disable(axes=[axis for axis in self._config.axis_configs.keys()])

    def stop_axes(self, axes: Sequence[int], mode: AxisStopMode) -> None:
        """Stop the axes motion.

        Parameters
        ----------
        axes: Sequence[int]
            The axes.
        mode: AxisStopMode
            The stop mode.
        """
        mode_str = f"{int(mode.value) - 1}"
        if mode_str not in ("0", "1", "2", "3"):
            raise ValueError(f"{self._card_name}: A invalid mode {mode}.")
        for axis in axes:
            if axis not in self._config.axis_configs:
                raise ValueError(
                    f"{self._card_name}: The target axis {axis} isn't in {self._config.axis_configs.keys()}"
                )
            self._send_cmd(cmd=f"CANCEL({mode_str}) AXIS({axis})")

    def find_homes(self, axes: Sequence[int] | None = None) -> None:
        """Command the axes to find the homes and set the biases.

        The axis runs in reverse at SPEED speed until it touches the origin switch. Then the axis moves forward at
        CREEP speed until it leaves the origin switch. It will run in opposite direction when it encounters the
        negative limit switch during the homing phase. Finally, it resets DPOS (the Demanded Position) value to 0 and
        correct MPOS (Measured Feedback Position) at the same time. It will stop directly when it touches the positive
        limit switch in the crawling stage.

        The MODE 14 of zmotion control card implement this. But if we need to invert the axis direction on software, we
        must use MODE 13 whose logic is contrary with MODE 14

        Parameters
        ----------
        axes: Optional[Sequence[int]]
            None: use all internal axes.
            Sequence[int]: the target axes.
        """
        target_axes = axes if (axes is not None) else [axis for axis in self._config.axis_configs.keys()]
        try:
            for idx, axis in enumerate(target_axes):
                if axis not in self._config.axis_configs:
                    raise ValueError(
                        f"{self._card_name}: The target axis {axis} isn't in {self._config.axis_configs.keys()}"
                    )
                axis_config = self._config.axis_configs[axis]
                if axis_config.no_limit_sensor:
                    raise ValueError(f"{self._card_name}: The target axis {axis} do not support find home")
                self._send_cmd(cmd=f"SPEED({axis})={axis_config.default_home_search_speed}")
                self._send_cmd(cmd=f"ACCEL({axis})={axis_config.default_home_search_accel}")
                self._send_cmd(cmd=f"DECEL({axis})={axis_config.default_home_search_decel}")
                self._send_cmd(cmd=f"CREEP({axis})={axis_config.default_home_creep_speed}")
                self._send_cmd(
                    cmd=f"DATUM({13 if self._config.axis_configs[axis].need_invert_direction else 14}) AXIS({axis})"
                )
            # the motion control card state in state subscribe has a delay (depending on the axes number,
            # 0.02s delay when monitor 4 axes), we should waite for it.
            time.sleep(1.0)
            self._wait_until_found_home(axes=[axis for axis in target_axes])
            time.sleep(0.5)
            for axis in target_axes:
                self._send_cmd(cmd=f"DPOS({axis})=0")
                self._send_cmd(cmd=f"MPOS({axis})=0")
        except Exception:
            self.stop_axes(axes=target_axes, mode=AxisStopMode.CANCEL_CURRENT_MOTION_AND_COMMAND_IN_BUFFER)
            raise

    def get_in(self, reg: int) -> bool:
        if self._config.monitor_di_min_address < reg < self._config.monitor_di_max_address:
            self._has_latest_state.wait()
            card_state = self._state.get_value()
            return card_state.digital_inputs[reg]
        return True if int(self._send_cmd(cmd=f"?IN({reg})")) else False

    def set_outs(self, target_values: dict[int, bool]) -> None:
        for target, value in target_values.items():
            if target in self._axis_enable_do_addresses:
                raise ValueError(
                    f"{self._card_name}: Failed to set the value of the output {target}. You can't set these digital"
                    f"outputs {self._axis_enable_do_addresses} , which are used for enable/disable the axis motor."
                )
        self._set_outs(target_values)

    def get_out(self, reg: int) -> bool:
        if self._config.monitor_do_min_address < reg < self._config.monitor_do_max_address:
            self._has_latest_state.wait()
            card_state = self._state.get_value()
            return card_state.digital_outputs[reg]
        return True if int(self._send_cmd(cmd=f"?OP({reg})")) else False

    def set_ip(self, ip_address: str) -> None:
        if len(ip_address.split(".")) != 4:
            raise NameError(f"Can't set to the ip {ip_address}, which has a error length, ")
        try:
            self._send_cmd(cmd=f"IP_ADDRESS={ip_address}", cmd_interface=_SendCommandInterface.ZMC_DIRECT)
        except ConnectionError:
            raise ConnectionResetError(
                f"Success to set the ip address to {ip_address}, please reboot the motion control card."
            )
