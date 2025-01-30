import logging
from pyrsistent import pmap, pvector

from usage.data_types import (
    AxisConfig,
    MotionControlCardConfig,
)
from usage.zmotion_control_card import ZMotionControlCard


def _get_example_config() -> MotionControlCardConfig:
    return MotionControlCardConfig(
        ip_address="192.168.0.11",
        axis_configs=pmap(
            {
                0: AxisConfig(
                    axis_name=0,
                    max_pulse_frequency=6000000,
                    units=1000,
                    pos_limit=(-200.0, 2300),
                    max_speed=6000,
                    max_accel=15000,
                    default_home_search_speed=400,
                    default_home_search_accel=400,
                    default_home_search_decel=3000,
                    default_home_creep_speed=100,
                    home_di_address=16,
                    reverse_limit_di_address=20,
                    forward_limit_di_address=21,
                    need_invert_direction=True,
                    enable_second_after_first_power_on=7,
                    enable_do_address=16,
                    error_di_address=40,
                ),
                1: AxisConfig(
                    axis_name=1,
                    max_pulse_frequency=6000000,
                    units=1000,
                    pos_limit=(-200.0, 2300),
                    max_speed=6000,
                    max_accel=15000,
                    default_home_search_speed=400,
                    default_home_search_accel=400,
                    default_home_search_decel=3000,
                    default_home_creep_speed=100,
                    home_di_address=17,
                    reverse_limit_di_address=22,
                    forward_limit_di_address=23,
                    need_invert_direction=True,
                    enable_second_after_first_power_on=7,
                    enable_do_address=18,
                    error_di_address=41,
                ),
                2: AxisConfig(
                    axis_name=2,
                    max_pulse_frequency=6000000,
                    units=1000,
                    pos_limit=(-200.0, 2300),
                    max_speed=6000,
                    max_accel=15000,
                    default_home_search_speed=400,
                    default_home_search_accel=400,
                    default_home_search_decel=3000,
                    default_home_creep_speed=100,
                    home_di_address=18,
                    reverse_limit_di_address=24,
                    forward_limit_di_address=25,
                    need_invert_direction=True,
                    enable_second_after_first_power_on=7,
                    enable_do_address=20,
                    error_di_address=42,
                ),
                3: AxisConfig(
                    axis_name=3,
                    max_pulse_frequency=6000000,
                    units=1000,
                    pos_limit=(-200.0, 2300),
                    max_speed=6000,
                    max_accel=15000,
                    default_home_search_speed=400,
                    default_home_search_accel=400,
                    default_home_search_decel=3000,
                    default_home_creep_speed=100,
                    home_di_address=19,
                    reverse_limit_di_address=26,
                    forward_limit_di_address=27,
                    need_invert_direction=True,
                    enable_second_after_first_power_on=7,
                    enable_do_address=22,
                    error_di_address=43,
                ),
                4: AxisConfig(
                    axis_name=4,
                    max_pulse_frequency=4000000,
                    units=100,
                    pos_limit=(-200.0, 2300),
                    max_speed=2200,
                    max_accel=6000,
                    default_home_search_speed=150,
                    default_home_search_accel=150,
                    default_home_search_decel=750,
                    default_home_creep_speed=50,
                    home_di_address=28,
                    reverse_limit_di_address=33,
                    forward_limit_di_address=32,
                    need_invert_direction=False,
                    enable_second_after_first_power_on=0,
                    enable_do_address=24,
                    error_di_address=44,
                ),
            }
        ),
        invert_di_addresses=pvector([40, 41, 42, 43, 44, *[num for num in range(16, 28)]]),
        emergency_stop_di_address=0,
        monitor_di_min_address=0,
        monitor_di_max_address=63,
        monitor_do_min_address=0,
        monitor_do_max_address=31,
    )


if __name__ == "__main__":
    rob = ZMotionControlCard(config=_get_example_config())
    logging.info(f"Current position: {rob.get_position(axis=0)}")
    # rob.set_ip(ip_address="192.168.10.114")
    test_axis = 0
    rob.axes_enable(axes=[test_axis])
    try:
        # ########################################################
        # # USE THIS BLOCK TO OPERATE IO AND READ POSITION
        logging.info(f"The input of gpio 0 is : {rob.get_in(1)}")
        # rob.set_outs({5: True})
        # print(f"The output of gpio 5 is : {rob.get_in(5)}")
        logging.info(f"Current position of axis {test_axis}: {rob.get_position(axis=test_axis)}")
        # ########################################################
        # ########################################################
        # # USE THIS BLOCK TO FIND THE HOME
        rob.find_homes(axes=[test_axis])
        logging.info("Finished finding home.")
        rob.move_abs(axis=test_axis, position=-50.5, speed=50, accel=100, wait=True)
        # ########################################################
        # ########################################################
        # # USE THIS BLOCK TO MOVE
        while True:
            rob.move_abs(axis=test_axis, position=10, speed=50, accel=100, wait=True)
            print(f"Axis position: {rob.get_position(axis=test_axis)}")
            rob.move_abs(axis=test_axis, position=20, speed=50, accel=100, wait=True)
            print(f"Axis position: {rob.get_position(axis=test_axis)}")
        # ########################################################
    except KeyboardInterrupt:
        pass
    finally:
        logging.info("Finished the running.")
        rob.emergency_stop()
