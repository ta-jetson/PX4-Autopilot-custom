px4_add_module(
    MODULE servo_control
    MAIN servo_control
    SRCS
        servo_control.cpp
    HEADERS
        servo_control.h
    DEPENDS
        hysteresis
        mathlib
        px4_work_queue
)
