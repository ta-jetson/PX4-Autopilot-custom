#pragma once

#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <drivers/drv_hrt.h>

class ServoControl : public px4::ModuleBase<ServoControl>
{
public:
    ServoControl() = default;
    ~ServoControl() override = default;

    static int task_spawn(int argc, char *argv[]) { return ModuleBase::task_spawn(argc, argv); }
    static ServoControl *instantiate(int argc, char *argv[]) { return new ServoControl(); }

    static int custom_command(int argc, char *argv[]) { return print_usage("No custom command."); }

    static int print_usage(const char *reason = nullptr)
    {
        if (reason) {
            PX4_WARN("%s", reason);
        }
        PRINT_MODULE_DESCRIPTION("Servo Control via Joystick Button");
        PRINT_MODULE_USAGE_NAME("servo_control", "module");
        return 0;
    }

    int run() override;

private:
    float _pwm_norm = 0.0f;
    bool _prev_button = false;

    void send_pwm_value(float value);
};
