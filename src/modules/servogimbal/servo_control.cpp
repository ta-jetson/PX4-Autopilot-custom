#include "servo_control.h"

int ServoControl::run()
{
    px4_pollfd_struct_t fds{};
    int sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    fds.fd = sub;
    fds.events = POLLIN;

    PX4_INFO("Servo Control Module started");

    while (!should_exit()) {
        int ret = px4_poll(&fds, 1, 100);

        if (ret > 0 && (fds.revents & POLLIN)) {
            manual_control_setpoint_s ctrl{};
            orb_copy(ORB_ID(manual_control_setpoint), sub, &ctrl);

            bool button = ctrl.buttons & (1 << 0);  // Button 0

            if (button && !_prev_button) { // rising edge
                _pwm_norm += 0.05f;
                if (_pwm_norm > 1.0f) _pwm_norm = 1.0f;

                PX4_INFO("PWM increased to %.2f", _pwm_norm);
                send_pwm_value(_pwm_norm);
            }

            _prev_button = button;
        }
    }

    return 0;
}

void ServoControl::send_pwm_value(float value)
{
    actuator_controls_s act{};
    act.timestamp = hrt_absolute_time();
    act.control[5] = value; // Index 5 = AUX5
    orb_advert_t pub = orb_advertise(ORB_ID(actuator_controls_0), &act);
}
