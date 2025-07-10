#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>

class ServoControl : public ModuleBase<ServoControl>
{
public:
    int run() override;

private:
    float _pwm_norm = 0.0f;  // normalized PWM value [0.0 to 1.0]
    bool _prev_button = false;  // for edge detection

    void send_pwm_value(float value);
};

int ServoControl::run()
{
    px4_pollfd_struct_t fds{};
    int sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    fds.fd = sub;
    fds.events = POLLIN;

    PX4_INFO("Servo control started");

    while (!should_exit()) {
        int ret = px4_poll(&fds, 1, 100);

        if (ret > 0 && (fds.revents & POLLIN)) {
            manual_control_setpoint_s ctrl{};
            orb_copy(ORB_ID(manual_control_setpoint), sub, &ctrl);

            // Button 1 pressed? (Change to button index as needed)
            bool button = ctrl.buttons & (1 << 0);  // button 0

            if (button && !_prev_button) {
                // rising edge detected
                _pwm_norm += 0.05f;  // increment by 5%
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
    act.control[5] = value; // AUX5 corresponds to actuator_controls_0[5]
    orb_advert_t pub = orb_advertise(ORB_ID(actuator_controls_0), &act);
}
