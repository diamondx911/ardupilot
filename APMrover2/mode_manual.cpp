#include "mode.h"
#include "Rover.h"



RC_Channel *rc7 = RC_Channels::rc_channel(CH_7);
float simple_cos_yaw;
float simple_sin_yaw;
void ModeManual::update()
{
    // check for radio failsafe
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
    } else {
    	        // copy RC scaled inputs to outputs

    	uint16_t radio7_in = rc7->get_radio_in();

    	if(radio7_in == 2084){

    	    simple_cos_yaw = ahrs.cos_yaw();
    	    simple_sin_yaw = ahrs.sin_yaw();

            g2.motors.set_throttle(channel_throttle->get_control_in());
            g2.motors.set_steering(channel_steer->get_control_in()*simple_cos_yaw);
    	} else {
    	        g2.motors.set_throttle(channel_throttle->get_control_in());
    	        g2.motors.set_steering(channel_steer->get_control_in());
    			}



    }


    // mark us as in_reverse when using a negative throttle to stop AHRS getting off
    rover.set_reverse(is_negative(g2.motors.get_throttle()));
}
