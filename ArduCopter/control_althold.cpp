#include "Copter.h"

double Input_front_right, Input_front, Input_front_left, Output, Input2;
double Kp=0.15, Ki=0.1, Kd=0.004, Setpoint;
//long previousMillis = 0;
//long interval = 70;
#define TEST_P_front 8.0f    // default was 2.0
#define TEST_I_front 0.05f   // default was 0.05
#define TEST_D_front 3.0f    // default was 4.5

long previousMillis = 0;
long interval = 70;

#define TEST_IMAX 1
#define TEST_FILTER 5.0f
#define TEST_DT 0.01f
#define TEST_INITIAL_FF 0.0f

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::althold_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::althold_run()
{
    AltHoldModeState althold_state;
    long currentMillis = AP_HAL::millis();
   RC_Channel *rc12 = RC_Channels::rc_channel(CH_12);
    	RC_Channel *rc14 = RC_Channels::rc_channel(CH_14);
    	int16_t error_front;
    AC_PID pid(TEST_P_front, TEST_I_front, TEST_D_front, TEST_IMAX * 100, TEST_FILTER, TEST_DT);



    	float control_P_front, control_I_front, control_D_front;
    rangefinder.update();
    Input_front = rangefinder.distance_cm(1);
    Input_front_right = rangefinder.distance_cm(0);
    Input_front_left = rangefinder.distance_cm(2);
    Input_front = double(Input_front);
    Input_front_left = double(Input_front_left);
  //  long currentMillis = AP_HAL::millis();
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        uint16_t radio12_in = rc12->get_radio_in();
        uint16_t radio14_in = rc14->get_radio_in();



        if(radio14_in == 2084 && Input_front_right <= 150.0f){



     //   	Setpoint = (radio12_in - 1094) * (800.0f - 200.0f) / (1934 - 1094) + 200.0f;


        			  error_front =  Input_front_right - 150.0f;
        	           pid.set_input_filter_all(error_front);
        	           control_P_front = pid.get_p();
        	           control_I_front = pid.get_i();
        	           control_D_front = pid.get_d();




        	float output_pid_front = -(control_P_front + control_I_front + control_D_front);
        	output_pid_front = constrain_float(output_pid_front,-600.0f,600.0f);

        			attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, output_pid_front, target_yaw_rate, get_smoothing_gain());
        			//attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
        			// adjust climb rate using rangefinder


        			        	        // get avoidance adjusted climb rate
        			        	        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        			        	        if(Input_front_right <= 200.0f){

        			        	        	pos_control->set_alt_target_from_climb_rate_ff(20.0f, G_Dt, false);
        			        	        	pos_control->update_z_controller();

        			        	        } else{

        			        	        	pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        			        	        pos_control->update_z_controller();

        			        	        }

        			        	        // call position controller

        			        	  //      pos_control->set_xy_target(200.0f,0.0f);
        			        	   //     pos_control->stick_to_wall(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler, Input_front , false);
 			if(currentMillis - previousMillis > interval){
        				  previousMillis = currentMillis;
        				  hal.console->printf("target climb rate: %f \n", Input_front_right);
        			//	  hal.console->printf("range finder front right: %f \n", Input_front_right);
        			//	  hal.console->printf("range finder front left: %f \n", Input_front_left);
        			//	hal.console->printf("xtarget: %f \n", output_pid_front);
        			//	  hal.console->printf("altitude is: %f \n", Input_front);
        					}
        			        	        break;














        }else{

        	// call attitude controller
        	        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        	        // adjust climb rate using rangefinder
        	        if (rangefinder_alt_ok()) {
        	            // if rangefinder is ok, use surface tracking
        	            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        	        }

        	        // get avoidance adjusted climb rate
        	        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        	        // call position controller
        	        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        	        pos_control->update_z_controller();
        	        break;

   }




    }
}
