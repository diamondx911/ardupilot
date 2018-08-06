#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_LightWareSerial.h>
#include <AC_PID/PID_v1.h>
#include <AC_PID/AC_PID.h>
#include <AP_Motors/AP_Motors_class.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_Scheduler/AP_Scheduler.h>


// ============================ADDED CODE BY ALI==============================
double Input, Output, Input2;
double Kp=0.15, Ki=0.1, Kd=0.004, Setpoint;
long previousMillis = 0;
long interval = 70;
#define TEST_P 8.0f    // default was 9.0
#define TEST_I 0.05f   // default was 0.05
#define TEST_D 5.5f    // default was 1.2
#define TEST_IMAX 1
#define TEST_FILTER 5.0f
#define TEST_DT 0.01f
#define TEST_INITIAL_FF 0.0f



extern const AP_HAL::HAL& hal;
static AP_SerialManager serial_manager;
//static RangeFinder sonar {serial_manager, ROTATION_PITCH_270};

//const AP_HAL::HAL& hal = AP_HAL::get_HAL();




//============================================================================



/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::stabilize_init(bool ignore_checks)
{


   //if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
      (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
    return false;
    }
    //set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;

    // read sonar data ================ ALI ADDED CODE ========================

    // print welcome message
    //   hal.console->printf("Range Finder library test\n");
    //   hal.rcout->set_freq(0xF, 490);
       // ========================================================================
   //hal.rcout->enable_ch(0);
   // hal.rcout->enable_ch(1);
//    hal.rcout->enable_ch(2);
  //  hal.rcout->enable_ch(3);
    //hal.rcout->write(3, control_P+control_I+control_D);
    //hal.rcout->cork();

  //  AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_PLI2C);
   //     AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", -1.0f);
   //     AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 1.0f);
 //   sonar.init();

}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more

void Copter::stabilize_run()
{
AC_PID pid(TEST_P, TEST_I, TEST_D, TEST_IMAX * 100, TEST_FILTER, TEST_DT);
	int16_t error;
   Setpoint = 200;


	float control_P, control_I, control_D;
Input = rangefinder.distance_cm(0);
 Input2 = double(Input);

            error =  Input2 - Setpoint;
            pid.set_input_filter_all(error);
            control_P = pid.get_p();
            control_I = pid.get_i();
            control_D = pid.get_d();
float output_pid = -(control_P + control_I + control_D);
output_pid = constrain_float(output_pid,-500.0f,500.0f);

	PID myPID(&Input2, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
myPID.SetMode(AUTOMATIC);
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;




    //float target_climb_rate = 0.0f;

    //if not armed set throttle to zero and exit immediately
   if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
      motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
      attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);


  return;
   }

   //  clear landing flag
    set_land_complete(false);


    rangefinder.update();



 Input2 = rangefinder.distance_cm(1);
    //Input = rangefinder.distance_cm_orient(ROTATION_PITCH_270);



 //Input = sonar.distance_cm(1);

/**

        AC_HELI_PID heli_pid(TEST_P, TEST_I, TEST_D, TEST_IMAX * 100, TEST_FILTER, TEST_DT, TEST_INITIAL_FF);
        uint16_t radio_in;
        uint16_t radio_trim;

**/
    myPID.Compute();



    // ========================================================================

 motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

   //motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
 //   hal.rcout->write(0, 1500);

// hal.rcout->write(0, 1500);
  //  hal.rcout->write(2, 1500);
//    hal.rcout->write(1, 1500);


    long currentMillis = AP_HAL::millis();

//Input = get_auto_heading();



    // apply SIMPLE mode transform to pilot inputs
   update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
   // int rollx = channel_pitch->get_control_in();
    //int yawx = channel_yaw->get_control_in();
   // float xaccel = ins.get_accel().x;
   // int test_roll;
   // int test_yaw;
   // int roll;
   // int yaw;
   // int head;

   // head = get_auto_heading();
/**
    if (rollx > 0){
    		test_roll = (rollx - (0)) * (2000 - 1000) / (4500 - (0)) + 1000;
    		rollx = get_pilot_desired_throttle(test_roll);
    		    		attitude_control->set_throttle_out(rollx, true, g.throttle_filt);


    	//hal.rcout->write(2, test_roll);

    	if(currentMillis - previousMillis > interval){
    	    		    previousMillis = currentMillis;
    	    		    hal.console->printf("rollx: %d \n", test_roll);
    	    		    }
    }
    	else if(rollx < 0){
    		test_roll = - rollx;
    		roll = (test_roll - (0)) * (2000 - 1250) / (4500 - (0)) + 1250;
    		//hal.rcout->cork();
    		hal.rcout->write(0, roll);
    		if(currentMillis - previousMillis > interval){
    		    previousMillis = currentMillis;
    		    hal.console->printf("rollx: %d \n", roll);
    		    }
    		else if(rollx == 0) {

    		if(currentMillis - previousMillis > interval){
    		    	    		    previousMillis = currentMillis;
    		    	    		    hal.console->printf("rollx: %d \n", rollx);
    		    	    		    }

    		}

    	}

    if (yawx > 0){
       		test_yaw = (yawx - (0)) * (2000 - 1250) / (4500 - (0)) + 1250;
       	hal.rcout->write(1, test_yaw);
       	if(currentMillis - previousMillis > interval){
       	    		    previousMillis = currentMillis;
       	    		    hal.console->printf("rollx: %d \n", test_yaw);
       	    		    }
       }
       	else if(yawx < 0){
       		test_yaw = - yawx;
       		yaw = (test_yaw - (0)) * (2000 - 1250) / (4500 - (0)) + 1250;
       		//hal.rcout->cork();
       		hal.rcout->write(3,yaw);
       		if(currentMillis - previousMillis > interval){
       		    previousMillis = currentMillis;

       		    }
       		else if(rollx == 0) {

       		if(currentMillis - previousMillis > interval){
       		    	    		    previousMillis = currentMillis;

       		    	    		    }

       		}
       	}


**/



    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
   pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

//float target_pitch_before = target_pitch;

//if(Input <= 300){
	//double coeff = (1 - (Input / 200)) * 1300;

//	target_pitch = float(coeff) - target_pitch;
//	avoid.adjust_roll_pitch(target_roll,target_pitch,aparm.angle_max);
	target_pitch = constrain_float(target_pitch,-4500.0f,4500.0f);
	    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, output_pid, target_yaw_rate, get_smoothing_gain());
	    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
	   		if(currentMillis - previousMillis > interval){
	    previousMillis = currentMillis;
	    hal.console->printf("distance up 0: %f \n", Input);
	    hal.console->printf("Output pitch: %f \n", Input);

	}


//}




    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle

//attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
 //  attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

    if(currentMillis - previousMillis > interval){
    previousMillis = currentMillis;
    hal.console->printf("Output pitch: %f \n", Output);
  // hal.console->printf("dustance: %f \n", Input2);

    }
}
