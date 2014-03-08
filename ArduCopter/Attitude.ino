/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


// prototype this for use inside the GCS class
void gcs_send_text_fmt(const prog_char_t *fmt, ...);

static void
get_stabilize_roll(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180(target_angle - ahrs.roll_sensor);

    // limit the error we're feeding to the PID
    target_angle            = constrain(target_angle, -4500, 4500);

        // convert to desired Rate:
    int32_t target_rate = g.pi_stabilize_roll.get_p(target_angle);

    int16_t i_stab;
    if(labs(ahrs.roll_sensor) < 500) {
        target_angle            = constrain(target_angle, -500, 500);
        i_stab                          = g.pi_stabilize_roll.get_i(target_angle, G_Dt);
    }else{
        i_stab                          = g.pi_stabilize_roll.get_integrator();
    }

    // set targets for rate controller
    set_roll_rate_target(target_rate+i_stab, EARTH_FRAME);
}

static void
get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180(target_angle - ahrs.pitch_sensor);

    // limit the error we're feeding to the PID
    target_angle            = constrain(target_angle, -4500, 4500);

    // convert to desired Rate:
    int32_t target_rate = g.pi_stabilize_pitch.get_p(target_angle);

    int16_t i_stab;
    if(labs(ahrs.pitch_sensor) < 500) {
        target_angle            = constrain(target_angle, -500, 500);
        i_stab                          = g.pi_stabilize_pitch.get_i(target_angle, G_Dt);
    }else{
        i_stab                          = g.pi_stabilize_pitch.get_integrator();
    }

    // set targets for rate controller
    set_pitch_rate_target(target_rate + i_stab, EARTH_FRAME);
}

static void
get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate,i_term;
    int32_t angle_error;
    int32_t output = 0;

    // angle error
    angle_error             = wrap_180(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID

    angle_error             = constrain(angle_error, -4500, 4500);

    // convert angle error to desired Rate:
    target_rate = g.pi_stabilize_yaw.get_p(angle_error);
    i_term = g.pi_stabilize_yaw.get_i(angle_error, G_Dt);

    // do not use rate controllers for helicotpers with external gyros
#if FRAME_CONFIG == HELI_FRAME
    if(motors.ext_gyro_enabled) {
        g.rc_4.servo_out = constrain((target_rate + i_term), -4500, 4500);
    }
#endif

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && g.radio_tuning == CH6_YAW_KP ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_KP, angle_error, target_rate, i_term, 0, output, tuning_value);
        }
    }
#endif

    // set targets for rate controller
    set_yaw_rate_target(target_rate+i_term, EARTH_FRAME);
}

static void
get_acro_roll(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_roll_rate_target(target_rate, BODY_FRAME);
}

static void
get_acro_pitch(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_pitch_rate_target(target_rate, BODY_FRAME);
}

static void
get_acro_yaw(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_yaw_rate_target(target_rate, BODY_FRAME);
}

// Roll with rate input and stabilized in the earth frame
static void
get_roll_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired roll rate
    int32_t target_rate = stick_angle * g.acro_p - (roll_axis * g.acro_balance_roll)/100;

    // convert the input to the desired roll rate
    roll_axis += target_rate * G_Dt;
    roll_axis = wrap_180(roll_axis);

    // ensure that we don't reach gimbal lock
    if (labs(roll_axis > 4500) && g.acro_trainer_enabled) {
        roll_axis	= constrain(roll_axis, -4500, 4500);
        angle_error = wrap_180(roll_axis - ahrs.roll_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180(roll_axis - ahrs.roll_sensor);
        angle_error	= constrain(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);
    }

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe)) {
        angle_error = 0;
    }

    // update roll_axis to be within max_angle_overshoot of our current heading
    roll_axis = wrap_180(angle_error + ahrs.roll_sensor);

    // set earth frame targets for rate controller

    // set earth frame targets for rate controller
	set_roll_rate_target(g.pi_stabilize_roll.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Pitch with rate input and stabilized in the earth frame
static void
get_pitch_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired pitch rate
    int32_t target_rate = stick_angle * g.acro_p - (pitch_axis * g.acro_balance_pitch)/100;

    // convert the input to the desired pitch rate
    pitch_axis += target_rate * G_Dt;
    pitch_axis = wrap_180(pitch_axis);

    // ensure that we don't reach gimbal lock
    if (labs(pitch_axis) > 4500) {
        pitch_axis	= constrain(pitch_axis, -4500, 4500);
        angle_error = wrap_180(pitch_axis - ahrs.pitch_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180(pitch_axis - ahrs.pitch_sensor);
        angle_error	= constrain(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);
    }

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe)) {
        angle_error = 0;
    }

    // update pitch_axis to be within max_angle_overshoot of our current heading
    pitch_axis = wrap_180(angle_error + ahrs.pitch_sensor);

    // set earth frame targets for rate controller
	set_pitch_rate_target(g.pi_stabilize_pitch.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Yaw with rate input and stabilized in the earth frame
static void
get_yaw_rate_stabilized_ef(int32_t stick_angle)
{

    int32_t angle_error = 0;

    // convert the input to the desired yaw rate
    int32_t target_rate = stick_angle * g.acro_p;

    // convert the input to the desired yaw rate
    nav_yaw += target_rate * G_Dt;
    nav_yaw = wrap_360(nav_yaw);

    // calculate difference between desired heading and current heading
    angle_error = wrap_180(nav_yaw - ahrs.yaw_sensor);

    // limit the maximum overshoot
    angle_error	= constrain(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe)) {
    	angle_error = 0;
    }

    // update nav_yaw to be within max_angle_overshoot of our current heading
    nav_yaw = wrap_360(angle_error + ahrs.yaw_sensor);

    // set earth frame targets for rate controller
	set_yaw_rate_target(g.pi_stabilize_yaw.get_p(angle_error)+target_rate, EARTH_FRAME);
}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        roll_rate_target_bf = desired_rate;
    }else{
        roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        pitch_rate_target_bf = desired_rate;
    }else{
        pitch_rate_target_ef = desired_rate;
    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        yaw_rate_target_bf = desired_rate;
    }else{
        yaw_rate_target_ef = desired_rate;
    }
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
update_rate_contoller_targets()
{
    if( rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        roll_rate_target_bf 	= roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        pitch_rate_target_bf 	= cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        yaw_rate_target_bf 		= cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
#if FRAME_CONFIG == HELI_FRAME          // helicopters only use rate controllers for yaw and only when not using an external gyro
    if(!motors.ext_gyro_enabled) {
		g.rc_1.servo_out = get_heli_rate_roll(roll_rate_target_bf);
		g.rc_2.servo_out = get_heli_rate_pitch(pitch_rate_target_bf);
        g.rc_4.servo_out = get_heli_rate_yaw(yaw_rate_target_bf);
    }
#else
    // call rate controllers
    g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
    g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);
#endif

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( g.throttle_accel_enabled && throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
    }
}

#if FRAME_CONFIG == HELI_FRAME
// init_rate_controllers - set-up filters for rate controller inputs
void init_rate_controllers()
{
   // initalise low pass filters on rate controller inputs
   // 1st parameter is time_step, 2nd parameter is time_constant
   rate_roll_filter.set_cutoff_frequency(0.01, 2.0);
   rate_pitch_filter.set_cutoff_frequency(0.01, 2.0);
   // rate_yaw_filter.set_cutoff_frequency(0.01, 2.0);
   // other option for initialisation is rate_roll_filter.set_cutoff_frequency(<time_step>,<cutoff_freq>);
}

static int16_t
get_heli_rate_roll(int32_t target_rate)
{
    int32_t p,i,d,ff;                // used to capture pid values for logging
	int32_t current_rate;			// this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

	// get current rate
	current_rate    = (omega.x * DEGX100);

    // filter input
    current_rate = rate_roll_filter.apply(current_rate);
	
    // call pid controller
    rate_error  = target_rate - current_rate;
    p   = g.pid_rate_roll.get_p(rate_error);

    if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_roll.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_roll.get_integrator();
		}
	} else {
		i			= g.pid_rate_roll.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);		// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_roll.get_d(rate_error, G_Dt);
	
	ff = g.heli_roll_ff * target_rate;

    output = p + i + d + ff;

    // constrain output
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                     			// this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

	// get current rate
	current_rate    = (omega.y * DEGX100);
	
	// filter input
	current_rate = rate_pitch_filter.apply(current_rate);
	
	// call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);						// Helicopters get huge feed-forward
	
	if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_pitch.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_pitch.get_integrator();
		}
	} else {
		i			= g.pid_rate_pitch.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);	// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
	
	ff = g.heli_pitch_ff*target_rate;
    
    output = p + i + d + ff;

    // constrain output
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_heli_rate_roll to update pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, 0, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                               // this iteration's rate
    int32_t rate_error;
    int32_t output;

	// get current rate
    current_rate    = (omega.z * DEGX100);
	
	// filter input
	// current_rate = rate_yaw_filter.apply(current_rate);
	
    // rate control
    rate_error              = target_rate - current_rate;

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    
    i = g.pid_rate_yaw.get_i(rate_error, G_Dt);

    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);
	
	ff = g.heli_yaw_ff*target_rate;

    output  = p + i + d + ff;
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_RATE_KP || g.radio_tuning == CH6_YAW_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
	
#endif

	// output control
	return output;
}
#endif // HELI_FRAME

#if FRAME_CONFIG != HELI_FRAME
static int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (omega.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = g.pid_rate_roll.get_p(rate_error);

    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i	= g.pid_rate_roll.get_integrator();
    }else{
        i   = g.pid_rate_roll.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;                          // Note: get_rate_pitch pid logging relies on this function to update pid_log_counter so if you change the line above you must change the equivalent line in get_rate_pitch
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (omega.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);
    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i = g.pid_rate_pitch.get_integrator();
    }else{
        i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
    }
    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_rate_roll having updated pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (omega.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    // freeze I term if we've breached yaw limits
    if( motors.reached_limit(AP_MOTOR_YAW_LIMIT) ) {
        i = g.pid_rate_yaw.get_integrator();
    }else{
        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
    }
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && g.radio_tuning == CH6_YAW_RATE_KP ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

#if FRAME_CONFIG == TRI_FRAME
    // constrain output
    return output;
#else // !TRI_FRAME
    // output control:
    int16_t yaw_limit = 2200 + abs(g.rc_4.control_in);

    // smoother Yaw control:
    return constrain(output, -yaw_limit, yaw_limit);
#endif // TRI_FRAME
}
#endif // !HELI_FRAME

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t input_roll)
{
#if OPTFLOW == ENABLED
    static float tot_x_cm = 0;      // total distance from target
    static uint32_t last_of_roll_update = 0;
    int32_t new_roll = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_roll_update) {
        last_of_roll_update = optflow.last_update;

        // add new distance moved
        tot_x_cm += optflow.x_cm;

        // only stop roll if caller isn't modifying roll
        if( input_roll == 0 && current_loc.alt < 1500) {
            p = g.pid_optflow_roll.get_p(-tot_x_cm);
            i = g.pid_optflow_roll.get_i(-tot_x_cm,1.0);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_roll.get_d(-tot_x_cm,1.0);
            new_roll = p+i+d;
        }else{
            g.pid_optflow_roll.reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        // limit amount of change and maximum angle
        of_roll = constrain(new_roll, (of_roll-20), (of_roll+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            pid_log_counter++;              // Note: get_of_pitch pid logging relies on this function updating pid_log_counter so if you change the line above you must change the equivalent line in get_of_pitch
            if( pid_log_counter >= 5 ) {    // (update rate / desired output rate) = (100hz / 10hz) = 10
                pid_log_counter = 0;
                Log_Write_PID(CH6_OPTFLOW_KP, tot_x_cm, p, i, d, of_roll, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_roll = constrain(of_roll, -1000, 1000);

    return input_roll+of_roll;
#else
    return input_roll;
#endif
}

static int32_t
get_of_pitch(int32_t input_pitch)
{
#if OPTFLOW == ENABLED
    static float tot_y_cm = 0;  // total distance from target
    static uint32_t last_of_pitch_update = 0;
    int32_t new_pitch = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_pitch_update ) {
        last_of_pitch_update = optflow.last_update;

        // add new distance moved
        tot_y_cm += optflow.y_cm;

        // only stop roll if caller isn't modifying pitch
        if( input_pitch == 0 && current_loc.alt < 1500 ) {
            p = g.pid_optflow_pitch.get_p(tot_y_cm);
            i = g.pid_optflow_pitch.get_i(tot_y_cm, 1.0);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_pitch.get_d(tot_y_cm, 1.0);
            new_pitch = p + i + d;
        }else{
            tot_y_cm = 0;
            g.pid_optflow_pitch.reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain(new_pitch, (of_pitch-20), (of_pitch+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            if( pid_log_counter == 0 ) {        // relies on get_of_roll having updated the pid_log_counter
                Log_Write_PID(CH6_OPTFLOW_KP+100, tot_y_cm, p, i, d, of_pitch, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_pitch = constrain(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
#else
    return input_pitch;
#endif
}

/*************************************************************
 * yaw controllers
 *************************************************************/

static void get_look_ahead_yaw(int16_t pilot_yaw)
{
    // Commanded Yaw to automatically look ahead.
    if (g_gps->fix && g_gps->ground_course > YAW_LOOK_AHEAD_MIN_SPEED) {
        nav_yaw = get_yaw_slew(nav_yaw, g_gps->ground_course, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360(nav_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    }else{
        nav_yaw += pilot_yaw * g.acro_p * G_Dt;
        nav_yaw = wrap_360(nav_yaw);
        get_stabilize_yaw(nav_yaw);
    }
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
static void update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = g.throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * .99 + (float)throttle * .01;
        g.throttle_cruise = throttle_avg;
    }
}

#if FRAME_CONFIG == HELI_FRAME
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
// for traditional helicopters
static int16_t get_angle_boost(int16_t throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
    angle_boost_factor = 1.0 - constrain(angle_boost_factor, .5, 1.0);
    int16_t throttle_above_mid = max(throttle - motors.throttle_mid,0);

    // to allow logging of angle boost
    angle_boost = throttle_above_mid*angle_boost_factor;

    return throttle + angle_boost;
}
#else   // all multicopters
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
static int16_t get_angle_boost(int16_t throttle)
{
    float temp = cos_pitch_x * cos_roll_x;
    int16_t throttle_out;

    temp = constrain(temp, .5, 1.0);
    temp = constrain(9000-max(labs(roll_axis),labs(pitch_axis)), 0, 3000) / (3000 * temp);
    throttle_out = constrain((float)(throttle-g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);
    //Serial.printf("Thin:%4.2f  sincos:%4.2f  temp:%4.2f  roll_axis:%4.2f  Out:%4.2f   \n", 1.0*throttle, 1.0*cos_pitch_x * cos_roll_x, 1.0*temp, 1.0*roll_axis, 1.0*constrain((float)value * temp, 0, 1000));

    // to allow logging of angle boost
    angle_boost = throttle_out - throttle;

    return throttle_out;
}
#endif // FRAME_CONFIG == HELI_FRAME

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        g.rc_3.servo_out = get_angle_boost(throttle_out);
    }else{
        g.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        angle_boost = 0;
    }
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
void set_throttle_accel_target( int16_t desired_acceleration )
{
    if( g.throttle_accel_enabled ) {
        throttle_accel_target_ef = desired_acceleration;
        throttle_accel_controller_active = true;
    }else{
        // To-Do log dataflash or tlog error
        cliSerial->print_P(PSTR("Err: target sent to inactive acc thr controller!\n"));
    }
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
void throttle_accel_deactivate()
{
    throttle_accel_controller_active = false;
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
static int16_t
get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(ahrs.get_accel_ef().z + gravity) * 100;

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_accel_error = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        z_accel_error = z_accel_error + 0.11164 * (constrain(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle_accel.get_p(z_accel_error);
    // freeze I term if we've breached throttle limits
    if( motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT) ) {
        i = g.pid_throttle_accel.get_integrator();
    }else{
        i = g.pid_throttle_accel.get_i(z_accel_error, .01);
    }
    d = g.pid_throttle_accel.get_d(z_accel_error, .01);

    //
    // limit the rate
    output =  constrain(p+i+d+g.throttle_cruise, g.throttle_min, g.throttle_max);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THR_ACCEL_KP || g.radio_tuning == CH6_THR_ACCEL_KI || g.radio_tuning == CH6_THR_ACCEL_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THR_ACCEL_KP, z_accel_error, p, i, d, output, tuning_value);
        }
    }
#endif

    return output;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain(throttle_control,0,1000);
    g.throttle_mid = constrain(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND 100        // the throttle input channel's deadband in PWM
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+THROTTLE_IN_DEADBAND)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-THROTTLE_IN_DEADBAND)  // bottom of the deadband
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_pilot_desired_acceleration - transform pilot's throttle input to a desired acceleration
// default upper and lower bounds are 500 cm/s/s (roughly 1/2 a G)
// returns acceleration in cm/s/s
static int16_t get_pilot_desired_acceleration(int16_t throttle_control)
{
    int32_t desired_accel = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_accel = (int32_t)ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if(throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_accel = (int32_t)ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_accel = 0;
    }

    return desired_accel;
}

// get_pilot_desired_direct_alt - transform pilot's throttle input to a desired altitude
// return altitude in cm between 0 to 10m
static int32_t get_pilot_desired_direct_alt(int16_t throttle_control)
{
    int32_t desired_alt = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    desired_alt = throttle_control;

    return desired_alt;
}

// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
static void
get_throttle_rate(int16_t z_target_speed)
{
    static uint32_t last_call_ms = 0;
    static float z_rate_error = 0;   // The velocity error in cm.
    int32_t p,i,d;      // used to capture pid values for logging
    int16_t output;     // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_rate_error    = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error    = z_rate_error + 0.20085 * ((z_target_speed - climb_rate) - z_rate_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle.get_p(z_rate_error);

    // freeze I term if we've breached throttle limits
    if(motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT)) {
        i = g.pid_throttle.get_integrator();
    }else{
        i = g.pid_throttle.get_i(z_rate_error, .02);
    }
    d = g.pid_throttle.get_d(z_rate_error, .02);

    // consolidate target acceleration
    output =  p+i+d;

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THROTTLE_KP || g.radio_tuning == CH6_THROTTLE_KI || g.radio_tuning == CH6_THROTTLE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THROTTLE_KP, z_rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // send output to accelerometer based throttle controller if enabled otherwise send directly to motors
    if( g.throttle_accel_enabled ) {
        // set target for accel based throttle controller
        set_throttle_accel_target(output);
    }else{
        set_throttle_out(g.throttle_cruise+output, true);
    }

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(g.rc_3.servo_out);
    }
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
static void
get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    int16_t desired_rate;
    int32_t linear_distance;      // the distace we swap between linear and sqrt.

    // calculate altitude error
    alt_error    = target_alt - current_loc.alt;

    // check kP to avoid division by zero
    if( g.pi_alt_hold.kP() != 0 ) {
        linear_distance = 250/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*250*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*250*(-alt_error-linear_distance));
        }else{
            desired_rate = g.pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }

    desired_rate = constrain(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

    // update altitude error reported to GCS
    altitude_error = alt_error;

    // TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
static void
get_throttle_althold_with_slew(int16_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    // limit target altitude change
    controller_desired_alt += constrain(target_alt-controller_desired_alt, min_climb_rate*0.02, max_climb_rate*0.02);

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    get_throttle_althold(controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
static void
get_throttle_rate_stabilized(int16_t target_rate)
{
    controller_desired_alt += target_rate * 0.02;

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    set_new_altitude(controller_desired_alt);

    get_throttle_althold(controller_desired_alt, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
static void
get_throttle_land()
{
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        get_throttle_althold_with_slew(LAND_START_ALT, g.auto_velocity_z_min, -abs(g.land_speed));
    }else{
        get_throttle_rate_stabilized(-abs(g.land_speed));

        // detect whether we have landed by watching for minimum throttle and now movement
        if (abs(climb_rate) < 20 && (g.rc_3.servo_out <= get_angle_boost(g.throttle_min) || g.pid_throttle_accel.get_integrator() <= -150)) {
            if( land_detector < LAND_DETECTOR_TRIGGER ) {
                land_detector++;
            }else{
                set_land_complete(true);
                if( g.rc_3.control_in == 0 || ap.failsafe ) {
                    init_disarm_motors();
                }
            }
        }else{
            // we've sensed movement up or down so decrease land_detector
            if (land_detector > 0 ) {
                land_detector--;
            }
        }
    }
}

// get_throttle_surface_tracking - hold copter at the desired distance above the ground
// updates accel based throttle controller targets
static void
get_throttle_surface_tracking(int16_t target_rate)
{
    static float target_sonar_alt = 0;   // The desired altitude in cm above the ground
    static uint32_t last_call_ms = 0;
    float distance_error;
    float sonar_induced_slew_rate;

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 200 ) {
        target_sonar_alt = sonar_alt + controller_desired_alt - current_loc.alt;
    }
    last_call_ms = now;

    target_sonar_alt += target_rate * 0.02;

    distance_error = (target_sonar_alt-sonar_alt);
    sonar_induced_slew_rate = constrain(fabs(THR_SURFACE_TRACKING_P * distance_error),0,THR_SURFACE_TRACKING_VELZ_MAX);

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain(target_sonar_alt,sonar_alt-750,sonar_alt+750);
    controller_desired_alt = current_loc.alt+(target_sonar_alt-sonar_alt);
    set_new_altitude(controller_desired_alt);

    get_throttle_althold_with_slew(controller_desired_alt, target_rate-sonar_induced_slew_rate, target_rate+sonar_induced_slew_rate);   // VELZ_MAX limits how quickly we react
}

/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_stability_I();
    reset_wind_I();
    reset_throttle_I();
    reset_optflow_I();

    // This is the only place we reset Yaw
    g.pi_stabilize_yaw.reset_I();
}

static void reset_rate_I()
{
    g.pid_rate_roll.reset_I();
    g.pid_rate_pitch.reset_I();
    g.pid_rate_yaw.reset_I();
}

static void reset_optflow_I(void)
{
    g.pid_optflow_roll.reset_I();
    g.pid_optflow_pitch.reset_I();
    of_roll = 0;
    of_pitch = 0;
}

static void reset_wind_I(void)
{
    // Wind Compensation
    // this i is not currently being used, but we reset it anyway
    // because someone may modify it and not realize it, causing a bug
    g.pi_loiter_lat.reset_I();
    g.pi_loiter_lon.reset_I();

    g.pid_loiter_rate_lat.reset_I();
    g.pid_loiter_rate_lon.reset_I();

    g.pid_nav_lat.reset_I();
    g.pid_nav_lon.reset_I();
}

static void reset_throttle_I(void)
{
    // For Altitude Hold
    g.pi_alt_hold.reset_I();
    g.pid_throttle.reset_I();
    g.pid_throttle_accel.reset_I();
}

static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_throttle_accel.set_integrator(pilot_throttle-g.throttle_cruise);
}

static void reset_stability_I(void)
{
    // Used to balance a quad
    // This only needs to be reset during Auto-leveling in flight
    g.pi_stabilize_roll.reset_I();
    g.pi_stabilize_pitch.reset_I();
}



//****************** Experimental VTOL Stuff =====================

/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
static float get_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain(speed_scaler, 0.5, 2.0);
    } else {
        if (g.channel_throttle.servo_out > 0) {
            speed_scaler = 0.5 + ((float)THROTTLE_CRUISE / g.channel_throttle.servo_out / 2.0);                 // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67;
        }
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain(speed_scaler, 0.6, 1.67);
    }
    return speed_scaler;
}
/*
  return true if the current settings and mode should allow for stick mixing
 */
static bool stick_mixing_enabled(void)
{
#if 0   
  if (control_mode == CIRCLE || control_mode > FLY_BY_WIRE_B) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing &&
            geofence_stickmixing() &&
            failsafe == FAILSAFE_NONE) {
            // we're in an auto mode, and haven't triggered failsafe
            return true;
        } else {
            return false;
        }
    }
#endif    
    // non-auto mode. Always do stick mixing
    return true;
}
static void stabilize()
{
    float ch1_inf = 1.0;
    float ch2_inf = 1.0;
    float ch4_inf = 1.0;
    float speed_scaler = get_speed_scaler();

#if 0
    if(crash_timer > 0) {
        nav_roll_cd = 0;
    }
    if (inverted_flight) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }
#endif
#if 1  // Use just normal PID's at the moment for VTOL
//#if APM_CONTROL == DISABLED
	// Calculate dersired servo output for the roll
	// ---------------------------------------------
	g.channel_roll_out.servo_out = g.pidServoRoll.get_pid((nav_roll_cd - ahrs.roll_sensor), speed_scaler);
	int32_t tempcalc = nav_pitch_cd +
	        fabs(ahrs.roll_sensor * g.kff_pitch_compensation) +
	        (g.channel_throttle.servo_out * g.kff_throttle_to_pitch) -
	        (ahrs.pitch_sensor - g.pitch_trim_cd);

    if (inverted_flight) {
        // when flying upside down the elevator control is inverted
        tempcalc = -tempcalc;
    }    
    g.channel_pitch_out.servo_out = g.pidServoPitch.get_pid(tempcalc, speed_scaler);
    
 
     static char r;
 
    // calculate roll and pitch control using new APM_Control library
	//g.channel_roll_out.servo_out = g.rollController.get_servo_out(nav_roll_cd, speed_scaler, control_mode == STABILIZE);
	//g.channel_pitch_out.servo_out = g.pitchController.get_servo_out(nav_pitch_cd, speed_scaler, control_mode == STABILIZE);    
#endif

    // Mix Stick input to allow users to override control surfaces
    // -----------------------------------------------------------
    if (stick_mixing_enabled()) {
        if (control_mode != FLY_BY_WIRE_A && 
            control_mode != FLY_BY_WIRE_B) {
            // do stick mixing on aileron/elevator if not in a fly by
            // wire mode
            ch1_inf = (float)g.channel_roll.radio_in - (float)g.channel_roll.radio_trim;
            ch1_inf = fabs(ch1_inf);
            ch1_inf = min(ch1_inf, 400.0);
            ch1_inf = ((400.0 - ch1_inf) /400.0);

            ch2_inf = (float)g.channel_pitch.radio_in - g.channel_pitch.radio_trim;
            ch2_inf = fabs(ch2_inf);
            ch2_inf = min(ch2_inf, 400.0);
            ch2_inf = ((400.0 - ch2_inf) /400.0);
//if(!(r++&63))gcs_send_text_fmt(PSTR("pitch roll  %d %d %d %d \n"),g.channel_pitch_out.servo_out,g.channel_roll_out.servo_out,g.channel_pitch.radio_in,g.channel_roll.radio_in);
            // scale the sensor input based on the stick input
            // -----------------------------------------------
            g.channel_roll_out.servo_out *= ch1_inf;
            g.channel_pitch_out.servo_out *= ch2_inf;
            
            // Mix in stick inputs
            // -------------------
            g.channel_roll_out.servo_out +=     g.channel_roll.pwm_to_angle();
            g.channel_pitch_out.servo_out +=    g.channel_pitch.pwm_to_angle();
        }
 //if(!(r++&63))gcs_send_text_fmt(PSTR("pitch roll  %d %d %d %d\n"),g.channel_pitch_out.servo_out,g.channel_roll_out.servo_out,nav_roll_cd,nav_pitch_cd);
        // stick mixing performed for rudder for all cases including FBW
        // important for steering on the ground during landing
        // -----------------------------------------------
        ch4_inf = (float)g.channel_rudder.radio_in - (float)g.channel_rudder.radio_trim;
        ch4_inf = fabs(ch4_inf);
        ch4_inf = min(ch4_inf, 400.0);
        ch4_inf = ((400.0 - ch4_inf) /400.0);
    }
#if 0  // No Rudder yet 
	// Apply output to Rudder
	// ----------------------
	calc_nav_yaw(speed_scaler, ch4_inf);
	g.channel_rudder.servo_out *= ch4_inf;
	g.channel_rudder.servo_out += g.channel_rudder.pwm_to_angle();
#endif
	// Call slew rate limiter if used
	// ------------------------------
	//#if(ROLL_SLEW_LIMIT != 0)
	//	g.channel_roll.servo_out = roll_slew_limit(g.channel_roll.servo_out);
	//#endif
}
/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t flapSpeedSource = 0;
    int16_t last_throttle = g.channel_throttle.radio_out;

    if(control_mode == MANUAL) {
        // do a direct pass through of radio PWM values
        if ( g.mix_mode == 0 ) {
            g.channel_roll_out.radio_out                = g.channel_roll.radio_in;
            g.channel_pitch_out.radio_out               = g.channel_pitch.radio_in;
        } else {
            //JS140308 Had to go from PWM -> angle -> PWM, since background updates seem to destroy radio_in values, or something?
            g.channel_roll_out.servo_out = BOOL_TO_SIGN(g.reverse_ch1_elevon)*(/*elevon1_trim-1500*/ + g.channel_pitch.pwm_to_angle() - BOOL_TO_SIGN(g.reverse_elevons)*g.channel_roll.pwm_to_angle());
            g.channel_pitch_out.servo_out = BOOL_TO_SIGN(g.reverse_ch2_elevon)*(/*elevon2_trim-1500*/ + g.channel_pitch.pwm_to_angle() + BOOL_TO_SIGN(g.reverse_elevons)*g.channel_roll.pwm_to_angle());
            g.channel_roll_out.calc_pwm();
            g.channel_pitch_out.calc_pwm();

            /*            
            g.channel_roll_out.radio_out                = (int16_t)( elevon1_trim -1500 + BOOL_TO_SIGN(g.reverse_ch1_elevon) * (g.channel_pitch.radio_in-1500) - (BOOL_TO_SIGN(g.reverse_elevons) * (g.channel_roll.radio_in-1500)) );
            g.channel_pitch_out.radio_out               = (int16_t)( elevon2_trim -1500 + BOOL_TO_SIGN(g.reverse_ch2_elevon) * (g.channel_pitch.radio_in-1500) + (BOOL_TO_SIGN(g.reverse_elevons) * (g.channel_roll.radio_in-1500)) );
            */

        } 
   #if 0
        g.channel_throttle.radio_out    = g.channel_throttle.radio_in;
        g.channel_rudder.radio_out              = g.channel_rudder.radio_in;

        // setup extra aileron channel. We want this to come from the
        // main aileron input channel, but using the 2nd channels dead
        // zone, reverse and min/max settings. We need to use
        // pwm_to_angle_dz() to ensure we don't trim the value for the
        // deadzone of the main aileron channel, otherwise the 2nd
        // aileron won't quite follow the first one
        int16_t aileron_in = g.channel_roll.pwm_to_angle_dz(0);
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, aileron_in);

        // this aileron variant assumes you have the corresponding
        // input channel setup in your transmitter for manual control
        // of the 2nd aileron
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);

        // copy flap control from transmitter
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);

        if (g.mix_mode != 0) {
            // set any differential spoilers to follow the elevons in
            // manual mode. 
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler1, g.channel_roll.radio_out);
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler2, g.channel_pitch.radio_out);
        }
  #endif
    } else {
        if (g.mix_mode == 0) {
            // both types of secondary aileron are slaved to the roll servo out
         //   RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, g.channel_roll.servo_out);
          //  RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron_with_input, g.channel_roll.servo_out);
        }else{
            /*Elevon mode*/
            int16_t ch1;
            int16_t ch2;
            // Convert to radio_out angle (not PWM)
            ch1 = g.channel_pitch_out.servo_out - (BOOL_TO_SIGN(g.reverse_elevons) * g.channel_roll_out.servo_out);
            ch2 = g.channel_pitch_out.servo_out + (BOOL_TO_SIGN(g.reverse_elevons) * g.channel_roll_out.servo_out);
#if 0
			/* Differential Spoilers
               If differential spoilers are setup, then we translate
               rudder control into splitting of the two ailerons on
               the side of the aircraft where we want to induce
               additional drag.
             */
			if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) && RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
				float ch3 = ch1;
				float ch4 = ch2;
				if ( BOOL_TO_SIGN(g.reverse_elevons) * g.channel_rudder.servo_out < 0) {
				    ch1 += abs(g.channel_rudder.servo_out);
				    ch3 -= abs(g.channel_rudder.servo_out);
				} else {
					ch2 += abs(g.channel_rudder.servo_out);
				    ch4 -= abs(g.channel_rudder.servo_out);
				}
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler1, ch3);
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler2, ch4);
			}
#endif

            g.channel_roll_out.servo_out  =     (int16_t)( /* elevon1_trim -1500 + */ (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1)));
            g.channel_pitch_out.servo_out =     (int16_t)( /* elevon2_trim -1500 + */ (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2)));
        }							// trim doesn't work, since ...servo_out is in degrees -4500 - +4500 , not PWM (~900 - ~2100)

#if OBC_FAILSAFE == ENABLED
        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
        if (obc.crash_plane()) {
            g.channel_roll.servo_out = -4500;
            g.channel_pitch.servo_out = -4500;
            g.channel_rudder.servo_out = -4500;
            g.channel_throttle.servo_out = 0;
        }
#endif
        


        // push out the PWM values
/*
       if (g.mix_mode == 0) {
            g.channel_roll_out.calc_pwm();
            g.channel_pitch_out.calc_pwm();
       }
*/

        static int r;
if(!(r++&63))gcs_send_text_fmt(PSTR("pitch roll  %d %d %d %d\n"),g.channel_pitch_out.servo_out,g.channel_roll_out.servo_out,g.channel_pitch_out.radio_out,g.channel_roll_out.radio_out);
       // g.channel_rudder.calc_pwm();

#if THROTTLE_OUT == 0
        g.channel_throttle.servo_out = 0;
#else
        // convert 0 to 100% into PWM
        g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out, 
                                                 g.throttle_min.get(), 
                                                 g.throttle_max.get());

        if (suppress_throttle()) {
            // throttle is suppressed in auto mode
            g.channel_throttle.servo_out = 0;
            if (g.throttle_suppress_manual) {
                // manual pass through of throttle while throttle is suppressed
                g.channel_throttle.radio_out = g.channel_throttle.radio_in;
            } else {
                g.channel_throttle.calc_pwm();                
            }
        } else if (g.throttle_passthru_stabilize && 
                   (control_mode == STABILIZE || control_mode == FLY_BY_WIRE_A)) {
            // manual pass through of throttle while in FBWA or
            // STABILIZE mode with THR_PASS_STAB set
            g.channel_throttle.radio_out = g.channel_throttle.radio_in;
        } else {
            // normal throttle calculation based on servo_out
            g.channel_throttle.calc_pwm();
        }
#endif
    }
#if 0
    // Auto flap deployment
    if(control_mode < FLY_BY_WIRE_B) {
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);
    } else if (control_mode >= FLY_BY_WIRE_B) {
        // FIXME: use target_airspeed in both FBW_B and g.airspeed_enabled cases - Doug?
        if (control_mode == FLY_BY_WIRE_B) {
            flapSpeedSource = target_airspeed_cm * 0.01;
        } else if (airspeed.use()) {
            flapSpeedSource = g.airspeed_cruise_cm * 0.01;
        } else {
            flapSpeedSource = g.throttle_cruise;
        }
        if ( g.flap_1_speed != 0 && flapSpeedSource > g.flap_1_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, 0);
        } else if (g.flap_2_speed != 0 && flapSpeedSource > g.flap_2_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_1_percent);
        } else {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_2_percent);
        }
    }

    if (control_mode >= FLY_BY_WIRE_B) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit(last_throttle);
    }
#endif    
#if 0 // Do this different way in VTOL
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
    // send values to the PWM timers for output
    // ----------------------------------------
    APM_RC.OutputCh(CH_1, g.channel_roll.radio_out);     // send to Servos
    APM_RC.OutputCh(CH_2, g.channel_pitch.radio_out);     // send to Servos
    APM_RC.OutputCh(CH_3, g.channel_throttle.radio_out);     // send to Servos
    APM_RC.OutputCh(CH_4, g.channel_rudder.radio_out);     // send to Servos
    // Route configurable aux. functions to their respective servos
    g.rc_5.output_ch(CH_5);
    g.rc_6.output_ch(CH_6);
    g.rc_7.output_ch(CH_7);
    g.rc_8.output_ch(CH_8);
 # if CONFIG_APM_HARDWARE != APM_HARDWARE_APM1
    g.rc_9.output_ch(CH_9);
    g.rc_10.output_ch(CH_10);
    g.rc_11.output_ch(CH_11);
 # endif
#endif
#endif
}
