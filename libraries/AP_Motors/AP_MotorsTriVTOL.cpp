/*
 *       AP_MotorsTriVTOL.cpp - ArduCopter motors library
 *       Code by Kate Alhola katix.org
 *       Based on Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsTriVTOL.h"

// init
void AP_MotorsTriVTOL::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsTriVTOL::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    _rc->SetFastOutputChannels(_BV(_motor_to_channel_map[AP_MOTORS_MOT_1]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_2]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_4]), _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsTriVTOL::enable()
{
    // enable output channels
    _rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_1]);
    _rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_2]);
    _rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_4]);
    _rc->enable_out(AP_MOTORS_CH_TRI_YAW);
    _rc->enable_out(AP_MOTORS_CH_TRI_TILT);
    _rc->enable_out(AP_MOTORS_CH_VTOL_ELEV);
    _rc->enable_out(AP_MOTORS_CH_VTOL_AILE);
}

// output_min - sends minimum values out to the motors
void AP_MotorsTriVTOL::output_min()
{
    // fill the motor_out[] array for HIL use
    motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_min;
    motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_min;
    motor_out[AP_MOTORS_MOT_4] = _rc_throttle->radio_min;

    // send minimum value to each motor
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], _rc_throttle->radio_min);
    //    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_CH_TRI_YAW], _rc_yaw->radio_trim);
    //_rc->OutputCh(_motor_to_channel_map[AP_MOTORS_CH_TRI_TILT], _rc_yaw->radio_trim);
}

// output_armed - sends commands to the motors
void AP_MotorsTriVTOL::output_armed()
{
    int16_t out_min = _rc_throttle->radio_min;
    int16_t out_max = _rc_throttle->radio_max;

    // Throttle is 0 to 1000 only
    _rc_throttle->servo_out = constrain(_rc_throttle->servo_out, 0, _max_throttle);

    if(_rc_throttle->servo_out > 0)
        out_min = _rc_throttle->radio_min + _min_throttle;

    // capture desired roll, pitch, yaw and throttle from receiver
    _rc_roll->calc_pwm();
    _rc_pitch->calc_pwm();
    _rc_throttle->calc_pwm();

    int roll_out            = (float)_rc_roll->pwm_out * .866;
    int pitch_out           = _rc_pitch->pwm_out / 2;

    //left front
    motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_out + roll_out + pitch_out;
    //right front
    motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_out - roll_out + pitch_out;
    // rear
    motor_out[AP_MOTORS_MOT_4] = _rc_throttle->radio_out - _rc_pitch->pwm_out;

    // Tridge's stability patch
    if(motor_out[AP_MOTORS_MOT_1] > out_max) {
        motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_1] - out_max) >> 1;
        motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_1] - out_max) >> 1;
        motor_out[AP_MOTORS_MOT_1] = out_max;
    }

    if(motor_out[AP_MOTORS_MOT_2] > out_max) {
        motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_2] - out_max) >> 1;
        motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_2] - out_max) >> 1;
        motor_out[AP_MOTORS_MOT_2] = out_max;
    }

    if(motor_out[AP_MOTORS_MOT_4] > out_max) {
        motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_4] - out_max) >> 1;
        motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_4] - out_max) >> 1;
        motor_out[AP_MOTORS_MOT_4] = out_max;
    }

    // adjust for throttle curve
    if( _throttle_curve_enabled ) {
        motor_out[AP_MOTORS_MOT_1] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_1]);
        motor_out[AP_MOTORS_MOT_2] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_2]);
        motor_out[AP_MOTORS_MOT_4] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_4]);
    }

    // ensure motors don't drop below a minimum value and stop
    motor_out[AP_MOTORS_MOT_1] = max(motor_out[AP_MOTORS_MOT_1],    out_min);
    motor_out[AP_MOTORS_MOT_2] = max(motor_out[AP_MOTORS_MOT_2],    out_min);
    motor_out[AP_MOTORS_MOT_4] = max(motor_out[AP_MOTORS_MOT_4],    out_min);

#if CUT_MOTORS == ENABLED
    // if we are not sending a throttle output, we cut the motors
    if(_rc_throttle->servo_out == 0) {
        motor_out[AP_MOTORS_MOT_1]      = _rc_throttle->radio_min;
        motor_out[AP_MOTORS_MOT_2]      = _rc_throttle->radio_min;
        motor_out[AP_MOTORS_MOT_4] = _rc_throttle->radio_min;
    }
#endif

    // send output to each motor
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], motor_out[AP_MOTORS_MOT_1]);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], motor_out[AP_MOTORS_MOT_2]);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], motor_out[AP_MOTORS_MOT_4]);

    // also send out to tail command (we rely on any auto pilot to have updated the rc_yaw->radio_out to the correct value)
    // note we do not save the radio_out to the motor_out array so it may not appear in the ch7out in the status screen of the mission planner
    // note: we use _rc_tail's (aka channel 7's) REV parameter to control whether the servo is reversed or not but this is a bit nonsensical.
    //       a separate servo object (including min, max settings etc) would be better or at least a separate parameter to specify the direction of the tail servo
    if( _rc_tail->get_reverse() == true ) {
      _rc->OutputCh(AP_MOTORS_CH_TRI_YAW, _rc_yaw->radio_trim - (_rc_yaw->radio_out - _rc_yaw->radio_trim));
	 motor_out[AP_MOTORS_CH_TRI_YAW]=_rc_yaw->radio_trim - (_rc_yaw->radio_out - _rc_yaw->radio_trim);
    }else{
        _rc->OutputCh(AP_MOTORS_CH_TRI_YAW, _rc_yaw->radio_out);
	motor_out[AP_MOTORS_CH_TRI_YAW]=_rc_yaw->radio_out;
    }
    output_servos();

 // static char r;
 //  if(!(r++&15))gcs_send_text_fmt(PSTR("motors %d %d %d %d\n"),motor_out[AP_MOTORS_MOT_1],motor_out[AP_MOTORS_MOT_2],motor_out[AP_MOTORS_MOT_4],motor_out[AP_MOTORS_CH_TRI_TILT],motor_out[AP_MOTORS_CH_TRI_YAW]);
}

// Output servo channels 

void AP_MotorsTriVTOL::output_servos()
{
    _rc_yaw->calc_pwm();
    _rc_tilt->calc_pwm();
    _rc_elev->calc_pwm();
    _rc_aile->calc_pwm();

    motor_out[AP_MOTORS_CH_TRI_TILT]= _rc_aile->radio_in; // RC5in=mode, RC5out=pitch, RC6out=aile,RC8out=elev
    motor_out[AP_MOTORS_CH_VTOL_AILE]= _rc_aile->radio_out;
    motor_out[AP_MOTORS_CH_VTOL_ELEV]= _rc_elev->radio_out;
    
 _rc->OutputCh(AP_MOTORS_CH_TRI_TILT,motor_out[AP_MOTORS_CH_TRI_TILT]);
 _rc->OutputCh(AP_MOTORS_CH_VTOL_AILE,motor_out[AP_MOTORS_CH_VTOL_AILE]);
 _rc->OutputCh(AP_MOTORS_CH_VTOL_ELEV,motor_out[AP_MOTORS_CH_VTOL_ELEV]);
}

// output_disarmed - sends commands to the motors
void AP_MotorsTriVTOL::output_disarmed()
{
    if(_rc_throttle->control_in > 0) {
        // we have pushed up the throttle
        // remove safety
        _auto_armed = true;
    }

    // fill the motor_out[] array for HIL use
    for (unsigned char i = AP_MOTORS_MOT_1; i < AP_MOTORS_MOT_4; i++) {
        motor_out[i] = _rc_throttle->radio_min;
    }
    output_servos();
    // Send minimum values to all motors
    output_min();
}

// output_disarmed - sends commands to the motors
void AP_MotorsTriVTOL::output_test()
{
    // Send minimum values to all motors
    output_min();

    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min);
    delay(4000);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min + 100);
    delay(300);

    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min);
    delay(2000);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], _rc_throttle->radio_min + 100);
    delay(300);

    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], _rc_throttle->radio_min);
    delay(2000);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min + 100);
    delay(300);

    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], motor_out[AP_MOTORS_MOT_1]);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], motor_out[AP_MOTORS_MOT_2]);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], motor_out[AP_MOTORS_MOT_4]);
}
