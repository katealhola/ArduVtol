// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_MotorsTriVTOL.h
/// @brief	Motor control class for Tilt rotor Tricopters/VTOL

#ifndef AP_MOTORSTRIVTOL
#define AP_MOTORSTRIVTOL

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_Motors.h>

// tail servo uses channel 7
#define AP_MOTORS_CH_TRI_YAW    CH_7
// Tilt servu uses channel 10
#define AP_MOTORS_CH_TRI_TILT    CH_10	// JS mod 140306: Output on ch10 (50Hz), Input ch5. CH5 output is 490Hz
#define AP_MOTORS_CH_VTOL_AILE    CH_6
#define AP_MOTORS_CH_VTOL_ELEV   CH_8

/// @class      AP_MotorsTri
class AP_MotorsTriVTOL : public AP_Motors {
public:

    /// Constructor
    AP_MotorsTriVTOL( uint8_t APM_version, APM_RC_Class* rc_out, RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, RC_Channel* rc_tail,RC_Channel* rc_tilt,RC_Channel* rc_aile,RC_Channel* rc_elev, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(APM_version, rc_out, rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _rc_tail(rc_tail),_rc_tilt(rc_tilt),_rc_aile(rc_aile),_rc_elev(rc_elev) {
    };

    // init
    virtual void            Init();

    // set update rate to motors - a value in hertz
    void                    set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    virtual void            enable();

    // get basic information about the platform
    virtual uint8_t         get_num_motors() {
        return 7;
    };                       // 3 motors + 1 tail servo + 1 tilt servo + 2 aile/elev servos 
	void output_servos();

    // motor test
    virtual void        output_test();

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    RC_Channel*         _rc_tail;       // REV parameter used from this channel to determine direction of tail servo movement
    RC_Channel*         _rc_tilt;       // this channel for tilt servo
    RC_Channel*         _rc_aile;       // this channel for left aileron/elevon servo
    RC_Channel*         _rc_elev;       // this channel for right aileron/elevon servo
};

#endif  // AP_MOTORSTRIVTOL
