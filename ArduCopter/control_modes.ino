/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CONTROL_SWITCH_COUNTER  10  // 10 iterations at 100hz (i.e. 1/10th of a second) at a new switch position will cause flight mode change
static void read_control_switch()
{
    static uint8_t switch_counter = 0;

    byte switchPosition = readSwitch();

    if (oldSwitchPosition != switchPosition) {
        switch_counter++;
        if(switch_counter >= CONTROL_SWITCH_COUNTER) {
            oldSwitchPosition       = switchPosition;
            switch_counter          = 0;

            // ignore flight mode changes if in failsafe
            if( !ap.failsafe ) {
                set_mode(flight_modes[switchPosition]);

                if(g.ch7_option != CH7_SIMPLE_MODE) {
                    // set Simple mode using stored paramters from Mission planner
                    // rather than by the control switch
                    set_simple_mode(g.simple_modes & (1 << switchPosition));
                }
            }
        }
    }else{
        // reset switch_counter if there's been no change
        // we don't want 10 intermittant blips causing a flight mode change
        switch_counter = 0;
    }
}

static byte readSwitch(void){
    int16_t pulsewidth = g.rc_5.radio_in;                       // default for Arducopter

    if (pulsewidth > 1230 && pulsewidth <= 1360) return 1;
    if (pulsewidth > 1360 && pulsewidth <= 1490) return 2;
    if (pulsewidth > 1490 && pulsewidth <= 1620) return 3;
    if (pulsewidth > 1620 && pulsewidth <= 1749) return 4;
    if (pulsewidth >= 1750) return 5;
    return 0;
}

static void reset_control_switch()
{
    oldSwitchPosition = -1;
    read_control_switch();
}

// read at 10 hz
// set this to your trainer switch
static void read_trim_switch()
{
    // return immediately if the CH7 switch has not changed position
    if (ap_system.CH7_flag == (g.rc_7.radio_in >= CH7_PWM_TRIGGER)) {
        return;
    }

    // set the ch7 flag
    ap_system.CH7_flag = (g.rc_7.radio_in >= CH7_PWM_TRIGGER);

    // multi-mode
    int8_t option;

    if(g.ch7_option == CH7_MULTI_MODE) {
        if (g.rc_6.radio_in < CH6_PWM_TRIGGER_LOW) {
            option = CH7_FLIP;
        }else if (g.rc_6.radio_in > CH6_PWM_TRIGGER_HIGH) {
            option = CH7_SAVE_WP;
        }else{
            option = CH7_RTL;
        }
    }else{
        option = g.ch7_option;
    }

    switch(option) {
        case CH7_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if(ap_system.CH7_flag && g.rc_3.control_in >= 0 && ap.takeoff_complete) {
                init_flip();
            }
            break;

        case CH7_SIMPLE_MODE:
            set_simple_mode(ap_system.CH7_flag);
            break;

        case CH7_RTL:
            if (ap_system.CH7_flag) {
                // engage RTL
                set_mode(RTL);
            }else{
                // disengage RTL to previous flight mode if we are currently in RTL or loiter
                if (control_mode == RTL || control_mode == LOITER) {
                    reset_control_switch();
                }
            }
            break;

        case CH7_SAVE_TRIM:
            if(ap_system.CH7_flag && control_mode <= ACRO && g.rc_3.control_in == 0) {
                save_trim();
            }
            break;

        case CH7_SAVE_WP:
            // save when CH7 switch is switched off
            if (ap_system.CH7_flag == false) {

                // if in auto mode, reset the mission
                if(control_mode == AUTO) {
                    CH7_wp_index = 0;
                    g.command_total.set_and_save(1);
                    set_mode(RTL);
                    return;
                }

                if(CH7_wp_index == 0) {
                    // this is our first WP, let's save WP 1 as a takeoff
                    // increment index to WP index of 1 (home is stored at 0)
                    CH7_wp_index = 1;

                    Location temp   = home;
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    temp.id = MAV_CMD_NAV_TAKEOFF;
                    temp.alt = current_loc.alt;

                    // save command:
                    // we use the current altitude to be the target for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    // If we are above the altitude, we will skip the command.
                    set_cmd_with_index(temp, CH7_wp_index);
                }

                // increment index
                CH7_wp_index++;

                // set the next_WP (home is stored at 0)
                // max out at 100 since I think we need to stay under the EEPROM limit
                CH7_wp_index = constrain(CH7_wp_index, 1, 100);

                if(g.rc_3.control_in > 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    current_loc.id = MAV_CMD_NAV_WAYPOINT;
                }else{
                    // set our location ID to 21, MAV_CMD_NAV_LAND
                    current_loc.id = MAV_CMD_NAV_LAND;
                }

                // save command
                set_cmd_with_index(current_loc, CH7_wp_index);

                // Cause the CopterLEDs to blink twice to indicate saved waypoint
                copter_leds_nav_blink = 10;
            }
            break;

#if CAMERA == ENABLED
        case CH7_CAMERA_TRIGGER:
            if(ap_system.CH7_flag) {
                do_take_picture();
            }
            break;
#endif

        case CH7_SONAR:
            // enable or disable the sonar
            g.sonar_enabled = ap_system.CH7_flag;
            break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
static void save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)g.rc_1.control_in/100.0);
    float pitch_trim = ToRad((float)g.rc_2.control_in/100.0);
    ahrs.add_trim(roll_trim, pitch_trim);
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
static void auto_trim()
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        led_mode = SAVE_TRIM_LEDS;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)g.rc_1.control_in / 4000.0);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)g.rc_2.control_in / 4000.0);

        // make sure accelerometer values impact attitude quickly
        ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(auto_trim_counter == 0) {
            ahrs.set_fast_gains(false);
            led_mode = NORMAL_LEDS;
        }
    }
}

