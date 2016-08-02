/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

bool gps_glitch_switch_mode_on_resolve = false;

static void gps_glitch_update() {
    bool short_glitch = ahrs.get_NavEKF().getGpsGlitchStatus();

    if (short_glitch && !failsafe.gps_glitch) {
        gps_glitch_on_event();
    } else if (!short_glitch && failsafe.gps_glitch) {
        gps_glitch_off_event();
    }

    if (failsafe.gps_glitch && control_mode != control_mode_glitch_on) {
        // if the mode has changed during gps glitch, don't return to control_mode_glitch_off on resolve
        gps_glitch_switch_mode_on_resolve = false;
    }

    uint8_t timeoutStatus;
    ahrs.get_NavEKF().getFilterTimeouts(timeoutStatus);
    bool long_glitch = (timeoutStatus) & (1<<(0));

}

static void gps_glitch_mode_change_commanded(uint8_t mode_commanded)
{
    // ensure that commanded mode switches to ALT_HOLD will cancel the switch back to FLY
    if (mode_commanded != control_mode_glitch_off) {
        gps_glitch_switch_mode_on_resolve = false;
    }
}

static bool gps_glitch_action_mode(uint8_t mode) {
    switch(control_mode) {
        case LAND:
            return landing_with_GPS();
        case RTL:
            return rtl_state == Land;
        case GUIDED: // GUIDED for solo because shots modes use GUIDED
        case LOITER:
        case DRIFT:
        case STOP:
        case POSHOLD:
            return true;
        default:
            return false;
    }
    return false;
}

static void gps_glitch_on_event() {
    // a GPS glitch is also captured as a failsafe event
    failsafe.gps_glitch = true;

    // do not perform any mode changes if we are disarmed or the mode does not allow it
    if (!motors.armed() || gps_glitch_action_mode(control_mode)) {
        return;
    }

    // Handle the cases where the operator does not have stick control
    if (control_mode == GUIDED) {
        if(set_mode(RTL)) {
            // The mission effectiveness has likely been compromised so it is best to RTL and abort the mission
            gps_glitch_switch_mode_on_resolve = true;
            control_mode_glitch_on = RTL;
            // we want to stay in RTLK when the glitch condition clears
            control_mode_glitch_off = RTL;
        } else if (set_mode(LAND)) {
            // If we cannot return home, then we should land to reduce the distance of any fly-away
            gps_glitch_switch_mode_on_resolve = true;
            control_mode_glitch_on = RTL;
            // we want to go to RTL if the glitch clears
            control_mode_glitch_off = RTL;
        }
    }

    // handle the case where the operator does have stick control
    if (motors.armed() && gps_glitch_action_mode(control_mode) && !failsafe.radio) {
        if(set_mode(ALT_HOLD)) {
            gps_glitch_switch_mode_on_resolve = true;
        }
    }
}

static void gps_glitch_off_event() {
    failsafe.gps_glitch = false;

    if (gps_glitch_switch_mode_on_resolve) {
        set_mode(LOITER);
    }
}
