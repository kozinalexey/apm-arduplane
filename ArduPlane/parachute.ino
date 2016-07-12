// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if PARACHUTE == ENABLED
static void parachute_check()
{
    parachute.update();
}

/*
  parachute_release - trigger the release of the parachute
*/
static void parachute_release()
{
    if (parachute.released()) {
        return;
    }
    
    // send message to gcs and dataflash
    //gcs_send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");
    gcs_send_text_P((gcs_severity) MAV_SEVERITY_CRITICAL,PSTR("Parachute: Released"));
    // release parachute
    parachute.release();
}

/*
  parachute_manual_release - trigger the release of the parachute,
  after performing some checks for pilot error checks if the vehicle
  is landed
*/
static bool parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled() || parachute.released()) {
        return false;
    }

    // do not release if vehicle is not flying
    if (!is_flying()) {
        // warn user of reason for failure
        gcs_send_text_P((gcs_severity) MAV_SEVERITY_WARNING,PSTR("Parachute: Not flying"));
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
        return false;
    }

    if (adjusted_relative_altitude_cm() / 100 < parachute.alt_min()) {
        //gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Parachute: Too low");
        gcs_send_text_P((gcs_severity) MAV_SEVERITY_WARNING,PSTR("Parachute: Too low"));
        return false;
    }

    // if we get this far release parachute
    parachute_release();

    return true;
}

#endif
