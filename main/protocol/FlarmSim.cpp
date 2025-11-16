/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlarmSim.h"

#include "nmea_util.h"
#include "comm/DeviceMgr.h"
#include "protocol/ClockIntf.h"
#include "protocol/Clock.h"

#include <logdefnone.h>

#include <string>



// A FLARM "AU" message simulator

// Messages to simulate FLARM AU sentences sequence
// -> Level, rel. bearing, craft type, rel. vertical, rel. distance
const char *pflau_msg[] = {
    "1,-30, 2, -60, 300",
    "1,-22, 2, -54, 260",
    "1,-15, 2, -50, 205",
    "1, -5, 2, -45, 157",
    "2, 14, 2, -40, 110",
    "2, 45, 2, -35, 90",
    "3, 80, 2, -30, 100",
    "3, 95, 2, -25, 137",
    "2, 97, 2, -20, 167",
    "1, 96, 2, -15, 167",
    "1, 96, 2, -10, 166",
    ""
    };

FlarmSim *FlarmSim::_sim = nullptr;

// wait 5 seconds before first message 
// iterate and inject through messages every other second
// then delete simulator
bool FlarmSim::tick()
{
    ESP_LOGI(FNAME,"flarmSim tick");

    int nextmsg = _tick_count-5;

    if ( nextmsg >= 0 ) {
        if ( *pflau_msg[nextmsg] != '\0' )
        {
            std::string msg("$PFLAU,3,1,2,1,");
            msg += pflau_msg[nextmsg];
            msg += ",1234*";
            msg += NMEA::CheckSum(msg.c_str()) + "\r\n";
            _d->_link->process(msg.c_str(), msg.size());
        }
        else {
            delete this;
            return true;
        }
    }
    _tick_count++;
    return false;
}

FlarmSim::FlarmSim(Device *d) :
    Clock_I(100), // generates a time-out callback ca. every second
    _d(d)
{
    ESP_LOGI(FNAME,"Kick ticker");
    Clock::start(this);
}

FlarmSim::~FlarmSim()
{
    InterfaceCtrl *ic = _d->_itf;
    // Reconnect datalink
    ic->addDataLink(_d->_link);
    _sim = nullptr;
}

// start a flarm alarm situation simulation
// precondition: Flarm device is configured
void FlarmSim::StartSim()
{
    if ( ! _sim ) {
        ESP_LOGI(FNAME,"Start Simulator");
        // only one at a time
        Device *d = DEVMAN->getDevice(FLARM_DEV);
        if ( d ) {
            ESP_LOGI(FNAME,"Found Flarm %d", d->_id);
            // a Flarm is connected
            InterfaceCtrl *ic = d->_itf;
            ic->MoveDataLink(0); // the device has a backup of the data link pointer
            _sim = new FlarmSim(d);
        }

    }
}

