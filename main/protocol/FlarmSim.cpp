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
#include "protocol/Clock.h"
#include "math/Trigonometry.h"
#include "math/Quaternion.h"
#include "logdefnone.h"

#include <esp_random.h>

#include <string>



// A FLARM "AU" message simulator


struct SIMRUN {
    vector_f target_rel_pos; // x=front, y=right, z=up
    vector_f target_rel_velocity; // x=front, y=right, z=up
    float target_omega; // deg per second
    float ownship_omega;
};
// Assume ownship at origin, flying along x axis at 27m/s (~100km/h)
// Target position and velocity relative to ownship
const SIMRUN simVariant[6] = {
    // simVariant 0: crossing from left to right, descending
    { {300.f, -200.f, -30.f}, {0.f, 20.f, -1.8f}, 0.f, 0.f },
    // simVariant 1: crossing from right to left, climbing
    { {300.f, 200.f, 30.f}, {0.f, -20.f, 1.8f}, 0.f, 0.f },
    // simVariant 2: head-on, descending
    { {300.f, 0.f, -20.f}, {-20.f, 0.f, -1.5f}, 0.f, 0.f },
    // simVariant 3: overtaking from behind left, climbing
    { {-70.f, -45.f, 10.f}, {37.f, 1.f, 0.7f}, 0.f, 0.f },
    // simVariant 4: crossing from left to right, level
    { {300.f, -100.f, 0.f}, {5.f, 20.f, 0.f}, 0.f, 0.f },
    // simVariant 5: target circling
    { {300.f, 0.f, -20.f}, {-5.f, -17.f, 1.5f}, 12.f, 0.f }
};


FlarmSim *FlarmSim::_sim = nullptr;

// wait 5 seconds before first message 
// iterate and inject through messages every other second
// then delete simulator
bool FlarmSim::tick()
{
    ESP_LOGI(FNAME,"flarmSim tick");

    if ( _tick_count < 20 || ! _done )
    {
        // Messages to simulate FLARM AU sentences sequence
        // -> Level, rel. bearing, craft type, rel. vertical, rel. distance
        vector_f o_to_t = _target_pos - _own_pos;
        int dist = o_to_t.get_norm();
        if ( dist > 310 ) { _done = true;}
        int bear = rad2deg( fast_atan2(o_to_t.y, o_to_t.x) );
        int vert = o_to_t.z;
        int levl = 3 - (dist / 50);
        if ( levl < 1 ) { levl = 1; }
        std::string msg("$PFLAU,3,1,2,1,");
        msg += std::to_string(levl) + "," + std::to_string(bear) + ",2," + std::to_string(vert) 
            + "," + std::to_string(dist);
        ESP_LOGI(FNAME,"SimMsg: %s", msg.c_str());
        msg += "," + std::to_string(_icao_id) + "*"; // dummy ICAO

        msg += NMEA::CheckSum(msg.c_str()) + "\r\n";
        _d->_link->process(msg.c_str(), msg.size());

        // increment own, and target position
        _own_pos.x += 27.f; // ownship at 100km/h
        Quaternion q(deg2rad(_target_omega), vector_ijk(0.f, 0.f, 1.f));
        _target_inc = q * _target_inc;  
        _target_pos += _target_inc;
    }
    else {
        delete this;
        return true;
    }

    _tick_count++;
    return false;
}

FlarmSim::FlarmSim(Device *d, const SIMRUN *sv) :
    Clock_I(100), // generates a time-out callback ca. every second
    _d(d)
{
    // init vectors
    _target_pos = sv->target_rel_pos;
    _target_inc = sv->target_rel_velocity;
    _target_omega = sv->target_omega;
    _own_pos = vector_f{0.f, 0.f, 0.f};
    _icao_id = esp_random() & 0xFFFFFF; // random ICAO
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
void FlarmSim::StartSim(int variant)
{
    if ( ! _sim ) {
        ESP_LOGI(FNAME,"Start Simulator %d", variant);
        // only one at a time
        Device *d = DEVMAN->getDevice(FLARM_DEV);
        if ( d ) {
            ESP_LOGI(FNAME,"Found Flarm %d", d->_id);
            // a Flarm is connected
            InterfaceCtrl *ic = d->_itf;
            ic->MoveDataLink(0); // the device has a backup of the data link pointer
            _sim = new FlarmSim(d, simVariant + variant);
        }

    }
}

