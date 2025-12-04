/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "CANPeerCaps.h"

#include "InterfaceCtrl.h"
#include "comm/Devices.h"
#include "comm/DeviceMgr.h"
#include "setup/SetupNG.h"
#include "logdef.h"

#include <cstring>

// The XCV CAN peer capabilities query message.
//
// - Caps need to be know on early stage to configure the device properly. Thus the Xcv Sync message is not sufficient and 
//      might not act in the desired timely fashion. A dedicated CAN message is used to query the peer capabilities
//      right after power-on or reset.
//      The suggested message is used from both sides, master and client (protocol).
//   $PJPCAP, <token>, <protocol type>\r\n
//
// The expected peer response
// - The respons is just a string with one letter per capability.
//   $PJPCAP, <token>, <cap ids string>*<CRC>\r\n
//
// OR, piggiback on the registration query/response message from master to client
//   $PJPREG, <token>, <protocol type>, <client caps>\r\n
//   $PJMACC, <token>, <drive id>, <master id>, <cap ids string>*<CRC>\r\n
//

static const char *CAP_BITS = "GFWRHLEA";

std::string CANPeerCaps::encodeCaps(int cap_flags)
{
    // to compose the cap flags message "$PJCAP, 123, caps_string"
    std::string caps;
    int all_caps = strlen(CAP_BITS);
    for ( int i = 0; i < all_caps; i++ ) {
        if ( cap_flags & (1 << i) ) {
            caps += CAP_BITS[i];
        }
    }
    ESP_LOGI(FNAME, "encodeCaps: %x->%s", cap_flags, caps.c_str());
    return caps;
}

int CANPeerCaps::decodeCaps(const char* caps_str)
{
    // grab cap flags message "$PJCAP, 123, caps_string"
    int caps = 0;
    if ( caps_str != nullptr ) {
        size_t len = strlen(caps_str);
        for ( int i = 0; i < len; i++ ) {
            const char *p = strchr(CAP_BITS, caps_str[i]);
            if ( p != nullptr ) {
                caps |= 1 << (int)(p-CAP_BITS);
            }
        }
    }
    ESP_LOGI(FNAME, "decodeCaps: %s->%x", caps_str, caps);
    return caps;
}

// translate devices to capabilities
void CANPeerCaps::updateMyCapabilities(DeviceId did, bool add)
{
    int cap = 0;
    switch ( did ) {
        case ANEMOI_DEV:
            cap = XcvCaps::EXTWIND_CAP;
            break;
        case FLARM_DEV:
            cap = XcvCaps::FLARM_CAP | XcvCaps::GPS_CAP;
            break;
        case MAGSENS_DEV:
            cap = XcvCaps::HEADING_CAP;
            break;
        case RADIO_KRT2_DEV:
        case RADIO_ATR833_DEV:
            cap = XcvCaps::RADIOCTRL_CAP;
            break;
        default:
            break;
    }
    if ( cap != 0 ) {
        if ( add ) {
            my_caps.set(my_caps.get() | cap);
        } else {
            my_caps.set(my_caps.get() & (~cap));
        }
    }
}

void CANPeerCaps::setupPeerProtos(int listen_port, int send_port)
{
    // look for caps the peer has but not this device
    int capdiff  = peer_caps.get() & (~my_caps.get());

    // setup peer protocols based on extra peer caps
    if ( capdiff & XcvCaps::FLARM_CAP ) {
        ESP_LOGI(FNAME, "Peer has FLARM host capability, add virtual device on peer connection");
        DEVMAN->addDevice(FLARM_DEV, FLARM_P, listen_port, send_port, CAN_BUS, false);
    }
}