/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

template<typename T>
class SetupNG;

#include <string>
#include <cstdint>

enum DeviceId : uint8_t;

// a shared message from registration Master and Client on CAN bus
// here is just the helper to encode and to decode the caps string

namespace CANPeerCaps
{
    // add a capability
    void addCapability(int cap);

    // remove a capability
    void removeCapability(int cap);
    
    // update my caps from connected devices
    void updateCapsFromDev(DeviceId did, bool add);

    // setup peer protocols based on caps
    void setupPeerProtos(int listen_port, int send_port);
};

