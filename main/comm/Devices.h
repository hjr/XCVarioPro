/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <cstdint>

// List of supported devices
enum DeviceId : uint8_t
{
    NO_DEVICE,
    ANEMOI_DEV,
    ATR833_HOST_DEV,
    FLARM_DEV,
    FLARM_HOST_DEV,
    FLARM2_DEV,
    FLARM_HOST2_DEV,
    JUMBO_DEV,
    MAGSENS_DEV,
    MAGLEG_DEV,
    CANREGISTRAR_DEV,  // <- 10 CAN id registry
    NAVI_DEV,
    NAVI2_DEV,
    RADIO_ATR833_DEV,
    RADIO_KRT2_DEV,
    RADIO_REMOTE_DEV,
    XCVARIOFIRST_DEV,
    XCVARIOSECOND_DEV,
    FLARM_PROXY,
    RADIO_PROXY,
    TEST_DEV,
    TEST_DEV2
};


// Supported protocol id's
enum ProtocolType : uint8_t
{
    NO_ONE = 0, // not a protocol
    ANEMOI_P,
    ATR833_REMOTE_P,
    BORGELT_P,
    CAMBRIDGE_P,
    FLARM_P,
    FLARMBIN_P,
    FLARMHOST_P,
    GARMIN_P,
    JUMBOCMD_P,
    KRT2_REMOTE_P, // <- 10
    MAGSENS_P,
    MAGSENSBIN_P,
    NMEASTD_P,
    OPENVARIO_P,
    REGISTRATION_P, // CAN id registration
    XCVARIO_P,
    XCVQUERY_P,
    XCVSYNC_P,
    XCNAV_P,
    TEST_P
};
// old ones .. P_EYE_PEYA, P_EYE_PEYI


// Capabilities resulting out of protocol negotiation
enum XcvCaps : uint8_t
{
    GPS_CAP         = 1<<0, // "G"
    FLARM_CAP       = 1<<1, // "F"
    EXTWIND_CAP     = 1<<2, // "W"
    RADIOCTRL_CAP   = 1<<3, // "R"
    HEADING_CAP     = 1<<4, // "H"
    FLAPSENS_CAP    = 1<<5, // "L"
    GEARSENS_CAP    = 1<<6, // "E"
    AHRS_CAP        = 1<<7 // "A"
};

constexpr int CAN_REG_PORT = 0x7f0;

class Message;

namespace DEV
{

Message* plsMessage(DeviceId target, int port); // can fail
Message* acqMessage(DeviceId target, int port); // garanty, but potential wait
inline void relMessage(Message *msg);

bool Send(Message* msg);

}
