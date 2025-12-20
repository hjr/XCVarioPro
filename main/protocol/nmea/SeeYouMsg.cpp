/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SeeYouMsg.h"
#include "protocol/nmea_util.h"
#include "comm/DataLink.h"
#include "comm/Messages.h"
#include "Units.h"

#include "logdef.h"

#include <cmath>

// The Naviter/SeeYou protocol parser.
//
// Supported messages:


const ParserEntry SeeYouMsg::_pt[] = {
    {}
};

/*
    Sentence has following format:
    $PLXVF, <TIME>, <IMUaccelX>, <IMUaccelY>, <IMUaccelZ>, <VARIO>, <IAS>, <ALT>, <S2FMode>*CS\r\n
    TIME = time since epoch in seconds and millis  (float)
    IMUaccel* = G aspect (float)
    VARIO = m/s (float)
    IAS = km/h (float)
    ALT = m QNH (float)
    S2FMODE = 0:1; 0 == vario mode (int)
    CS = standard NMEA checksum
*/
void NmeaPrtcl::sendSeeYou(float accx, float accy, float accz, float vario, float ias, float alt, bool cruise)
{
    if ( _dl.isBinActive() ) {
        return; // no NMEA output in binary mode
    }
    Message* msg = newMessage();

    msg->buffer = "$PLXVF,";
    char tmp[50];
    struct timeval tv;
    gettimeofday(&tv, NULL);
    std::sprintf(tmp, "%d.%03d", (int)(tv.tv_sec % (60 * 60 * 24)), (int)(tv.tv_usec / 1000));
    msg->buffer += tmp;
    std::sprintf(tmp, ",%.1f", accx);
    msg->buffer += tmp;
    std::sprintf(tmp, ",%.1f", accy);
    msg->buffer += tmp;
    std::sprintf(tmp, ",%.1f", accz);
    msg->buffer += tmp;
    std::sprintf(tmp, ",%.1f", vario);
    msg->buffer += tmp;
    std::sprintf(tmp, ",%.1f", ias);
    msg->buffer += tmp;
    std::sprintf(tmp, ",%.1f", alt);
    msg->buffer += tmp;
    std::sprintf(tmp, ",%1d", !cruise);

    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    ESP_LOGD(FNAME, "Borgelt %s", msg->buffer.c_str());
    DEV::Send(msg);
}


