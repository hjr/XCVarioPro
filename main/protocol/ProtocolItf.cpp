/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/


#include "ProtocolItf.h"

#include "comm/Messages.h"
#include "logdefnone.h"

ProtocolItf::~ProtocolItf()
{
    ESP_LOGI(FNAME, "Dtor dev %d, proto %d", _did, getProtocolId());
}

Message* ProtocolItf::newMessage() const
{
    return DEV::acqMessage(_did, _send_port);
}