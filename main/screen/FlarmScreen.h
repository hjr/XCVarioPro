/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/MenuEntry.h"
#include "protocol/WatchDog.h"

struct Point;
struct Line;

class FlarmScreen: public MenuEntry, public WDBark_I
{
public:
    static FlarmScreen *create();

    virtual void exit(int ups=1) override;
    void display(int mode = 0) override;
    const char *value() const override { return nullptr; }
    void rot(int count) override {}
    void press() override;
    void longPress() override { press(); }
    void draw();

protected:
    // warn timeout
    void barked() override;

private:
    FlarmScreen();
    virtual ~FlarmScreen() = default;
    FlarmScreen(const FlarmScreen&) = delete;
    FlarmScreen& operator=(const FlarmScreen&) = delete;
    int _tick = 0;
    int _alarmtick = 0;
    WatchDog_C _time_out;
    uint16_t _prev_alarm = 0;
};


// global access
extern FlarmScreen *FLARMSCREEN;