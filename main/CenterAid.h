/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "sensor/Filters.h"
#include "math/Units.h"

#include <cstdint>

constexpr int CA_NUM_DIRS = 24;  // every 15°

class PolarGauge;
struct Point;

struct Thermal {
    mps_t    strength;
    uint32_t timestamp;
    constexpr Thermal() : strength(0.f), timestamp(0) {}
    void set(mps_t s);
    float getStrength() const;
};

class CenterAid {
    friend Thermal;
public:
    static CenterAid *create(PolarGauge &g);
    static void remove();
    void checkHeading(rad_t vheading, rad_t omega, rad_t bank); // 10Hz ticker
    void drawCenterAid();
    void setGliderOnTop(bool onTop) { _glider_on_top = onTop; }

private:
    CenterAid(PolarGauge &g);
    void drawThermal(const Thermal& th, int idir);
    void drawGlider(int16_t cx, int16_t cy);
    // int maxClimbIndex();
    Point getThermalCG() const;

    const PolarGauge &_gauge;
    bool _glider_on_top; // circle aid reference on top, or 90° on the side.

    Thermal thermals[CA_NUM_DIRS] = {};  // every 15°: [m/s]
    int8_t _idir = 0;
    uint8_t _cdir = 0; // right & left
    LowPassFilterT<float> _confidence; // 0..1, how confident we are that we are thermaling;
    rad_t cur_heading = 0.f; // accumulating heading delta and dice in e.g.15° steps
    mps_t _peak_value = .5f; // we start with expectation of .5 m/s thermals
    static float th_norm;
};

extern CenterAid  *theCenteraid;
