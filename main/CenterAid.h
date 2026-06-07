/*
 * CenterAid.h
 *
 *  Created on: Jan 23, 2022
 *      Author: gittest3
 */
#pragma once

#include "wind/CircleWind.h"
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
	void tick(int tick);
	void drawCenterAid();
    void redrawAt(int deg);
    void setGeometry(int r);
	void setGliderOnTop(bool onTop) { _glider_on_top = onTop; }

private:
	CenterAid(PolarGauge &g);
	void drawThermal(const Thermal& th, int idir);
	void drawGlider(int16_t cx, int16_t cy);
	bool maxClimb();
	void checkThermal();
	void calcFlightMode( rad_t headingDiff );
	int maxClimbIndex();
    Point getThermalCG() const;

    const PolarGauge &_gauge;
	bool _glider_on_top; // circle aid reference on top, or 90° on the side.

	Thermal thermals[CA_NUM_DIRS] = {};  // every 15°: [m/s]
	rad_t cur_heading = 0.f;
	rad_t gps_heading = 0.f;
	rad_t gyro_last = 0.f;
	// int8_t gyro_foot_off; // difference from GPS to gyro heading
	int8_t _idir = 0;
	circling_t flightmode = circling_t::undefined;
	uint8_t turn_left = 0; // fixme flight status need to go central to a fusion processor using sensors as avail
	uint8_t turn_right = 0;
	uint8_t fly_straight = 0;
	uint32_t last_rts = 0;
	mps_t peak_value = 1.0; // we start with expectation of 1 m/s thermals
	static float th_norm;
};

extern CenterAid  *theCenteraid;
