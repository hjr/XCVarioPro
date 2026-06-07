/*
 * CenterAid.cpp
 *
 *  Created on: Jan 23, 2022
 *      Author: Eckhard Völlm
 */

#include "CenterAid.h"

#include "math/Units.h"
#include "screen/element/PolarGauge.h"
#include "driver/time/Clock.h"
#include "math/Trigonometry.h"
#include "Flarm.h"
#include "setup/SetupNG.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "sensor/imu/AccMPU6050.h"
#include "logdefnone.h"

#include <algorithm>


constexpr rad_t CA_STEP = Units::deg_to_rad(360/CA_NUM_DIRS); // 15
constexpr rad_t CA_STEP_2 = CA_STEP/2.f;  // 7.5
constexpr int16_t MAX_DISK_RAD = 7;

extern AdaptUGC *MYUCG;
CenterAid  *theCenteraid = nullptr;

float CenterAid::th_norm = 1.0;

// the THermal helper struct
void Thermal::set(mps_t s) {
    strength = s;
    timestamp = Clock::getMillis();
}

// normalized strecngth -1 .. 0 .. 1
float Thermal::getStrength() const {
    // scale to peak value and limit to max disk radius
    // as well as fade with time passing
    float agescale = 1.f - (Clock::getMillis() - timestamp) / 60000.f; // fade out over 60 seconds
    if ( agescale < 0.f ) {
        return 0.f;
    }
    return std::min(strength / CenterAid::th_norm * agescale, 1.f);
}

// Thermal assistant
CenterAid *CenterAid::create(PolarGauge &g)
{
    if ( ! theCenteraid ) {
        // create the singleton
        theCenteraid = new CenterAid(g);
    }
    return theCenteraid;
}
void CenterAid::remove()
{
    if ( theCenteraid ) {
        // remove the singleton
        CenterAid *tmp = theCenteraid;
        theCenteraid = nullptr;
        delete tmp;
    }
}

CenterAid::CenterAid(PolarGauge &g) :
    _gauge(g),
    _glider_on_top(true)
{
    _glider_on_top = vario_centeraid.get() != 2;
}

void CenterAid::drawThermal(const Thermal& th, int idir) {
    // ESP_LOGI(FNAME,"drawThermal, tn: %d, idir: %d, ds: %d", tn, idir, draw_red );
    if (idir >= CA_NUM_DIRS || idir < 0) {
        ESP_LOGE(FNAME, "index out of range: %d", idir);
        return;
    }
    int ddir = idir;
    if (!_glider_on_top) {
        if (flightmode == circling_t::circlingR) {
            ddir = (idir + 3 * CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the left
        } else {
            ddir = (idir + CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the right
        }
    }
    int16_t cx = _gauge._ref.x + fast_sin_rad(ddir * CA_STEP) * _gauge._radius;
    int16_t cy = _gauge._ref.y - fast_cos_rad(ddir * CA_STEP) * _gauge._radius;

    // a green thermal spot that brightens with increasing thermal strength (up to 10%)
    ucg_color_t col = { COLOR_GREEN };
    float ths = th.getStrength();
    if ( ths >= 0.f ) {
        col.fadeTo(col, ths * 1.2f);
    }
    else {
        col = { COLOR_BLUE };
    }
    // ESP_LOGI(FNAME,"drawThermal, th: %.1f, ths: %.3f, color: %d,%d,%d", th.strength, ths, col.color[0], col.color[1], col.color[2]);
    MYUCG->setColor(col.color[0], col.color[1], col.color[2]);
    if (idir != 0) {
        MYUCG->startBuffering(cx-MAX_DISK_RAD, cy-MAX_DISK_RAD, 2*MAX_DISK_RAD, 2*MAX_DISK_RAD);
        // MYUCG->drawFrame(cx-MAX_DISK_RAD, cy-MAX_DISK_RAD, 2*MAX_DISK_RAD-1, 2*MAX_DISK_RAD-1);
        int16_t radius = std::clamp(static_cast<int16_t>(std::abs(ths) * MAX_DISK_RAD), (int16_t)1, MAX_DISK_RAD);
        MYUCG->drawDisc(cx, cy, radius, UCG_DRAW_ALL);
        MYUCG->finishBuffering();
    }
    else {
        // draw glider here icon
        drawGlider(cx, cy);
    }
}

//  x Cxy + (A,-W)
//  .  .
//  .     .
//  .         .
//  .  Cxy      x Cxy + (B,0)
//  .         .
//  .     .
//  .  .
//  x Cxy + (A,W)
//
void CenterAid::drawGlider(int16_t cx, int16_t cy) {
    constexpr int16_t A = -3;
    constexpr int16_t B = 3;
    constexpr int16_t W = 8;
    const int16_t triangle[3][6] = { { A, -W, A, W, B, 0}, {-A, -W, -B, 0, -A, W}, {-W, B, W, B, 0, A} }; // left, right, tail

    // select triangle for circling direction
    MYUCG->setColor(COLOR_WHITE);
    int16_t *trptr = (int16_t*)triangle[2];
    if (_glider_on_top) {
        MYUCG->startBuffering(cx-B, cy-W, 2*B+1, 2*W+1);
        // MYUCG->drawFrame(cx-B, cy-W, 2*B, 2*W);
        if ( flightmode == circling_t::circlingR )
            trptr = (int16_t*)triangle[0];
        else {
            trptr = (int16_t*)triangle[1];
        }
    } else {
        MYUCG->startBuffering(cx-W, cy-B, 2*W+1, 2*B+1);
        // MYUCG->drawFrame(cx-W, cy-B, 2*W, 2*B);
    }
    if ( flightmode == circling_t::circlingR || flightmode == circling_t::circlingL ) {
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->drawTriangle(cx + trptr[0], cy + trptr[1], cx + trptr[2], cy + trptr[3], cx + trptr[4], cy + trptr[5]);
    }
    MYUCG->finishBuffering();
}

int CenterAid::maxClimbIndex(){
	int max=0;
	int max_index = -1;
	for( int i=0; i<CA_NUM_DIRS; i++ ){
		if( max < thermals[i].strength ){
			max = thermals[i].strength;
			max_index = i;
		}
	}
	return max_index;
}

Point CenterAid::getThermalCG() const {
    float sx = 0.0f, sy = 0.0f;
    float sum = 0.f;

    for (int i = 0; i < 24; i++) {
        float w = thermals[i].strength;
        sum += w;
        sx += w * fast_cos_idx(i * 360/CA_NUM_DIRS*2);
        sy += w * fast_sin_idx(i * 360/CA_NUM_DIRS*2);
    }

    float magnitude = sqrtf(sx*sx + sy*sy);
    float eccentricity = magnitude / sum;  // 0.0 .. ~1.0
    float roundness = 1.0f - eccentricity;


    float angle = atan2f(sy, sx);
    return Point(sx, sy);
}

void CenterAid::checkThermal(){
	// ESP_LOGI(FNAME,"checkThermal");
    // clalc new idir from current heading
    int8_t new_idir = (int)(cur_heading * ((float)CA_NUM_DIRS / PI2f)) % CA_NUM_DIRS;
    if ( new_idir == _idir ) {
        return; // no change, nothing to do
    }
	_idir = new_idir;
	mps_t te = te_vario.get();
	if( te > peak_value  ) {
		peak_value += (te - peak_value)*0.1;  // a bit low passing to catch values out of the row
    }
	if( peak_value > 1.0 ) {              // don't go below 1 m/s this is maximum sensitivity
		peak_value = peak_value * 0.999;  // Peak value aging 0.1% per 100 msec or 1% per second
    }
	// ESP_LOGI(FNAME,"newThermal dir:%d, TE:%.2f Peak:%.2f TI:%d", _idir, th, peak_value, ti  );
    thermals[_idir].set(te);
}

void CenterAid::drawCenterAid(){
	// ESP_LOGI(FNAME,"drawCenterAid");
	// int maxIndex = maxClimbIndex();
    // recalc the current thermal norm for the current peak value
    th_norm = peak_value;
    ESP_LOGI(FNAME,"CenterAid draw, peak norm: %.2f", th_norm);
	for( int i=0; i<CA_NUM_DIRS; i++ ){
		int d = (i+_idir) % CA_NUM_DIRS;
		// ESP_LOGI(FNAME,"dir:%d TE:%.1f", d, thermals[d].strength );
		drawThermal( thermals[d], i );
	}
}

// void CenterAid::redrawAt(int deg) {
//     int i = (deg * CA_NUM_DIRS / 360) % CA_NUM_DIRS;
//     int d = (i+idir) % CA_NUM_DIRS;
//     drawThermal(thermals[d], i);
//     i = (i + CA_NUM_DIRS / 2) % CA_NUM_DIRS; // opposite direction
//     d = (i+idir) % CA_NUM_DIRS;
//     drawThermal(thermals[d], i);
// }


void CenterAid::calcFlightMode( rad_t headingDiff ){
	// ESP_LOGI(FNAME,"calcFlightMode cur_head: %.1f hdiff:%.1f gs:%.0f GPSS:%d MH:%.1f FM:%d", cur_heading, headingDiff, tas.get(), Flarm::gpsStatus(), mag_hdt.get(), flightmode  );
	if( ! airborne.get() ){
		if( flightmode != circling_t::undefined ) {
			flightmode = circling_t::undefined;
			ESP_LOGI(FNAME,"New fm: undefined, no AS");
		}
	}
	else{
		if( headingDiff > Units::deg_to_rad(4.f) ){
            if( turn_right < 4 ) {
                turn_right++;
            }
            fly_straight = 0;
            if( flightmode != circling_t::circlingR && turn_right > 2 ) {
                flightmode = circling_t::circlingR;
                ESP_LOGI(FNAME,"New fm: circle right");
            }
		}
		else if( headingDiff < -Units::deg_to_rad(4.f) ) {
			if( turn_left < 4 ) {
				turn_left++;
            }
			fly_straight = 0;
			if( flightmode != circling_t::circlingL && turn_left > 2 ){
				flightmode = circling_t::circlingL;
				ESP_LOGI(FNAME,"New fm: circle left");
			}
		}
		else{
			turn_left = turn_right = 0;
			if( fly_straight < 4 ) {
				fly_straight++;
            }
			if( flightmode != circling_t::straight && fly_straight > 2 ) {
				flightmode = circling_t::straight;
				ESP_LOGI(FNAME,"New fm: straight");
			}
		}
	}
}

// 2 Hz called from sensor loop
// > tick : a 10 Hz counter
void CenterAid::tick(int tick){
    rad_t new_heading = -1.0;
    // if( mag_hdt.getValid() ) { // this is the best source for a heading, use this when avail
    //     new_heading = mag_hdt.get();
    // }
    if( new_heading < 0.f )  {         // fall back to GPS course and fuse gps heading with gyro
        rad_t gyro_footing = accSensor ? accSensor->getGyroFooting() : 0.f;
        // ESP_LOGI(FNAME,"COD %.1f", Units::rad_to_deg(gyro_footing) );
        if( false ) { // Flarm::gpsStatus() ){
            if( gyro_last == 0 ){
                gyro_last = gyro_footing;
            }
            rad_t gpshead = gnd_course.get();
            rad_t gyro = gyro_footing;
            rad_t gyro_delta =  gyro - gyro_last;
            gyro_last = gyro;
            rad_t diff = Vector::angleDiff( gpshead, gps_heading );
            gps_heading += (diff*0.01 + gyro_delta*1.07);
            new_heading = Vector::normalizePI2( gps_heading );
            // ESP_LOGI(FNAME,"GPS OK TC:%f gdY:%f fused:%f diff:%f", gpshead, gyro_delta, new_heading, diff );
        }else{     // trust as last resort just only gyro for Center Aid
            new_heading = gyro_footing;
            // ESP_LOGI(FNAME,"Gyro yaw %f", new_heading);
        }
        ESP_LOGI(FNAME,"NH %f", Units::rad_to_deg(new_heading) );
    }
    rad_t diff = Vector::angleDiff( new_heading, cur_heading );
    ESP_LOGI(FNAME,"new heading %.1f diff:%.1f", Units::rad_to_deg(new_heading), Units::rad_to_deg(diff) );
    if( new_heading != cur_heading ){
        uint32_t rts = Clock::getMillis();
        float dt = (float)(rts - last_rts)/1000.0f;
        last_rts = rts;
        calcFlightMode( diff/dt ); // we calculate own flight mode here
        cur_heading = new_heading;
    }
    checkThermal();
}
