/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlarmScreen.h"

#include "Colors.h"
#include "UiEvents.h"
#include "IpsDisplay.h"
#include "DrawDisplay.h"
#include "ESPAudio.h"
#include "KalmanMPU6050.h"
#include "Flarm.h"
#include "math/Trigonometry.h"
#include "math/Floats.h"
#include "math/Quaternion.h"
#include "vector_3d.h"
#include "AdaptUGC.h"
#include "logdefnone.h"


extern AdaptUGC *MYUCG;
FlarmScreen *FLARMSCREEN = nullptr;
constexpr int FLARM_ALARM_HOLDING_TIME = 7000; // msec


FlarmScreen *FlarmScreen::create()
{
    if ( ! FLARMSCREEN ) {
        FLARMSCREEN = new FlarmScreen();
    }
    return FLARMSCREEN;
}

FlarmScreen::FlarmScreen() :
    MenuEntry(),
    _time_out( this )
{
    ESP_LOGI(FNAME,"FlarmScreen created");
    _time_out.start( FLARM_ALARM_HOLDING_TIME );
    tick = 0;
}

void FlarmScreen::exit(int ups)
{
    ESP_LOGI(FNAME,"FlarmScreen exit");
    FLARMSCREEN = nullptr;
    MenuEntry::exit();
    if ( ! isRoot() ) {
        delete this;
    }
}

void FlarmScreen::display(int mode)
{
    if ( mode == 0 ) {
        Display->clear();
    }
    _time_out.pet();

    ESP_LOGI(FNAME, "flarm_screen mode %d", mode);
    // Draw horizon
    Quaternion attq = IMU::getAHRSQuaternion();
    float roll = IMU::getRollRad();
    Line l( attq, dwidth/2, dheight/2 );
    Point above[6], below[6];
    int na, nb;
    IpsDisplay::clipRectByLine(nullptr, l, above, &na, below, &nb);
    MYUCG->setColor( COLOR_SKYBLUE );
    IpsDisplay::drawPolygon(above, na);
    MYUCG->setColor( COLOR_EARTH );
    IpsDisplay::drawPolygon(below, nb);

    ESP_LOGI(FNAME,"Target in B%d°, dH%dm, dV%dm", Flarm::RelativeBearing, Flarm::RelativeDistance, Flarm::RelativeVertical );
    // calc from distance and bearing the vector to the target
    Quaternion qtmp(deg2rad(static_cast<float>(-Flarm::RelativeBearing)), vector_ijk(0.f, 0.f, 1.f));
    vector_ijk bearingVec = qtmp * vector_ijk(Flarm::RelativeDistance, 0.f, Flarm::RelativeVertical);
    ESP_LOGI(FNAME,"BearingVec %1.1f,%1.1f,%1.1f", bearingVec.x, bearingVec.y, bearingVec.z );

    // rotate according to own attitude
    vector_ijk buddyVec = attq * bearingVec;
    ESP_LOGI(FNAME,"BuddyVec %1.1f,%1.1f,%1.1f", buddyVec.x, buddyVec.y, buddyVec.z );
    Point p = IpsDisplay::projectToDisplayPlane(buddyVec, 100.f);
    ESP_LOGI(FNAME,"DisplPt %d,%d", p.x, p.y);
    // clip to display area
    p = IpsDisplay::clipToScreenCenter(p);
    ESP_LOGI(FNAME,"ClippPt %d,%d", p.x, p.y);

    // Put alarm level into color
    if ( Flarm::AlarmLevel == 1 ) {
        MYUCG->setColor(COLOR_LGREY);
    }
    else if ( Flarm::AlarmLevel == 2 ) {
        MYUCG->setColor(COLOR_YELLOW);
    }
    else if ( Flarm::AlarmLevel >= 3 ) {
        MYUCG->setColor(COLOR_RED);
    }
    // Draw target as a  disc, depict the distance through the size
    int16_t size = (300 - Flarm::RelativeDistance) / 5;
    if ( size < 15 ) {
        size = 15;
    }
    // Embedd a little glider symbol
    MYUCG->drawDisc( p.x, p.y, size, UCG_DRAW_ALL);
    if ( buddyVec.x > 0.f ) {
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->drawDisc( p.x, p.y, size/3, UCG_DRAW_ALL);
        // draw little wings as tetragon rolled according to horizontal angle
        Point w1 = Point( size, 0 ).rotate(-roll);
        Point th = Point(0, size/6).rotate(-roll);
        MYUCG->drawTetragon( p.x + w1.x + th.x, p.y + w1.y + th.y,
                             p.x - w1.x + th.x, p.y - w1.y + th.y,
                             p.x - w1.x - th.x, p.y - w1.y - th.y,
                             p.x + w1.x - th.x, p.y + w1.y - th.y );
        p += Point(0, -size * 2 / 3).rotate(-roll);
        w1 = Point(size / 3, 0).rotate(-roll);
        MYUCG->drawTetragon( p.x + w1.x + th.x, p.y + w1.y + th.y,
                             p.x - w1.x + th.x, p.y - w1.y + th.y,
                             p.x - w1.x, p.y - w1.y,
                             p.x + w1.x, p.y + w1.y);

    }
    else {
        MYUCG->setColor(COLOR_BLACK);
        // target is behind -> draw a black X and a circle around the disc
        MYUCG->drawCircle( p.x, p.y, size, UCG_DRAW_ALL);
        // Draw a diagonal X line
        Point x1 = Point(size, size).rotate(-roll);
        MYUCG->drawLine( p.x - x1.x, p.y - x1.y, p.x + x1.x, p.y + x1.y );
        MYUCG->drawLine( p.x - x1.y, p.y + x1.x, p.x + x1.y, p.y - x1.x );
    }
}

void FlarmScreen::press() {
    ESP_LOGI(FNAME,"FlarmScreen press - exit");
    exit();
}

void FlarmScreen::goOn()
{
    _time_out.pet();
}

void FlarmScreen::draw()
{
    ESP_LOGI(FNAME,"FlarmScreen draw");
    _time_out.pet();
    tick++;
    MYUCG->setFont(ucg_font_fub11_tr);
    MYUCG->setPrintPos(100, 160);
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->print(tick);
}


void FlarmScreen::barked()
{
    ESP_LOGI(FNAME,"FlarmScreen barked - timeout");
    // timeout
    int exitWarn = ButtonEvent(ButtonEvent::SHORT_PRESS).raw;
    // Route this event to the DrawDisplay context
    xQueueSend(uiEventQueue, &exitWarn, 0);
}