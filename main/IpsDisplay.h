
/*
 * IpsDisplay.h
 *
 *  Created on: Jan 25, 2018
 *      Author: iltis
 */

#pragma once

#include "esp_system.h"
#include "setup/SetupNG.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include <string>

typedef enum e_sreens { INIT_DISPLAY_NULL, INIT_DISPLAY_RETRO=1, INIT_DISPLAY_FLARM=2, INIT_DISPLAY_GLOAD=4, INIT_DISPLAY_HORIZON=8 } e_screens_t;
extern int screens_init;

class PolarGauge;
class WindIndicator;
class McCready;
class S2FBar;
class Battery;
class Altimeter;
class MultiGauge;
class CruiseStatus;
class FlapsBox;

// fixme needs a home
float getHeading();

class IpsDisplay {
public:
	IpsDisplay( AdaptUGC *aucg );
	virtual ~IpsDisplay();
	static void begin();
	static void bootDisplay();
	static void setGlobalColors();
	static void writeText( int line, const char *text );
	static void writeText( int line, std::string &text );
	                    //  TE,       aTE,       polar_sink,       alt, temperature, battery, s2f_delta, as2f, acl, wkf
	static void drawDisplay(float te, float ate, float polar_sink, float alt, float temperature, float volt, float s2fd, float s2f, float acl);

	static void drawLoadDisplay( float loadFactor );
	static void drawHorizon( float pitch, float roll, float yaw );
	static void drawLoadDisplayTexts();
	static void initDisplay();
	static void clear();   // erase whole display
	static void redrawValues();
	static void setBottomDirty();
	static void setCruiseChanged();

	static inline AdaptUGC *getDisplay() { return ucg; };
	static AdaptUGC *ucg;

    static PolarGauge *MAINgauge;
    static PolarGauge *WNDgauge;
    static McCready *MCgauge;
    static S2FBar *S2FBARgauge;
    static Battery *BATgauge;
    static Altimeter *ALTgauge;
    static MultiGauge *TOPgauge;
    static CruiseStatus *VCSTATgauge;
    static FlapsBox *FLAPSgauge;

  private:

	static int tick;

	// local variabls for dynamic display
	static int s2falt;
	static int s2fdalt;
	static bool wireless_alive;
	static int tempalt;

	static float average_climbf;

	static temp_status_t siliconTempStatusOld;

	static void drawBT();
	static void drawCable(int16_t x, int16_t y);
	static void drawWifi( int x, int y );
	static void drawConnection( int16_t x, int16_t y );
	static void drawTemperature( int x, int y, float t );

private:
	static void initLoadDisplay();
	static void drawAvg( float mps, float delta );
	static bool drawTopGauge(int val, int16_t x, int16_t y, bool inc_unit=false);
};

extern IpsDisplay *Display;
