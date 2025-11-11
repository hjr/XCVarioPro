/*
 * IpsDisplay.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: iltis
 *
 */

#include "IpsDisplay.h"

#include "math/Floats.h"
#include "screen/element/MultiGauge.h"
#include "screen/element/PolarGauge.h"
#include "screen/element/WindIndicator.h"
#include "screen/element/McCready.h"
#include "screen/element/S2FBar.h"
#include "screen/element/Battery.h"
#include "screen/element/Altimeter.h"
#include "screen/element/CruiseStatus.h"
#include "screen/element/FlapsBox.h"

#include "math/Trigenometry.h"
#include "comm/DeviceMgr.h"
#include "BLESender.h"
#include "OneWireESP32.h"
#include "sensor.h"
#include "Units.h"
#include "Flap.h"
#include "Flarm.h"
#include "setup/CruiseMode.h"
#include "wind/StraightWind.h"
#include "wind/CircleWind.h"
#include "comm/WifiApSta.h"
#include "comm/BTspp.h"
#include "comm/CanBus.h"
#include "protocol/AliveMonitor.h"
#include "setup/SetupNG.h"
#include "CenterAid.h"
#include "Rotate.h"
#include "AdaptUGC.h"
#include "logdefnone.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>


////////////////////////////
// types


// local variables
int screens_init = INIT_DISPLAY_NULL;

int   IpsDisplay::tick = 0;


// Average Vario data
PolarGauge* IpsDisplay::MAINgauge = nullptr;
PolarGauge* IpsDisplay::WNDgauge = nullptr;
McCready*   IpsDisplay::MCgauge = nullptr;
S2FBar*     IpsDisplay::S2FBARgauge = nullptr;
Battery*    IpsDisplay::BATgauge = nullptr;
Altimeter*	IpsDisplay::ALTgauge = nullptr;
MultiGauge*	IpsDisplay::TOPgauge = nullptr;
CruiseStatus* IpsDisplay::VCSTATgauge = nullptr;
FlapsBox*   IpsDisplay::FLAPSgauge = nullptr;

int16_t DISPLAY_H;
int16_t DISPLAY_W;

// u8g2_t IpsDisplay::u8g2c;

const int   dmid = 160;   // display middle
const int   bw    = 32;   // bar width

#define TRISIZE 15

#define FIELD_START 85
#define FIELD_START_UL 170
#define SIGNLEN 24+4
#define GAP 12

#define HEADFONTH 14
#define VARFONTH  54 // fub35_hn
#define YVAR HEADFONTH+VARFONTH
#define YVARMID (YVAR - (VARFONTH/2))

#define S2FFONTH 35 //31
#define YS2F YVAR+S2FFONTH+HEADFONTH+GAP-8


#define VARBARGAP (HEADFONTH+(HEADFONTH/2)+2)
#define MAXTEBAR ((DISPLAY_H-(VARBARGAP*2))/2)

#define BTSIZE  5
#define BTW    15
#define BTH    24
#define ASVALX 163

#define FLOGO  24

int S2FST = 45;

#define UNITVAR (vario_unit.get())
#define UNITAS (ias_unit.get())
#define UNITALT (alt_unit.get())

IpsDisplay *Display = nullptr;

static int AMIDY;
static int AMIDX;
static int AVGOFFX;
static int SPEEDYPOS;
constexpr const float OPT_Y_IN = 0.262f;

static int16_t INNER_RIGHT_ALIGN = 170;
static int16_t LOAD_MPG_POS = 0;
static int16_t LOAD_MIAS_POS = 0;

AdaptUGC *IpsDisplay::ucg = 0;

static int _ate = -1000;
int IpsDisplay::s2falt=-1;
int IpsDisplay::s2fdalt=0;
bool IpsDisplay::wireless_alive = false;
int IpsDisplay::tempalt = -2000;

temp_status_t IpsDisplay::siliconTempStatusOld = MPU_T_UNKNOWN;

// constexpr float sincosScale = 180.f/My_PIf*2.f; // rad -> deg/2 a 0.5deg resolution discrete scale
static int16_t old_vario_bar_val = 0;
static int16_t old_sink_bar_val = 0;

static bool bottom_dirty = false;
static bool mode_dirty = false;

bool flarm_connected=false;

static void initRefs()
{
	AVGOFFX = -5-38;
	SPEEDYPOS = OPT_Y_IN * DISPLAY_H + 19;
	INNER_RIGHT_ALIGN = DISPLAY_W - 44;
	LOAD_MPG_POS = DISPLAY_H*0.3;
	LOAD_MIAS_POS = DISPLAY_H*0.7;

	// grab screen layout
	AMIDX = (DISPLAY_W/2 + 30);
	AMIDY = (DISPLAY_H)/2;
	if ( display_orientation.get() == DISPLAY_NINETY ) {
		INNER_RIGHT_ALIGN = DISPLAY_W - 74;
		AMIDX = DISPLAY_W/2 - 43;
		AVGOFFX = -2;
	}
}

////////////////////////////
// IpsDisplay implementation
IpsDisplay::IpsDisplay( AdaptUGC *aucg ) {
	ucg = aucg;
	tick = 0;
	DISPLAY_W = ucg->getDisplayWidth();
	DISPLAY_H = ucg->getDisplayHeight();
}

IpsDisplay::~IpsDisplay() {
    if (MAINgauge) {
        delete MAINgauge;
        MAINgauge = nullptr;
    }
    if (WNDgauge) {
        delete WNDgauge;
        WNDgauge = nullptr;
    }
    if (MCgauge) {
        delete MCgauge;
        MCgauge = nullptr;
    }
    if (S2FBARgauge) {
        delete S2FBARgauge;
        S2FBARgauge = nullptr;
    }
    if (BATgauge) {
        delete BATgauge;
        BATgauge = nullptr;
    }
    if (ALTgauge) {
        delete ALTgauge;
        ALTgauge = nullptr;
    }
    if (TOPgauge) {
        delete TOPgauge;
        TOPgauge = nullptr;
    }
    if (VCSTATgauge) {
        delete VCSTATgauge;
        VCSTATgauge = nullptr;
    }
    if (FLAPSgauge) {
        delete FLAPSgauge;
        FLAPSgauge = nullptr;
    }
}

void IpsDisplay::writeText( int line, const char *text )
{
	ucg->setFont(ucg_font_ncenR14_hr, true );
	ucg->setPrintPos( 1, 26*line );
	ucg->setColor(COLOR_WHITE);
	ucg->printf("%s", text);
}

void IpsDisplay::writeText( int line, std::string &text ){
	ucg->setFont(ucg_font_ncenR14_hr, true );
	ucg->setPrintPos( 1, 26*line );
	ucg->setColor(COLOR_WHITE);
	ucg->printf("%s",text.c_str());
}


void IpsDisplay::clear(){
	// ESP_LOGI(FNAME,"display clear()");
	ucg->setColor( COLOR_BLACK );
	ucg->drawBox( 0,0,DISPLAY_W,DISPLAY_H );
	screens_init = INIT_DISPLAY_NULL;
}

void IpsDisplay::bootDisplay() {
	// ESP_LOGI(FNAME,"IpsDisplay::bootDisplay()");
	if( display_type.get() == ST7789_2INCH_12P )
		ucg->setRedBlueTwist( true );
	if( display_type.get() == ILI9341_TFT_18P )
		ucg->invertDisplay( true );
	// ESP_LOGI(FNAME,"clear boot");
    setGlobalColors();
	ucg->setColor(1, COLOR_BLACK );
	ucg->setColor(0, COLOR_WHITE );
	clear();
	ucg->setFont(ucg_font_fub11_tr);
}

void IpsDisplay::setGlobalColors() {
    // set global color variables according to selected display_variant
	if ( display_variant.get() == DISPLAY_WHITE_ON_BLACK ) {
		g_col_background = 255;
		g_col_highlight = 0;
		g_col_header_r=179;
		g_col_header_g=171;
		g_col_header_b=164;
		g_col_header_light_r=94;
		g_col_header_light_g=87;
		g_col_header_light_b=0;
	}
	else {
		g_col_background = 0;
		g_col_highlight = 255;
		g_col_header_r=179;
		g_col_header_g=171;
		g_col_header_b=164;
		g_col_header_light_r=161;
		g_col_header_light_g=168;
		g_col_header_light_b=255;
	}
}

void IpsDisplay::initDisplay() {
	// ESP_LOGI(FNAME,"IpsDisplay::initDisplay()");
    setGlobalColors();
    clear();

	// Create common elements
    initRefs();

    if (!MAINgauge) { // shared with
        int16_t scale_geometry = (display_orientation.get() == DISPLAY_NINETY) ? 120 : 90;
        MAINgauge = new PolarGauge(AMIDX, AMIDY, scale_geometry, DISPLAY_H / 2 - 20, PolarGauge::VARIO);
    }
    MAINgauge->setUnit(Units::Vario(1.));
    MAINgauge->setRange(scale_range.get(), 0.f, log_scale.get());
    MAINgauge->setColor(VN_COLOR_RED); // fixme temp needle_color.get());
    if (vario_mc_gauge.get()) {
        if ( !MCgauge ) {
            MCgauge = new McCready(1, DISPLAY_H + 2);
        }
        if ( !S2FBARgauge) {
            S2FBARgauge = new S2FBar(DISPLAY_W - 50, AMIDY, 28, 32);
        }
   }
    else {
        if ( MCgauge ) {
            delete MCgauge;
            MCgauge = nullptr;
        }
        if ( S2FBARgauge) {
            delete S2FBARgauge;
            S2FBARgauge = nullptr;
        }
    }
    if (!BATgauge) {
        BATgauge = new Battery(DISPLAY_W - 10, DISPLAY_H - 12);
    }
    if ( !VCSTATgauge ) {
        VCSTATgauge = new CruiseStatus(INNER_RIGHT_ALIGN - 8, 18);
    }
    if ( FLAP ) {
        if (!FLAPSgauge) {
            FLAPSgauge = new FlapsBox(FLAP, DISPLAY_W - 28, AMIDY, DISPLAY_H > DISPLAY_W);
        }
    }
    else {
        if ( FLAPSgauge ) {
            delete FLAPSgauge;
            FLAPSgauge = nullptr;
        }
    }

    ucg->setFontPosBottom();

    MAINgauge->drawScale();
    MAINgauge->forceAllRedraw();
    MAINgauge->setFigOffset(AVGOFFX, 0);

    if (!WNDgauge) {
        // create it always, because also the center aid is using it (keep it simple here)
        WNDgauge = new PolarGauge(AMIDX + AVGOFFX, AMIDY, 360, 50, PolarGauge::COMPASS);
    }
    WNDgauge->enableWindIndicator(wind_enable.get() > WA_OFF, wind_enable.get() == WA_EXTERNAL);
    WNDgauge->setWindRef(wind_reference.get());
    WNDgauge->setColor(needle_color.get());

    if (vario_centeraid.get()) {
        CenterAid::create(*WNDgauge);
    } else {
        CenterAid::remove();
    }

    VCSTATgauge->useSymbol(true);
    if (MCgauge) {
        MCgauge->setLarge(true);
    }

    if (vario_lower_gauge.get()) {
        if (!ALTgauge) {
            ALTgauge = new Altimeter(INNER_RIGHT_ALIGN, (1. - OPT_Y_IN) * DISPLAY_H + 19);
        }
    } else {
        if (ALTgauge) {
            delete ALTgauge;
            ALTgauge = nullptr;
        }
    }
    if (vario_upper_gauge.get()) {
        if (!TOPgauge) {
            TOPgauge = new MultiGauge(INNER_RIGHT_ALIGN, SPEEDYPOS, (MultiGauge::MultiDisplay)vario_upper_gauge.get());
        } else {
            TOPgauge->setDisplay((MultiGauge::MultiDisplay)(vario_upper_gauge.get()));
        }
    } else {
        if (TOPgauge) {
            delete TOPgauge;
            TOPgauge = nullptr;
        }
    }
    if (S2FBARgauge) {
        if (display_orientation.get() == DISPLAY_NINETY) {
            S2FBARgauge->setRef(DISPLAY_W - 120, AMIDY);
            S2FBARgauge->setWidth(36);
            S2FBARgauge->setGap(2);
        } else {
            if (FLAPSgauge) {
                S2FBARgauge->setRef(DISPLAY_W - 50, AMIDY);
                S2FBARgauge->setWidth(28);
            } else {
                S2FBARgauge->setRef(DISPLAY_W - 34, AMIDY);
                S2FBARgauge->setWidth(50);
            }
            S2FBARgauge->setGap(32);
        }
    }
    if (FLAPSgauge) {
        if (display_orientation.get() == DISPLAY_NINETY) {
            FLAPSgauge->setLength(120);
        } else {
            FLAPSgauge->setLength(100);
        }
    }

    // Unit's
    ucg->setFont(ucg_font_fub11_hr);
    ucg->setPrintPos(2, 50);
    ucg->setColor(COLOR_HEADER);
    ucg->print(Units::VarioUnit());
    if (TOPgauge) {
        TOPgauge->drawUnit();
    }

    if (FLAPSgauge) {
        FLAPSgauge->forceRedraw();
    }

    redrawValues();
}

void IpsDisplay::begin() {
	ESP_LOGI(FNAME,"IpsDisplay::begin");
	ucg->begin();
}


void IpsDisplay::redrawValues()
{
	// ESP_LOGI(FNAME,"IpsDisplay::redrawValues()");
	tempalt = -2000;
	s2falt = -1;
	s2fdalt = -1;
	wireless_alive = false;
    if (MCgauge) {
        MCgauge->forceRedraw();
    }
    BATgauge->forceRedraw();
    if (ALTgauge) {
        ALTgauge->forceRedraw();
    }
    if (TOPgauge) {
        TOPgauge->forceRedraw();
    }
    if (S2FBARgauge) {
        S2FBARgauge->forceRedraw();
    }
    mode_dirty = true;

	if ( FLAPSgauge ) FLAPSgauge->forceRedraw();
	old_vario_bar_val = 0;
	old_sink_bar_val = 0;
    _ate = -1000;
}

void IpsDisplay::drawBT() {
	bool bta=true;
	if( DEVMAN->isIntf(BT_SPP) && BTspp )
		bta = BTspp->isConnected();
	else if( DEVMAN->isIntf(BT_LE) )
		bta=BLESender::queueFull() ? false : true;
	if( bta != wireless_alive || flarm_alive.get() > ALIVE_NONE ) {
		int16_t btx=DISPLAY_W-18;
		int16_t bty=(BTH/2) + 6;
		if( ! bta )
			ucg->setColor( COLOR_MGREY );
		else
			ucg->setColor( COLOR_BLUE );  // blue

		ucg->drawRBox( btx-BTW/2, bty-BTH/2, BTW, BTH, BTW/2-1);
		// inner symbol
		if( flarm_alive.get() == ALIVE_OK )
			ucg->setColor( COLOR_GREEN );
		else
			ucg->setColor( COLOR_WHITE );
		ucg->drawTriangle( btx, bty, btx+BTSIZE, bty-BTSIZE, btx, bty-2*BTSIZE );
		ucg->drawTriangle( btx, bty, btx+BTSIZE, bty+BTSIZE, btx, bty+2*BTSIZE );
		ucg->drawLine( btx, bty, btx-BTSIZE, bty-BTSIZE );
		ucg->drawLine( btx, bty, btx-BTSIZE, bty+BTSIZE );

		wireless_alive = bta;
		flarm_connected = flarm_alive.get();
	}
	if( SetupCommon::isWired() ) {
		drawCable(DISPLAY_W-20, BTH + 22);
	}
}

void IpsDisplay::drawCable(int16_t x, int16_t y)
{
	const int16_t CANH = 8;
	const int16_t CANW = 14;

	int connectedXCV = xcv_alive.get();
	int connectedMag = mags_alive.get();

	(connectedXCV == ALIVE_OK)? ucg->setColor(COLOR_LBLUE) : ucg->setColor(COLOR_MGREY);
	// lower horizontal line
	if (connectedMag) {
		ucg->setColor(COLOR_GREEN);
	}
	ucg->drawLine( x-CANW/2, y+CANH/2, x+3, y+CANH/2 );
	ucg->drawLine( x-CANW/2, y+CANH/2-1, x+3, y+CANH/2-1 );
	ucg->drawDisc( x-CANW/2, y+CANH/2, 2, UCG_DRAW_ALL);
	(connectedMag == ALIVE_OK)? ucg->setColor(COLOR_LBLUE) : ucg->setColor(COLOR_MGREY);
	// Z diagonal line
	if (flarm_alive.get() == ALIVE_OK) { ucg->setColor(COLOR_GREEN); }
	ucg->drawLine( x+2, y+CANH/2, x-4, y-CANH/2 );
	ucg->drawLine( x+3, y+CANH/2-1, x-3, y-CANH/2-1 );
	// upper horizontal line
	(connectedXCV == ALIVE_OK)? ucg->setColor(COLOR_LBLUE) : ucg->setColor(COLOR_MGREY);
	ucg->drawLine( x-3, y-CANH/2, x+CANW/2, y-CANH/2 );
	ucg->drawLine( x-3, y-CANH/2-1, x+CANW/2, y-CANH/2-1 );
	ucg->drawDisc( x+CANW/2, y-CANH/2, 2, UCG_DRAW_ALL);
}

void IpsDisplay::drawWifi( int x, int y ) {
	if( !DEVMAN->isIntf(WIFI_APSTA) ) {
		return;
	}
	bool wla = WIFI->isAlive();
	if( wla != wireless_alive || flarm_alive.get() > ALIVE_NONE ){
		ESP_LOGI(FNAME,"IpsDisplay::drawWifi %d %d %d", x,y,wla);
		if( ! wla ) {
			ucg->setColor(COLOR_MGREY);
		} else {
			ucg->setColor( COLOR_BLUE );
		}
		ucg->drawCircle( x, y, 9, UCG_DRAW_UPPER_RIGHT);
		ucg->drawCircle( x, y, 10, UCG_DRAW_UPPER_RIGHT);
		ucg->drawCircle( x, y, 16, UCG_DRAW_UPPER_RIGHT);
		ucg->drawCircle( x, y, 17, UCG_DRAW_UPPER_RIGHT);
		if( flarm_alive.get() == ALIVE_OK ) {
			ucg->setColor( COLOR_GREEN );
		}
		ucg->drawDisc( x, y, 3, UCG_DRAW_ALL );
		flarm_connected = flarm_alive.get();
		wireless_alive = wla;
	}
	if( SetupCommon::isWired() ) {
		drawCable(DISPLAY_W-20, y+18);
	}
}

void IpsDisplay::drawConnection( int16_t x, int16_t y )
{
	if ( DEVMAN->isIntf(BT_SPP) || DEVMAN->isIntf(BT_LE) ) {
		drawBT();
	}
	else if( DEVMAN->isIntf(WIFI_APSTA) ) {
		drawWifi(x, y);
	}
	else if( SetupCommon::isWired() ) {
		drawCable(DISPLAY_W-18, y);
	}
}


// accept temperature in deg C and display in configured unit
// right-aligned value to x, incl. unit right of x
void IpsDisplay::drawTemperature( int x, int y, float t ) {
	ucg->setFont(ucg_font_fub14_hn, false);
	char s[32];
	if( t != DEVICE_DISCONNECTED_C ) {
		float temp_unit = Units::TemperatureUnit( t );
		sprintf(s, "%.1f ", std::roundf(temp_unit*10.f)/10.f );
	}
	else {
		strcpy(s, "---");
	}
	// ESP_LOGI(FNAME,"drawTemperature: %d,%d %s", x, y, s);
	ucg->setColor( COLOR_WHITE );
	ucg->setPrintPos(x,y-3);
	ucg->print(s);
	if( HAS_MPU_TEMP_CONTROL ){   // Color if T unit shows if MPU silicon temperature is locked, too high or too low
		switch( MPU.getSiliconTempStatus() ){
		case MPU_T_LOCKED:
			ucg->setColor( COLOR_HEADER );
			break;
		case MPU_T_LOW:
			ucg->setColor( COLOR_LBLUE );
			break;
		case MPU_T_HIGH:
			ucg->setColor( COLOR_RED );
			break;
		default:
			ucg->setColor( COLOR_HEADER );
		}
	}else{
		ucg->setColor( COLOR_HEADER );
	}
	ucg->setFont(ucg_font_fub11_hn, false);
	ucg->setPrintPos(x+ucg->getStrWidth(s)+2,y-3);
	ucg->printf("%s ", Units::TemperatureUnitStr(temperature_unit.get()));
}


void IpsDisplay::setBottomDirty()
{
    bottom_dirty = true;
}
void IpsDisplay::setCruiseChanged()
{
    mode_dirty = true;
}


//////////////////////////////////////////////
// The load display

static float old_gmax = 100;
static float old_gmin = -100;
static float old_ias_max = -1;

void IpsDisplay::drawLoadDisplayTexts(){
	ucg->setFont(ucg_font_fub11_hr, true);
	const char *text = "ext. G-Loads";
	int16_t text_width = ucg->getStrWidth( text );
	ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MPG_POS);
	ucg->setColor( COLOR_HEADER_LIGHT );
	ucg->print( text );
	// ucg->setPrintPos(INNER_RIGHT_ALIGN-60, LOAD_MNG_POS);
	// ucg->print( "MAX NEG G" );
	text = "max. IAS";
	text_width = ucg->getStrWidth( text );
	ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MIAS_POS);
	ucg->print( text );
}

void IpsDisplay::initLoadDisplay(){
	clear();
	ESP_LOGI(FNAME,"initLoadDisplay()");
	ucg->setColor( COLOR_HEADER );
	ucg->setFont(ucg_font_fub11_hr);
	ucg->setPrintPos(20,20);
	ucg->print("G-Force");
	drawLoadDisplayTexts();
	int max_gscale = gload_pos_limit.get() + 2;
	int min_gscale = gload_neg_limit.get() - 2;
	if ( ! MAINgauge ) { // shared with
		int16_t scale_geometry = ( display_orientation.get() == DISPLAY_NINETY ) ? 120 : 90;
		MAINgauge = new PolarGauge(AMIDX, AMIDY, scale_geometry, DISPLAY_H/2-20, PolarGauge::GLOAD);
	}
	MAINgauge->setFigOffset(0, 0);
	MAINgauge->setUnit(1.);
	MAINgauge->setRange(max_gscale, 1.f, false);
	MAINgauge->setColor(VN_COLOR_RED); // temp fixme needle_color.get());
	// put the scale colored section into the background
	MAINgauge->colorRange(gload_pos_limit_low.get(), gload_pos_limit.get(), PolarGauge::ORANGE);
	MAINgauge->colorRange(gload_pos_limit.get(), max_gscale, PolarGauge::RED);
	MAINgauge->colorRange(gload_neg_limit_low.get(), gload_neg_limit.get(), PolarGauge::ORANGE);
	MAINgauge->colorRange(gload_neg_limit.get(), min_gscale, PolarGauge::RED);
	MAINgauge->drawScale();
	MAINgauge->forceAllRedraw();
	old_gmax = 100;
	old_gmin = -100;
	old_ias_max = -1;
	bottom_dirty = false;
	ESP_LOGI(FNAME,"initLoadDisplay end");
}

static Point P1o;
static Point P2o;
static Point P3o;
static Point P4o;
static Point P5o;
static Point P6o;

static float oroll=0;
static int heading_old = -1;

void IpsDisplay::drawHorizon( float pitch, float roll, float yaw ){
	// ESP_LOGI(FNAME,"drawHorizon P: %1.1f R: %1.1f Y: %1.1f", rad2deg(pitch), rad2deg(roll), rad2deg(yaw) );
	tick++;
	if( !(screens_init & INIT_DISPLAY_HORIZON) ){
		clear();
		P1o.y = 0;
		ucg->setColor( COLOR_WHITE );
		ucg->drawTriangle( 1,   150, 20,  160, 1,   170 ); // Triangles l/r
		ucg->drawTriangle( 240, 150, 220, 160, 240, 170 );
		for( int i=-80; i<=80; i+=20 ){  // 10° scale
			ucg->drawHLine( 1,160+i, 20 );
			ucg->drawHLine( 220,160+i, 20 );
		}
		for( int i=-70; i<=70; i+=20 ){  // 5° scale
			ucg->drawHLine( 10,160+i, 10 );
			ucg->drawHLine( 220,160+i, 10 );
		}
		screens_init |= INIT_DISPLAY_HORIZON;
	}
	Point P1( -100, -60 );
	Point P2(  340, -60 );
	Point P3(  340, 160 );
	Point P4( -100, 160 );
	Point P5( -100, 380 );
	Point P6(  340, 380 );
	Point Center( 120, 160 );
	Point P1r = P1.rotate( Center, -roll );
	Point P2r = P2.rotate( Center, -roll );
	Point P3r = P3.rotate( Center, -roll );
	Point P4r = P4.rotate( Center, -roll );
	Point P5r = P5.rotate( Center, -roll );
	Point P6r = P6.rotate( Center, -roll );
	int p = -rint(rad2deg( pitch )*2);  // 1 deg := 1 pixel
	P1r.moveVertical(p);
	P2r.moveVertical(p);
	P3r.moveVertical(p);
	P4r.moveVertical(p);
	P5r.moveVertical(p);
	P6r.moveVertical(p);
	int heading = 0;

	// ESP_LOGI(FNAME,"P1:%d/%d P2:%d/%d P3:%d/%d P4:%d/%d roll:%f d:%d ", P1r.x, P1r.y+p, P2r.x, P2r.y+p, P3r.x, P3r.y+p, P4r.x , P4r.y+p, rad2deg(roll), p  );
	if( P1r.y != P1o.y || P1r.x != P1o.x ){
		// ESP_LOGI(FNAME,"drawHorizon P: %1.1f R: %1.1f Y: %1.1f", rad2deg(pitch), rad2deg(roll), rad2deg(yaw) );
		ucg->setClipRange( 20, 60, 200, 200 );
		ucg->setColor( COLOR_LBLUE );
		ucg->drawTetragon( P1r.x, P1r.y, P2r.x, P2r.y, P3r.x, P3r.y, P4r.x , P4r.y );
		ucg->setColor( COLOR_BROWN );
		ucg->drawTetragon( P4r.x, P4r.y, P3r.x, P3r.y, P6r.x, P6r.y, P5r.x , P5r.y );
		// Flarm::drawAirplane( 120, 160, true, false );  would be nice hence flickering
		P1o = P1r;
		P2o = P2r;
		P3o = P3r;
		P4o = P4r;
		P5o = P5r;
		P6o = P6r;
		oroll = roll;
		ucg->undoClipRange();
	}
	if( theCompass ){
		heading = fast_iroundf(mag_hdt.get());
		ucg->setFont(ucg_font_fub20_hr, true);
		ucg->setPrintPos(70,310);
		if( heading >= 360 )
			heading -= 360;
		//			ESP_LOGI(FNAME,"compass enable, heading: %d", heading );
		if( heading > 0  && heading != heading_old){
			ucg->setColor( COLOR_WHITE );
			ucg->printf("   %d°   ", heading );
			heading_old = heading;
		}
	}
}


void IpsDisplay::drawLoadDisplay( float loadFactor ){
	// ESP_LOGI(FNAME,"drawLoadDisplay %1.1f tick: %d", loadFactor, tick );
	tick++;

	if( !(screens_init & INIT_DISPLAY_GLOAD) ){
		initLoadDisplay();
		screens_init |= INIT_DISPLAY_GLOAD;
	}
	// draw G pointer
	MAINgauge->drawIndicator( loadFactor );

	// G load digital
	if( !(tick%3) ) {
		MAINgauge->drawFigure(loadFactor);
	}

	// Min/Max values
	if( old_gmax != gload_pos_max.get() || old_gmin != gload_neg_max.get() || !(tick%10) ) {
		if( gload_pos_max.get() < gload_pos_limit.get() && gload_neg_max.get() > gload_neg_limit.get()) {
			ucg->setColor( COLOR_WHITE );
		}
		else {
			ucg->setColor( COLOR_RED );
		}

		ucg->setFont(ucg_font_fub14_hr, true);
		char buf[60];
		sprintf( buf, "  %+1.1f / %+1.1f g", gload_pos_max.get(), gload_neg_max.get() );
		int16_t text_width = ucg->getStrWidth( buf );
		ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MPG_POS+24);
		ucg->print(buf);
		old_gmax = gload_pos_max.get();
		old_gmin = gload_neg_max.get();
	}
	if( old_ias_max != airspeed_max.get() || !(tick%10)){
		if( airspeed_max.get() < v_max.get() ) {
			ucg->setColor( COLOR_WHITE );
		} else {
			ucg->setColor( COLOR_RED );
		}
		
		ucg->setFont(ucg_font_fub14_hr, true);
		char buf[60];
		sprintf( buf, "  %3d %s", Units::SpeedRounded(airspeed_max.get()), Units::SpeedUnitStr() );
		int16_t text_width = ucg->getStrWidth( buf );
		ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MIAS_POS+24);
		ucg->print(buf);
		old_ias_max = airspeed_max.get();
	}

	if( !(tick%10)){
		drawLoadDisplayTexts();
	}
	if ( bottom_dirty ) {
		bottom_dirty = false;
		initLoadDisplay();
	}
}


float getHeading() { // fixme move to compass
	float heading = 0;
	heading = mag_hdt.get();
	if( (heading < 0) && Flarm::gpsStatus() ) {
		// fall back to GPS course
		heading = Flarm::getGndCourse();
	}
	return heading;
}

// fixme arg not needed on stack
void IpsDisplay::drawDisplay(float te_ms, float ate_ms, float polar_sink_ms, float altitude_m,
		float temp, float volt, float s2fd_ms, float s2f_ms, float acl_ms){
	// ESP_LOGI(FNAME,"drawDisplay polar_sink: %f AVario: %f m/s", polar_sink_ms, ate_ms );
	if( !(screens_init & INIT_DISPLAY_RETRO) ){
		initDisplay();
		screens_init |= INIT_DISPLAY_RETRO;
	}
	tick++;
	// ESP_LOGI(FNAME,"drawDisplay  TE=%0.1f IAS:%d km/h  WK=%d", te, airspeed, wksensor  );

	// todo integrate better into screen element
	if ( VCMode.isNetto() ) {
		te_ms -= polar_sink_ms;
		ate_ms -= polar_sink_ms;
	}
	if ( VCMode.getVMode() == CruiseMode::MODE_REL_NETTO ) { // Super Netto, considering circling sink
		te_ms += Speed2Fly.circlingSink( ias.get() );
		ate_ms += Speed2Fly.circlingSink( ias.get() );
	}

	// Unit adaption for mph and knots
	float s2f = Units::Speed( s2f_ms );
	float s2fd = Units::Speed( s2fd_ms );
	// int airspeed = fast_iroundf_positive(Units::Airspeed( airspeed_kmh ));

	// average Climb
	if( !(tick%2) ) {
		MAINgauge->drawFigure(ate_ms);
	}

	// S2F bar
    if ( (((int)s2fd != s2fdalt) || (s2falt != (int)(s2f+0.5)) || !(tick%11)) && S2FBARgauge ) {
		// static float s=0; // check the bar code
		// s2fd = sin(s) * 42.;
		// s+=0.04;
		S2FBARgauge->draw(s2fd);
		S2FBARgauge->drawSpeed(s2f);
	}

	// MC val
	if( MCgauge && !(tick%5) ) {
		MCgauge->draw(MC.get());
	}

	// Bluetooth etc
	if( !(tick%12) )
	{
		drawConnection(DISPLAY_W-25, FLOGO );
	}

	// Upper gauge
	if( vario_upper_gauge.get() && !(tick%3) ) {
		TOPgauge->draw();
	}

	// Altitude
	if( ALTgauge ) {
		ALTgauge->draw(altitude_m);
	}

	// Wind & center aid
	if( !(tick%2) ){
		if ( theCenteraid && ! VCMode.getCMode() ) {
			theCenteraid->drawCenterAid();
		}
		else if ( wind_enable.get() > WA_OFF ) {
			// static int16_t wdir=-1, idir=-1;
			// static int16_t wval=0, ival=0;
			// the wind simulator to check the wind indicator
			// float d = (rand()%180) / M_PI_2;
			// idir += abs(sin(d)) + 2;
			// ival = int((wval+d))%120;

			int16_t wdir=-1, idir=-1;
			int16_t wval=0, ival=0;
			int16_t ageStraight;
			int16_t ageCircling;
			if ( wind_enable.get() & WA_BOTH ) {
				if ( straightWind && ! straightWind->getWind(&wdir, &wval, &ageStraight) ) {
					wdir = -1;
				}

				if ( circleWind && ! circleWind->getWind(&wdir, &wval, &ageCircling) ) {
					wdir = -1;
				}
			}
			else {
				idir = extwind_inst_dir.get();
				ival = extwind_inst_speed.get();
				wdir = extwind_sptc_dir.get();
				wval = extwind_sptc_speed.get();
			}
			WNDgauge->drawWind(wdir, wval, idir, ival);
		}
	}

	// Vario indicator
	MAINgauge->draw(te_ms);
	if( VCMode.isGross() ){
        MAINgauge->drawPolarSink(polar_sink);
    }

	// Battery
	if ( !(tick%15) ) {
		BATgauge->draw(volt);
	}

	// Temperature Value
	temp_status_t mputemp = MPU.getSiliconTempStatus();
	if( (((int)(temp*10) != tempalt) || (mputemp != siliconTempStatusOld)) && !(tick%12)) {
		drawTemperature( 4, 30, temp );
		tempalt=(int)(temp*10);
		siliconTempStatusOld = mputemp;
	}

	// WK-Indicator
	if( FLAPSgauge && !(tick%3) )
	{
		FLAPSgauge->draw(ias.get());
	}

	// Cruise mode or circling
	if( mode_dirty ) {
        VCSTATgauge->draw();
        if (vario_centeraid.get() && !VCMode.getCMode()) {
            WNDgauge->clearGauge();
        } else {
            WNDgauge->drawRose();
        }
        mode_dirty = false;
    }

    // Medium Climb Indicator
    if (!(tick % 4)) {
        // static float s = 0; // check the diamond
        // average_climb.set(sin(s) * 2.);
        // s += 0.1;
        MAINgauge->drawAVG();
    }

    if (bottom_dirty) {
        ESP_LOGI(FNAME, "redraw scale around %f", -_range + 2);
        bottom_dirty = false;
        MAINgauge->drawScaleBottom();
        MAINgauge->forceAllRedraw();
        if (MCgauge)
        {
            MCgauge->forceRedraw();
        }
        BATgauge->forceRedraw();
    }
    // ESP_LOGI(FNAME,"IpsDisplay::drawDisplay  TE=%0.1f  x0:%d y0:%d x2:%d y2:%d", te, x0, y0, x2,y2 );
}


