/*
 * S2F.h
 *
 *  Created on: Dec 26, 2018
 *      Author: iltis
 */

#ifndef MAIN_S2F_H_
#define MAIN_S2F_H_

#include "Units.h"

class S2F {
public:
	S2F( );
	virtual ~S2F();
	void begin();
	void calculateOverweight();
	void modifyPolar();
	bool IsValid() const;
	void recalculatePolar();
	void change_ballast();
	void change_mc();
	void setPolar();
	static bool isPolarEqualTo(int idx);
	float speed( float st, bool circling=false );
	float sink( float v );
	inline float minsink_speed() { return _min_speed; };
	void recalcSinkNSpeeds();
	static float getBallastPercent();
	inline float circlingSink(float v) {
		if( v > polar_stall_speed.get()*0.6 )
			return _circling_sink;
		else
			return 0;
	};
	float cw( float v );
	void test( void );
	float getN();
	float getVn( float v );

private:
	float myballast;
	static float bal_percent;
	float a0,a1,a2;
	float w0,w1,w2;
	float _min_speed;
	float _min_sink;
	float _circling_speed;
	float _circling_sink;
	float _stall_speed_ms;
};

#endif /* MAIN_S2F_H_ */
