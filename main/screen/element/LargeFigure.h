/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"

// a related prominent figure to the gauge visual indicator
class LargeFigure : public ScreenElement
{
public:
    LargeFigure(int16_t x, int16_t y);
    void setPos(int16_t x, int16_t y) { _ref_x = x; _ref_y = y+_yoff; }
    void draw(float a);
private:
    int16_t _value = 0;
    int16_t _yoff;
};
