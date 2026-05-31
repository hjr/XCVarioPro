
/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "math/vector_3d.h"

class BiasEstimatorEKF {
public:
    BiasEstimatorEKF();
    void update(const vector_f& gyro_meas, bool isResting);
    inline const vector_f& getBias() const { return biasEstimate; }
    void reset();

private:
    // Separate Bias-Kalman-Filter (only for gyro bias, 3 states)
    vector_f biasEstimate = {};      // Current estimated gyro bias
    vector_f biasP = {};             // Covariance diagonal (simplified, independent per axis)
    static constexpr float biasQ = 1e-6f;  // process noise (very small)
    static constexpr float restR = 5e-2f;  // sensor noise at rest (very small → strong confidence)
    static constexpr float motionR = 1.0f; // sensor noise during motion (large → weak confidence)
};
