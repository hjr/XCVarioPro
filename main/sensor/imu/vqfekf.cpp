/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "vqfekf.h"

#include <algorithm>


BiasEstimatorEKF::BiasEstimatorEKF() :
    biasP{1e-3f, 1e-3f, 1e-3f} // initial covariance (relativ hoch, da Anfangsunsicherheit) 
{
}


// bias filter update
void BiasEstimatorEKF::update(const vector_f& gyro_meas, bool isResting) {
    constexpr float dt = 0.1f;

    // prediction
    biasP += dt * biasQ;

    if (!isResting) {
        // during motion: No direct measurement, only decrease confidence
        return;
    }

    // update
    vector_f innov = gyro_meas - biasEstimate;

    // Kalman gain
    vector_f K     = biasP / (biasP + restR);

    // state- und covariance update
    // constexpr float maxBiasStep = 1e-4f;
    // vector_f biasStep = K * innov;
    // biasEstimate += biasStep.clamp(-maxBiasStep, maxBiasStep);
    biasEstimate += K * innov;
    biasP *= vector_f(1.f, 1.f, 1.f) - K;
}

void BiasEstimatorEKF::reset() {
    biasEstimate = {};
    biasP = {1e-3f, 1e-3f, 1e-3f};
}