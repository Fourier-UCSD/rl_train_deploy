#include "../include/LowPassFilter.h"
// #include "LowPassFilter.h"

/**
 * This digital low pass filter is developed via the bilinear method, which transforms a
 * s-domain transfer function into z-domain transfer function
 * cutOffFreq : is the cutoff frequency of the low pass filter
 * dampRatio : the recommanded value is 0.707, which controls the maximum overshoot
 * dTime : is the sampling period
 */
LowPassFilter ::LowPassFilter(double cutOffFreq, double dampRatio,
                              double dTime, int nFilter) {
    dT = dTime;
    sigIn_1 = Eigen::VectorXd::Zero(nFilter);
    sigIn_2 = Eigen::VectorXd::Zero(nFilter);
    sigOut_1 = Eigen::VectorXd::Zero(nFilter);
    sigOut_2 = Eigen::VectorXd::Zero(nFilter);

    double freqInRad = 2. * M_PI * cutOffFreq;
    double c = 2.0 / dT;
    double sqrC = c * c;
    double sqrW = freqInRad * freqInRad;

    b2 = sqrC + 2.0 * dampRatio * freqInRad * c + sqrW;
    b1 = -2.0 * (sqrC - sqrW);
    b0 = sqrC - 2.0 * dampRatio * freqInRad * c + sqrW;

    a2 = sqrW;
    a1 = 2.0 * sqrW;
    a0 = sqrW;
    a2 /= b2;
    a1 /= b2;
    a0 /= b2;

    b1 /= b2;
    b0 /= b2;
    b2 = 1.0;
}

Eigen::VectorXd LowPassFilter::mFilter(Eigen::VectorXd sigIn) {

    Eigen::VectorXd sigOut = a2 * sigIn + a1 * sigIn_1 +
                             a0 * sigIn_2 - b1 * sigOut_1 - b0 * sigOut_2;
    sigIn_2 = sigIn_1;
    sigIn_1 = sigIn;
    sigOut_2 = sigOut_1;
    sigOut_1 = sigOut;
    return sigOut;
}
