#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <Eigen/Dense>
#include <math.h>

/**
 * This digital low pass filter is developed via the bilinear method, which transforms a
 * s-domain transfer function into z-domain transfer function
 * cutOffFreq : is the cutoff frequency of the low pass filter
 * dampRatio : the recommanded value is 0.707, which controls the maximum overshoot
 * dTime : is the sampling period
 * nDim : is the size of input signal vector needed to be filtered
 */
class LowPassFilter {
  public:
    LowPassFilter(double cutOffFreq, double dampRatio, double dTime, int nFilter);
    Eigen::VectorXd mFilter(Eigen::VectorXd sigIn);

  private:
    double dT;
    Eigen::VectorXd sigIn_1;
    Eigen::VectorXd sigIn_2;
    Eigen::VectorXd sigOut_1;
    Eigen::VectorXd sigOut_2;
    double b2, b1, b0;
    double a2, a1, a0;
};

#endif