#include "AudioRateParam.hpp"
#include <math.h>


void AudioRateParam::Init(int index, float min, float max, Curve curve)
{
    pmin_   = min;
    pmax_   = max;
    pcurve_ = curve;
    index_     = index;
    lmin_   = logf(min < 0.0000001f ? 0.0000001f : min);
    lmax_   = logf(max);
}

float AudioRateParam::Process(float input)
{
    switch(pcurve_)
    {
        case LINEAR: val_ = (input * (pmax_ - pmin_)) + pmin_; break;
        case EXPONENTIAL:
            val_ = input;
            val_ = ((val_ * val_) * (pmax_ - pmin_)) + pmin_;
            break;
        case LOGARITHMIC:
            val_ = expf((input * (lmax_ - lmin_)) + lmin_);
            break;
        case CUBE:
            val_ = input;
            val_ = ((val_ * (val_ * val_)) * (pmax_ - pmin_)) + pmin_;
            break;
        default: break;
    }
    return val_;
}