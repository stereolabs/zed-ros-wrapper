#include "csmartmean.hpp"

#include <math.h>

namespace sl_tools {
    CSmartMean::CSmartMean(int winSize) {
        mValCount = 0;

        mMeanCorr = 0.0;
        mMean = 0.0;
        mWinSize = winSize;

        mGamma = (static_cast<double>(mWinSize) - 1.) / static_cast<double>(mWinSize);
    }

    double CSmartMean::addValue(double val) {
        mValCount++;

        mMeanCorr = mGamma * mMeanCorr + (1. - mGamma) * val;
        mMean = mMeanCorr / (1. - pow(mGamma, mValCount));

        return mMean;
    }
}
