// /////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// /////////////////////////////////////////////////////////////////////////

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
