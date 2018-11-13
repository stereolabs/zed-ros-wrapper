#ifndef CSMARTWINMEAN_H
#define CSMARTWINMEAN_H

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

namespace sl_tools {
    /*!
     * \brief The CSmartMean class is used to
     * make a mobile window mean of a sequence of values
     * and reject outliers.
     * Tutorial:
     * https://www.myzhar.com/blog/tutorials/tutorial-exponential-weighted-average-good-moving-windows-average/
     */
    class CSmartMean {
      public:
        CSmartMean(int winSize);

        int getValCount() {
            return mValCount;   ///< Return the number of values in the sequence
        }

        double getMean() {
            return mMean;   ///< Return the updated mean
        }

        /*!
         * \brief addValue
         * Add a value to the sequence
         * \param val value to be added
         * \return mean value
         */
        double addValue(double val);

      private:
        int mWinSize; ///< The size of the window (number of values ti evaluate)
        int mValCount; ///< The number of values in sequence

        double mMeanCorr; ///< Used for bias correction
        double mMean;     ///< The mean of the last \ref mWinSize values

        double mGamma; ///< Weight value
    };
}

#endif // CSMARTWINMEAN_H
