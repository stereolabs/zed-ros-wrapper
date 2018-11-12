#ifndef CSMARTWINMEAN_H
#define CSMARTWINMEAN_H

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
