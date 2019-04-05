#ifndef SL_TOOLS_H
#define SL_TOOLS_H

///////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////

#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sl/Camera.hpp>
#include <string>
#include <vector>

namespace sl_tools {

    /* \brief Check if a ZED camera is ready
    * \param serial_number : the serial number of the camera to be checked
    */
    int checkCameraReady(unsigned int serial_number);

    /* \brief Get ZED camera properties
    * \param serial_number : the serial number of the camera
    */
    sl::DeviceProperties getZEDFromSN(unsigned int serial_number);

    std::vector<float> convertRodrigues(sl::float3 r);

    /* \brief Test if a file exist
    * \param name : the path to the file
    */
    bool file_exist(const std::string& name);

    /* \brief Get Stereolabs SDK version
     * \param major : major value for version
     * \param minor : minor value for version
     * \param sub_minor _ sub_minor value for version
     */
    std::string getSDKVersion(int& major, int& minor, int& sub_minor);

    /* \brief Convert StereoLabs timestamp to ROS timestamp
     *  \param t : Stereolabs timestamp to be converted
     */
    ros::Time slTime2Ros(sl::timeStamp t);

    /* \brief sl::Mat to ros message conversion
     * \param img : the image to publish
     * \param frameId : the id of the reference frame of the image
     * \param t : the ros::Time to stamp the image
     */
    sensor_msgs::ImagePtr imageToROSmsg(sl::Mat img, std::string frameId, ros::Time t);

    /* \brief Two sl::Mat to ros message conversion
     * \param left : the left image to publish
     * \param right : the right image to publish
     * \param frameId : the id of the reference frame of the image
     * \param t : the ros::Time to stamp the image
     */
    sensor_msgs::ImagePtr imagesToROSmsg(sl::Mat left, sl::Mat right, std::string frameId, ros::Time t);

    /* \brief String tokenization
     */
    std::vector<std::string> split_string(const std::string& s, char seperator);

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


} // namespace sl_tools

#endif  // SL_TOOLS_H
