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

#include <string>
#include <sl/Camera.hpp>
#include <ros/time.h>

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

    inline sl::uchar4 depackColor4(float colorIn) {
        sl::uchar4 out;
        uint32_t color_uint = *(uint32_t*) & colorIn;
        unsigned char* color_uchar = (unsigned char*) &color_uint;
        for (int c = 0; c < 3; c++) {
            out[c] = static_cast<unsigned char>(color_uchar[c]);
        }
        out.w = 255;
        return out;
    }

    
} // namespace

#endif // SL_TOOLS_H
