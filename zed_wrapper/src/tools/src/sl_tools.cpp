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

#include "sl_tools.h"

#include <sstream>
#include <sys/stat.h>
#include <vector>

#include <sensor_msgs/image_encodings.h>

#include <boost/make_shared.hpp>

namespace sl_tools {

    int checkCameraReady(unsigned int serial_number) {
        int id = -1;
        auto f = sl::Camera::getDeviceList();

        for (auto& it : f)
            if (it.serial_number == serial_number &&
                it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE) {
                id = it.id;
            }

        return id;
    }

    sl::DeviceProperties getZEDFromSN(unsigned int serial_number) {
        sl::DeviceProperties prop;
        auto f = sl::Camera::getDeviceList();

        for (auto& it : f) {
            if (it.serial_number == serial_number &&
                it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE) {
                prop = it;
            }
        }

        return prop;
    }

    std::vector<float>  convertRodrigues(sl::float3 r) {
        float theta = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);

        std::vector<float> R = {1.0f, 0.0f, 0.0f,
                                0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 1.0f
                               };

        if (theta < DBL_EPSILON) {
            return R;
        } else {
            float c = cos(theta);
            float s = sin(theta);
            float c1 = 1.f - c;
            float itheta = theta ? 1.f / theta : 0.f;

            r *= itheta;

            std::vector<float> rrt = {1.0f, 0.0f, 0.0f,
                                      0.0f, 1.0f, 0.0f,
                                      0.0f, 0.0f, 1.0f
                                     };

            float* p = rrt.data();
            p[0] = r.x * r.x;
            p[1] = r.x * r.y;
            p[2] = r.x * r.z;
            p[3] = r.x * r.y;
            p[4] = r.y * r.y;
            p[5] = r.y * r.z;
            p[6] = r.x * r.z;
            p[7] = r.y * r.z;
            p[8] = r.z * r.z;

            std::vector<float> r_x = {1.0f, 0.0f, 0.0f,
                                      0.0f, 1.0f, 0.0f,
                                      0.0f, 0.0f, 1.0f
                                     };
            p = r_x.data();
            p[0] = 0;
            p[1] = -r.z;
            p[2] = r.y;
            p[3] = r.z;
            p[4] = 0;
            p[5] = -r.x;
            p[6] = -r.y;
            p[7] = r.x;
            p[8] = 0;

            // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]

            sl::Matrix3f eye;
            eye.setIdentity();

            sl::Matrix3f sl_R(R.data());
            sl::Matrix3f sl_rrt(rrt.data());
            sl::Matrix3f sl_r_x(r_x.data());

            sl_R = eye * c + sl_rrt * c1 + sl_r_x * s;

            R[0] = sl_R.r00;
            R[1] = sl_R.r01;
            R[2] = sl_R.r02;
            R[3] = sl_R.r10;
            R[4] = sl_R.r11;
            R[5] = sl_R.r12;
            R[6] = sl_R.r20;
            R[7] = sl_R.r21;
            R[8] = sl_R.r22;
        }

        return R;
    }


    bool file_exist(const std::string& name) {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    }

    std::string getSDKVersion(int& major, int& minor, int& sub_minor) {
        std::string ver = sl::Camera::getSDKVersion().c_str();
        std::vector<std::string> strings;
        std::istringstream f(ver);
        std::string s;

        while (getline(f, s, '.')) {
            strings.push_back(s);
        }

        major = 0;
        minor = 0;
        sub_minor = 0;

        switch (strings.size()) {
        case 3:
            sub_minor = std::stoi(strings[2]);

        case 2:
            minor = std::stoi(strings[1]);

        case 1:
            major = std::stoi(strings[0]);
        }

        return ver;
    }

    ros::Time slTime2Ros(sl::timeStamp t) {
        uint32_t sec = static_cast<uint32_t>(t / 1000000000);
        uint32_t nsec = static_cast<uint32_t>(t % 1000000000);
        return ros::Time(sec, nsec);
    }

    sensor_msgs::ImagePtr imageToROSmsg(sl::Mat img, std::string frameId, ros::Time t) {

        sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::Image& imgMessage = *ptr;

        imgMessage.header.stamp = t;
        imgMessage.header.frame_id = frameId;
        imgMessage.height = img.getHeight();
        imgMessage.width = img.getWidth();

        int num = 1; // for endianness detection
        imgMessage.is_bigendian = !(*(char*)&num == 1);

        imgMessage.step = img.getStepBytes();

        size_t size = imgMessage.step * imgMessage.height;
        imgMessage.data.resize(size);

        sl::MAT_TYPE dataType = img.getDataType();

        switch (dataType) {
        case sl::MAT_TYPE_32F_C1: /**< float 1 channel.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::float1>(), size);
            break;

        case sl::MAT_TYPE_32F_C2: /**< float 2 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::float2>(), size);
            break;

        case sl::MAT_TYPE_32F_C3: /**< float 3 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::float3>(), size);
            break;

        case sl::MAT_TYPE_32F_C4: /**< float 4 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::float4>(), size);
            break;

        case sl::MAT_TYPE_8U_C1: /**< unsigned char 1 channel.*/
            imgMessage.encoding = sensor_msgs::image_encodings::MONO8;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::uchar1>(), size);
            break;

        case sl::MAT_TYPE_8U_C2: /**< unsigned char 2 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::uchar2>(), size);
            break;

        case sl::MAT_TYPE_8U_C3: /**< unsigned char 3 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::BGR8;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::uchar3>(), size);
            break;

        case sl::MAT_TYPE_8U_C4: /**< unsigned char 4 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::BGRA8;
            memcpy((char*)(&imgMessage.data[0]), img.getPtr<sl::uchar4>(), size);
            break;
        }

        return ptr;
    }

    sensor_msgs::ImagePtr imagesToROSmsg(sl::Mat left, sl::Mat right, std::string frameId, ros::Time t) {

        if (left.getWidth() != right.getWidth() ||
            left.getHeight() != right.getHeight() ||
            left.getChannels() != right.getChannels() ||
            left.getDataType() != right.getDataType()) {
            return nullptr;
        }

        sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::Image& imgMessage = *ptr;

        imgMessage.header.stamp = t;
        imgMessage.header.frame_id = frameId;
        imgMessage.height = left.getHeight();
        imgMessage.width = 2 * left.getWidth();

        int num = 1; // for endianness detection
        imgMessage.is_bigendian = !(*(char*)&num == 1);

        imgMessage.step = 2 * left.getStepBytes();

        size_t size = imgMessage.step * imgMessage.height;
        imgMessage.data.resize(size);

        sl::MAT_TYPE dataType = left.getDataType();

        int dataSize = 0;
        char* srcL;
        char* srcR;

        switch (dataType) {
        case sl::MAT_TYPE_32F_C1: /**< float 1 channel.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dataSize = sizeof(float);
            srcL = (char*)left.getPtr<sl::float1>();
            srcR = (char*)right.getPtr<sl::float1>();
            break;

        case sl::MAT_TYPE_32F_C2: /**< float 2 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
            dataSize = 2 * sizeof(float);
            srcL = (char*)left.getPtr<sl::float2>();
            srcR = (char*)right.getPtr<sl::float2>();
            break;

        case sl::MAT_TYPE_32F_C3: /**< float 3 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
            dataSize = 3 * sizeof(float);
            srcL = (char*)left.getPtr<sl::float3>();
            srcR = (char*)right.getPtr<sl::float3>();
            break;

        case sl::MAT_TYPE_32F_C4: /**< float 4 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
            dataSize = 4 * sizeof(float);
            srcL = (char*)left.getPtr<sl::float4>();
            srcR = (char*)right.getPtr<sl::float4>();
            break;

        case sl::MAT_TYPE_8U_C1: /**< unsigned char 1 channel.*/
            imgMessage.encoding = sensor_msgs::image_encodings::MONO8;
            dataSize = sizeof(char);
            srcL = (char*)left.getPtr<sl::uchar1>();
            srcR = (char*)right.getPtr<sl::uchar1>();
            break;

        case sl::MAT_TYPE_8U_C2: /**< unsigned char 2 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
            dataSize = 2 * sizeof(char);
            srcL = (char*)left.getPtr<sl::uchar2>();
            srcR = (char*)right.getPtr<sl::uchar2>();
            break;

        case sl::MAT_TYPE_8U_C3: /**< unsigned char 3 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::BGR8;
            dataSize = 3 * sizeof(char);
            srcL = (char*)left.getPtr<sl::uchar3>();
            srcR = (char*)right.getPtr<sl::uchar3>();
            break;

        case sl::MAT_TYPE_8U_C4: /**< unsigned char 4 channels.*/
            imgMessage.encoding = sensor_msgs::image_encodings::BGRA8;
            dataSize = 4 * sizeof(char);
            srcL = (char*)left.getPtr<sl::uchar4>();
            srcR = (char*)right.getPtr<sl::uchar4>();
            break;
        }

        char* dest = (char*)(&imgMessage.data[0]);

        for (int i = 0; i < left.getHeight(); i++) {
            memcpy(dest, srcL, left.getStepBytes());
            dest += left.getStepBytes();
            memcpy(dest, srcR, right.getStepBytes());
            dest += right.getStepBytes();

            srcL += left.getStepBytes();
            srcR += right.getStepBytes();
        }

        return ptr;
    }

    std::vector<std::string> split_string(const std::string& s, char seperator) {
        std::vector<std::string> output;
        std::string::size_type prev_pos = 0, pos = 0;

        while ((pos = s.find(seperator, pos)) != std::string::npos) {
            std::string substring(s.substr(prev_pos, pos - prev_pos));
            output.push_back(substring);
            prev_pos = ++pos;
        }

        output.push_back(s.substr(prev_pos, pos - prev_pos));
        return output;
    }

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

} // namespace
