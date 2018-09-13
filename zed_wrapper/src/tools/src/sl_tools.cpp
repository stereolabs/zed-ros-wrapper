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

#include <sys/stat.h>
#include <vector>
#include <sstream>

namespace sl_tools {

    int checkCameraReady(unsigned int serial_number) {
        int id = -1;
        auto f = sl::Camera::getDeviceList();
        for (auto& it : f)
            if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE) {
                id = it.id;
            }
        return id;
    }

    sl::DeviceProperties getZEDFromSN(unsigned int serial_number) {
        sl::DeviceProperties prop;
        auto f = sl::Camera::getDeviceList();
        for (auto& it : f) {
            if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE) {
                prop = it;
            }
        }
        return prop;
    }

    cv::Mat convertRodrigues(sl::float3 r) {
        double theta = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

        if (theta < DBL_EPSILON) {
            return R;
        } else {
            double c = cos(theta);
            double s = sin(theta);
            double c1 = 1. - c;
            double itheta = theta ? 1. / theta : 0.;

            r *= itheta;

            cv::Mat rrt = cv::Mat::eye(3, 3, CV_32F);
            float* p = (float*) rrt.data;
            p[0] = r.x * r.x;
            p[1] = r.x * r.y;
            p[2] = r.x * r.z;
            p[3] = r.x * r.y;
            p[4] = r.y * r.y;
            p[5] = r.y * r.z;
            p[6] = r.x * r.z;
            p[7] = r.y * r.z;
            p[8] = r.z * r.z;

            cv::Mat r_x = cv::Mat::eye(3, 3, CV_32F);
            p = (float*) r_x.data;
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
            R = c * cv::Mat::eye(3, 3, CV_32F) + c1 * rrt + s * r_x;
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
        uint32_t sec  = static_cast<uint32_t>(t / 1000000000);
        uint32_t nsec = static_cast<uint32_t>(t % 1000000000);
        return ros::Time(sec, nsec);
    }

} // namespace
