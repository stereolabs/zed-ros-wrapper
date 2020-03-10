///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
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

#ifndef RGBD_SENSOR_SYNC_HPP
#define RGBD_SENSOR_SYNC_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "zed_interfaces/RGBDSensors.h"

namespace zed_nodelets {

class RgbdSensorsSyncNodelet : public nodelet::Nodelet {

public:
    RgbdSensorsSyncNodelet();
    virtual ~RgbdSensorsSyncNodelet();

protected:
    /* \brief Initialization function called by the Nodelet base class
         */
    virtual void onInit();

    /* \brief Reads parameters from the param server
         */
    void readParameters();

private:
    // Node handlers
    ros::NodeHandle mNh;    // Node handler
    ros::NodeHandle mNhNs;  // Private Node handler

    // Output message
    zed_interfaces::RGBDSensorsPtr mOutSyncMsg; // Output message

    // Input messages
    sensor_msgs::ImagePtr mInRgbImgMsg;         // Input RGB image message
    sensor_msgs::ImagePtr mInDepthImgMsg;       // Input Depth image message
    sensor_msgs::ImagePtr mInRgbCamInfoMsg;     // Input RGB Camera Info message
    sensor_msgs::ImagePtr mInDepthCamInfoMsg;   // Input Depth Camera Info message
    sensor_msgs::ImuPtr mInImuMsg;              // Input IMU message
    sensor_msgs::ImuPtr mInMagMsg;              // Input Magnetometer message

    // Subscribers
    message_filters::Subscriber<sensor_msgs::Image> mSubRgbImage;
    message_filters::Subscriber<sensor_msgs::Image> mSubDepthImage;
    message_filters::Subscriber<sensor_msgs::CameraInfo> mSubRgbCamInfo;
    message_filters::Subscriber<sensor_msgs::CameraInfo> mSubDepthCamInfo;
    message_filters::Subscriber<sensor_msgs::Imu> mSubImu;
    message_filters::Subscriber<sensor_msgs::MagneticField> mSubMag;

    // Approx sync policies
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::Imu, sensor_msgs::MagneticField> ApproxFullSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::Imu> ApproxRgbdImuSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproxRgbdSyncPolicy;
    message_filters::Synchronizer<ApproxFullSyncPolicy>* mApproxFullSync = nullptr;
    message_filters::Synchronizer<ApproxRgbdImuSyncPolicy>* mApproxRgbdImuSync = nullptr;
    message_filters::Synchronizer<ApproxRgbdSyncPolicy>* mApproxRgbdSync = nullptr;

    // Exact sync policies
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::Imu, sensor_msgs::MagneticField> ExactFullSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::Imu> ExactRgbdImuSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactRgbdSyncPolicy;
    message_filters::Synchronizer<ExactFullSyncPolicy>* mExactFullSync = nullptr;
    message_filters::Synchronizer<ExactRgbdImuSyncPolicy>* mExactRgbdImuSync = nullptr;
    message_filters::Synchronizer<ExactRgbdSyncPolicy>* mExactRgbdSync = nullptr;

    // Params
    bool mUseApproxSync = true;
    bool mUseImu = true;
    bool mUseMag = true;
    int mQueueSize = 10;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_nodelets::RgbdSensorsSyncNodelet, nodelet::Nodelet)

#endif // RGBD_SENSOR_SYNC_HPP
