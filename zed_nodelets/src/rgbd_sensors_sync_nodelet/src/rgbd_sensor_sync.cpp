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

#include "rgbd_sensor_sync.hpp"

#ifndef NDEBUG
#include <ros/console.h>
#endif

namespace zed_nodelets {

RgbdSensorsSyncNodelet::RgbdSensorsSyncNodelet() {

}

RgbdSensorsSyncNodelet::~RgbdSensorsSyncNodelet() {
    if(mApproxFullSync)
        delete mApproxFullSync;
    if(mApproxRgbdImuSync)
        delete mApproxRgbdImuSync;
    if(mApproxRgbdSync)
        delete mApproxRgbdSync;

    if(mExactFullSync)
        delete mExactFullSync;
    if(mExactRgbdImuSync)
        delete mExactRgbdImuSync;
    if(mExactRgbdSync)
        delete mExactRgbdSync;
}

void RgbdSensorsSyncNodelet::onInit() {
    // Node handlers
    mNh = getMTNodeHandle();
    mNhP = getMTPrivateNodeHandle();

#ifndef NDEBUG
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
#endif

    NODELET_INFO( "**** Starting nodelet '%s'",getName().c_str() );

    readParameters();

    if( mUseApproxSync ) {
        NODELET_DEBUG( "Using Approximate Time sync");

        if( mUseImu && mUseMag ) {
            mApproxFullSync = new message_filters::Synchronizer<ApproxFullSyncPolicy>(ApproxFullSyncPolicy(mQueueSize),
                                                                                      mSubRgbImage, mSubDepthImage,
                                                                                      mSubRgbCamInfo, mSubDepthCamInfo,
                                                                                      mSubImu,mSubMag);
            mApproxFullSync->registerCallback(boost::bind(&RgbdSensorsSyncNodelet::callbackFull, this,
                                                          _1, _2, _3, _4, _5, _6));

            NODELET_DEBUG("RGB + Depth + IMU + Magnetometer Sync" );

        } else if( mUseImu ) {
            NODELET_DEBUG("RGB + Depth + IMU Sync" );


        } else if( mUseMag ) {
            NODELET_DEBUG("RGB + Depth + Magnetometer Sync" );


        } else {
            NODELET_DEBUG("RGB + Depth Sync" );


        }

    } else {
        NODELET_DEBUG( "Using Exact Time sync");

        if( mUseApproxSync ) {
            NODELET_DEBUG( "Using Approximate Time sync");

            if( mUseImu && mUseMag ) {
                NODELET_DEBUG("RGB + Depth + IMU + Magnetometer Sync" );

            } else if( mUseImu ) {
                NODELET_DEBUG("RGB + Depth + IMU Sync" );


            } else if( mUseMag ) {
                NODELET_DEBUG("RGB + Depth + Magnetometer Sync" );


            } else {
                NODELET_DEBUG("RGB + Depth Sync" );


            }

        }
    }

    // Create remappings
    ros::NodeHandle rgb_nh(mNh, mZedNodeletName+"/rgb");
    ros::NodeHandle depth_nh(mNh, mZedNodeletName+"/depth");
    ros::NodeHandle rgb_pnh(mNhP, mZedNodeletName+"/rgb");
    ros::NodeHandle depth_pnh(mNhP, mZedNodeletName+"/depth");
    ros::NodeHandle imu_nh(mNh, mZedNodeletName+"/imu");
    ros::NodeHandle imu_pnh(mNhP, mZedNodeletName+"/imu");

    image_transport::ImageTransport rgb_it(rgb_nh);
    image_transport::ImageTransport depth_it(depth_nh);

    image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
    image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

    mSubRgbImage.subscribe(rgb_it, rgb_nh.resolveName("image_rect_color"), 1, hintsRgb);
    mSubDepthImage.subscribe(depth_it, depth_nh.resolveName("depth_registered"), 1, hintsDepth);
    mSubRgbCamInfo.subscribe(rgb_nh, "camera_info", 1);
    mSubDepthCamInfo.subscribe(depth_nh, "camera_info", 1);

    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubRgbImage.getTopic().c_str());
    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubRgbCamInfo.getTopic().c_str());
    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubDepthImage.getTopic().c_str());
    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubDepthCamInfo.getTopic().c_str());

    if( mUseImu ) {
        mSubImu.subscribe(imu_nh,"data",1);
        NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubImu.getTopic().c_str());
    }

    if(mUseMag) {
        mSubMag.subscribe(imu_nh,"mag",1);
        NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubMag.getTopic().c_str());
    }
}

void RgbdSensorsSyncNodelet::readParameters() {
    NODELET_INFO("*** PARAMETERS [%s]***",  getName().c_str());

    mNhP.getParam("zed_nodelet_name", mZedNodeletName);
    mNhP.getParam("approx_sync", mUseApproxSync);
    mNhP.getParam("queue_size", mQueueSize);
    mNhP.getParam("sub_imu", mUseImu);
    mNhP.getParam("sub_mag", mUseMag);

    NODELET_INFO(" * zed_nodelet_name -> %s", mZedNodeletName.c_str());
    NODELET_INFO(" * approx_sync -> %s", mUseApproxSync?"true":"false");
    NODELET_INFO(" * queue_size  -> %d", mQueueSize);
    NODELET_INFO(" * sub_imu -> %s", mUseImu?"true":"false");
    NODELET_INFO(" * sub_mag -> %s", mUseMag?"true":"false");
}

void RgbdSensorsSyncNodelet::callbackFull(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth,
                                          const sensor_msgs::CameraInfoConstPtr &rgbCameraInfo,
                                          const sensor_msgs::CameraInfoConstPtr &depthCameraInfo,
                                          const sensor_msgs::ImuConstPtr &imu,
                                          const sensor_msgs::MagneticFieldConstPtr &mag) {
    NODELET_DEBUG( "Callback Full - topics timestamp: %lu, %lu, %lu, %lu, %lu, %lu",
                   rgb->header.stamp.toNSec(),depth->header.stamp.toNSec(),
                   rgbCameraInfo->header.stamp.toNSec(),depthCameraInfo->header.stamp.toNSec(),
                   imu->header.stamp.toNSec(), mag->header.stamp.toNSec());
}

}
