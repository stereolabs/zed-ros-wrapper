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

#include <chrono>

namespace zed_nodelets
{
RgbdSensorsSyncNodelet::RgbdSensorsSyncNodelet()
{
}

RgbdSensorsSyncNodelet::~RgbdSensorsSyncNodelet()
{
  if (mApproxFullSync)
    delete mApproxFullSync;
  if (mApproxRgbdImuSync)
    delete mApproxRgbdImuSync;
  if (mApproxRgbdMagSync)
    delete mApproxRgbdMagSync;
  if (mApproxRgbdSync)
    delete mApproxRgbdSync;

  if (mExactFullSync)
    delete mExactFullSync;
  if (mExactRgbdImuSync)
    delete mExactRgbdImuSync;
  if (mExactRgbdMagSync)
    delete mExactRgbdMagSync;
  if (mExactRgbdSync)
    delete mExactRgbdSync;
}

void RgbdSensorsSyncNodelet::onInit()
{
  // Node handlers
  mNh = getNodeHandle();
  mNhP = getPrivateNodeHandle();

#ifndef NDEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  NODELET_INFO("********** Starting nodelet '%s' **********", getName().c_str());

  readParameters();

  mPubRaw = mNhP.advertise<zed_interfaces::RGBDSensors>("rgbd_sens", 1);
  NODELET_INFO_STREAM("Advertised on topic " << mPubRaw.getTopic());

  if (mUseApproxSync)
  {
    NODELET_DEBUG("Using Approximate Time sync");

    if (mUseImu && mUseMag)
    {
      mApproxFullSync = new message_filters::Synchronizer<ApproxFullSyncPolicy>(
          ApproxFullSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo, mSubImu,
          mSubMag);
      mApproxFullSync->registerCallback(
          boost::bind(&RgbdSensorsSyncNodelet::callbackFull, this, _1, _2, _3, _4, _5, _6));

      NODELET_DEBUG("RGB + Depth + IMU + Magnetometer Sync");
    }
    else if (mUseImu)
    {
      NODELET_DEBUG("RGB + Depth + IMU Sync");

      mApproxRgbdImuSync = new message_filters::Synchronizer<ApproxRgbdImuSyncPolicy>(
          ApproxRgbdImuSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo, mSubImu);
      mApproxRgbdImuSync->registerCallback(
          boost::bind(&RgbdSensorsSyncNodelet::callbackRGBDIMU, this, _1, _2, _3, _4, _5));
    }
    else if (mUseMag)
    {
      NODELET_DEBUG("RGB + Depth + Magnetometer Sync");

      mApproxRgbdMagSync = new message_filters::Synchronizer<ApproxRgbdMagSyncPolicy>(
          ApproxRgbdMagSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo, mSubMag);
      mApproxRgbdMagSync->registerCallback(
          boost::bind(&RgbdSensorsSyncNodelet::callbackRGBDMag, this, _1, _2, _3, _4, _5));
    }
    else
    {
      NODELET_DEBUG("RGB + Depth Sync");

      mApproxRgbdSync = new message_filters::Synchronizer<ApproxRgbdSyncPolicy>(
          ApproxRgbdSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo);
      mApproxRgbdSync->registerCallback(boost::bind(&RgbdSensorsSyncNodelet::callbackRGBD, this, _1, _2, _3, _4));
    }
  }
  else
  {
    NODELET_DEBUG("Using Exact Time sync");

    if (mUseImu && mUseMag)
    {
      mExactFullSync = new message_filters::Synchronizer<ExactFullSyncPolicy>(
          ExactFullSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo, mSubImu,
          mSubMag);
      mExactFullSync->registerCallback(
          boost::bind(&RgbdSensorsSyncNodelet::callbackFull, this, _1, _2, _3, _4, _5, _6));

      NODELET_DEBUG("RGB + Depth + IMU + Magnetometer Sync");
    }
    else if (mUseImu)
    {
      NODELET_DEBUG("RGB + Depth + IMU Sync");

      mExactRgbdImuSync = new message_filters::Synchronizer<ExactRgbdImuSyncPolicy>(
          ExactRgbdImuSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo, mSubImu);
      mExactRgbdImuSync->registerCallback(
          boost::bind(&RgbdSensorsSyncNodelet::callbackRGBDIMU, this, _1, _2, _3, _4, _5));
    }
    else if (mUseMag)
    {
      NODELET_DEBUG("RGB + Depth + Magnetometer Sync");

      mExactRgbdMagSync = new message_filters::Synchronizer<ExactRgbdMagSyncPolicy>(
          ExactRgbdMagSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo, mSubMag);
      mExactRgbdMagSync->registerCallback(
          boost::bind(&RgbdSensorsSyncNodelet::callbackRGBDMag, this, _1, _2, _3, _4, _5));
    }
    else
    {
      NODELET_DEBUG("RGB + Depth Sync");

      mExactRgbdSync = new message_filters::Synchronizer<ExactRgbdSyncPolicy>(
          ExactRgbdSyncPolicy(mQueueSize), mSubRgbImage, mSubDepthImage, mSubRgbCamInfo, mSubDepthCamInfo);
      mExactRgbdSync->registerCallback(boost::bind(&RgbdSensorsSyncNodelet::callbackRGBD, this, _1, _2, _3, _4));
    }
  }

  // Create remappings
  ros::NodeHandle rgb_nh(mNh, mZedNodeletName + "/rgb");
  ros::NodeHandle depth_nh(mNh, mZedNodeletName + "/depth");
  ros::NodeHandle rgb_pnh(mNhP, mZedNodeletName + "/rgb");
  ros::NodeHandle depth_pnh(mNhP, mZedNodeletName + "/depth");
  ros::NodeHandle imu_nh(mNh, mZedNodeletName + "/imu");

  image_transport::ImageTransport rgb_it(rgb_nh);
  image_transport::ImageTransport depth_it(depth_nh);

  image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
  image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

  mSubRgbImage.subscribe(rgb_it, rgb_nh.resolveName("image_rect_color"), 1, hintsRgb);
  mSubDepthImage.subscribe(depth_it, depth_nh.resolveName("depth_registered"), 1, hintsDepth);
  mSubRgbCamInfo.subscribe(rgb_nh, "camera_info", 1);
  mSubDepthCamInfo.subscribe(depth_nh, "camera_info", 1);

  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubRgbImage.getTopic().c_str());
  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubRgbCamInfo.getTopic().c_str());
  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubDepthImage.getTopic().c_str());
  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubDepthCamInfo.getTopic().c_str());

  if (mUseImu)
  {
    mSubImu.subscribe(imu_nh, "data", 1);
    NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubImu.getTopic().c_str());
  }

  if (mUseMag)
  {
    mSubMag.subscribe(imu_nh, "mag", 1);
    NODELET_INFO_STREAM(" * Subscribed to topic: " << mSubMag.getTopic().c_str());
  }
}

void RgbdSensorsSyncNodelet::readParameters()
{
  NODELET_INFO("*** PARAMETERS [%s]***", getName().c_str());

  mNhP.getParam("zed_nodelet_name", mZedNodeletName);
  mNhP.getParam("approx_sync", mUseApproxSync);
  mNhP.getParam("queue_size", mQueueSize);
  mNhP.getParam("sub_imu", mUseImu);
  mNhP.getParam("sub_mag", mUseMag);

  NODELET_INFO(" * zed_nodelet_name -> %s", mZedNodeletName.c_str());
  NODELET_INFO(" * approx_sync -> %s", mUseApproxSync ? "true" : "false");
  NODELET_INFO(" * queue_size  -> %d", mQueueSize);
  NODELET_INFO(" * sub_imu -> %s", mUseImu ? "true" : "false");
  NODELET_INFO(" * sub_mag -> %s", mUseMag ? "true" : "false");
}

void RgbdSensorsSyncNodelet::callbackRGBD(const sensor_msgs::ImageConstPtr &rgb,
                                          const sensor_msgs::ImageConstPtr &depth,
                                          const sensor_msgs::CameraInfoConstPtr &rgbCameraInfo,
                                          const sensor_msgs::CameraInfoConstPtr &depthCameraInfo)
{
  // ----> Frequency calculation
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
  last_time = now;

  double freq = 1e6 / elapsed_usec;
  NODELET_DEBUG("Freq: %.2f", freq);
  // <---- Frequency calculation

  double rgbStamp = rgb->header.stamp.toSec();
  double depthStamp = depth->header.stamp.toSec();
  double rgbInfoStamp = rgbCameraInfo->header.stamp.toSec();
  double depthInfoStamp = depthCameraInfo->header.stamp.toSec();

  uint32_t subraw = mPubRaw.getNumSubscribers();

  if (subraw == 0)
  {
    return;
  }

  zed_interfaces::RGBDSensorsPtr outSyncMsg = boost::make_shared<zed_interfaces::RGBDSensors>();

  outSyncMsg->header.frame_id = rgb->header.frame_id;
  outSyncMsg->header.stamp = rgb->header.stamp;
  outSyncMsg->rgbCameraInfo = *rgbCameraInfo;
  outSyncMsg->depthCameraInfo = *depthCameraInfo;

  outSyncMsg->rgb = *rgb;
  outSyncMsg->depth = *depth;

  mPubRaw.publish(outSyncMsg);

  if (rgbStamp != rgb->header.stamp.toSec())
  {
    NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
                  "sure the node publishing the topics doesn't override the same data after publishing them. A "
                  "solution is to use this node within another nodelet manager. ");
    NODELET_ERROR("Stamps: "
                  "rgb=%f->%f",
                  rgbStamp, rgb->header.stamp.toSec());
  }
}

void RgbdSensorsSyncNodelet::callbackRGBDIMU(const sensor_msgs::ImageConstPtr &rgb,
                                             const sensor_msgs::ImageConstPtr &depth,
                                             const sensor_msgs::CameraInfoConstPtr &rgbCameraInfo,
                                             const sensor_msgs::CameraInfoConstPtr &depthCameraInfo,
                                             const sensor_msgs::ImuConstPtr &imu)
{
  // ----> Frequency calculation
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
  last_time = now;

  double freq = 1e6 / elapsed_usec;
  NODELET_DEBUG("Freq: %.2f", freq);
  // <---- Frequency calculation

  double rgbStamp = rgb->header.stamp.toSec();
  double depthStamp = depth->header.stamp.toSec();
  double rgbInfoStamp = rgbCameraInfo->header.stamp.toSec();
  double depthInfoStamp = depthCameraInfo->header.stamp.toSec();
  double imuStamp = imu->header.stamp.toSec();

  double diff = rgbStamp - imuStamp;

  NODELET_DEBUG("Callback RGBD+IMU - RGB TS: %.9f - IMU TS: %.9f - Diff:  %.9f", rgbStamp, imuStamp, diff);

  uint32_t subraw = mPubRaw.getNumSubscribers();

  if (subraw == 0)
  {
    return;
  }

  zed_interfaces::RGBDSensorsPtr outSyncMsg = boost::make_shared<zed_interfaces::RGBDSensors>();

  outSyncMsg->header.frame_id = rgb->header.frame_id;
  outSyncMsg->header.stamp = rgb->header.stamp;
  outSyncMsg->rgbCameraInfo = *rgbCameraInfo;
  outSyncMsg->depthCameraInfo = *depthCameraInfo;

  outSyncMsg->rgb = *rgb;
  outSyncMsg->depth = *depth;
  outSyncMsg->imu = *imu;

  mPubRaw.publish(outSyncMsg);

  if (rgbStamp != rgb->header.stamp.toSec() || imuStamp != imu->header.stamp.toSec())
  {
    NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
                  "sure the node publishing the topics doesn't override the same data after publishing them. A "
                  "solution is to use this node within another nodelet manager. ");
    NODELET_ERROR("Stamps: "
                  "rgb=%f->%f IMU=%f->%f",
                  rgbStamp, rgb->header.stamp.toSec(), imuStamp, imu->header.stamp.toSec());
  }
}

void RgbdSensorsSyncNodelet::callbackRGBDMag(const sensor_msgs::ImageConstPtr &rgb,
                                             const sensor_msgs::ImageConstPtr &depth,
                                             const sensor_msgs::CameraInfoConstPtr &rgbCameraInfo,
                                             const sensor_msgs::CameraInfoConstPtr &depthCameraInfo,
                                             const sensor_msgs::MagneticFieldConstPtr &mag)
{
  // ----> Frequency calculation
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
  last_time = now;

  double freq = 1e6 / elapsed_usec;
  NODELET_DEBUG("Freq: %.2f", freq);
  // <---- Frequency calculation

  double rgbStamp = rgb->header.stamp.toSec();
  double depthStamp = depth->header.stamp.toSec();
  double rgbInfoStamp = rgbCameraInfo->header.stamp.toSec();
  double depthInfoStamp = depthCameraInfo->header.stamp.toSec();
  double magStamp = mag->header.stamp.toSec();

  double diffMag = rgbStamp - magStamp;

  NODELET_DEBUG("Callback rgbd+mag - RGB TS: %.9f - MAG TS: %.9f - Diff MAG:  %.9f", rgbStamp, magStamp, diffMag);

  uint32_t subraw = mPubRaw.getNumSubscribers();

  if (subraw == 0)
  {
    return;
  }

  zed_interfaces::RGBDSensorsPtr outSyncMsg = boost::make_shared<zed_interfaces::RGBDSensors>();

  outSyncMsg->header.frame_id = rgb->header.frame_id;
  outSyncMsg->header.stamp = rgb->header.stamp;
  outSyncMsg->rgbCameraInfo = *rgbCameraInfo;
  outSyncMsg->depthCameraInfo = *depthCameraInfo;

  outSyncMsg->rgb = *rgb;
  outSyncMsg->depth = *depth;
  outSyncMsg->mag = *mag;

  mPubRaw.publish(outSyncMsg);

  if (rgbStamp != rgb->header.stamp.toSec() || magStamp != mag->header.stamp.toSec())
  {
    NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
                  "sure the node publishing the topics doesn't override the same data after publishing them. A "
                  "solution is to use this node within another nodelet manager. ");
    NODELET_ERROR("Stamps: "
                  "rgb=%f->%f MAG=%f->%f",
                  rgbStamp, rgb->header.stamp.toSec(), magStamp, mag->header.stamp.toSec());
  }
}

void RgbdSensorsSyncNodelet::callbackFull(const sensor_msgs::ImageConstPtr &rgb,
                                          const sensor_msgs::ImageConstPtr &depth,
                                          const sensor_msgs::CameraInfoConstPtr &rgbCameraInfo,
                                          const sensor_msgs::CameraInfoConstPtr &depthCameraInfo,
                                          const sensor_msgs::ImuConstPtr &imu,
                                          const sensor_msgs::MagneticFieldConstPtr &mag)
{
  // ----> Frequency calculation
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
  last_time = now;

  double freq = 1e6 / elapsed_usec;
  NODELET_DEBUG("Freq: %.2f", freq);
  // <---- Frequency calculation

  double rgbStamp = rgb->header.stamp.toSec();
  double depthStamp = depth->header.stamp.toSec();
  double rgbInfoStamp = rgbCameraInfo->header.stamp.toSec();
  double depthInfoStamp = depthCameraInfo->header.stamp.toSec();
  double imuStamp = imu->header.stamp.toSec();
  double magStamp = mag->header.stamp.toSec();

  double diffImu = rgbStamp - imuStamp;
  double diffMag = rgbStamp - magStamp;

  NODELET_DEBUG("Callback FULL - RGB TS: %.9f - IMU TS: %.9f - Diff IMU:  %.9f", rgbStamp, imuStamp, diffImu);
  NODELET_DEBUG("Callback FULL - RGB TS: %.9f - MAG TS: %.9f - Diff MAG:  %.9f", rgbStamp, magStamp, diffMag);

  uint32_t subraw = mPubRaw.getNumSubscribers();

  if (subraw == 0)
  {
    return;
  }

  zed_interfaces::RGBDSensorsPtr outSyncMsg = boost::make_shared<zed_interfaces::RGBDSensors>();

  outSyncMsg->header.frame_id = rgb->header.frame_id;
  outSyncMsg->header.stamp = rgb->header.stamp;
  outSyncMsg->rgbCameraInfo = *rgbCameraInfo;
  outSyncMsg->depthCameraInfo = *depthCameraInfo;

  outSyncMsg->rgb = *rgb;
  outSyncMsg->depth = *depth;
  outSyncMsg->imu = *imu;
  outSyncMsg->mag = *mag;

  mPubRaw.publish(outSyncMsg);

  if (rgbStamp != rgb->header.stamp.toSec() || imuStamp != imu->header.stamp.toSec() ||
      magStamp != mag->header.stamp.toSec())
  {
    NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
                  "sure the node publishing the topics doesn't override the same data after publishing them. A "
                  "solution is to use this node within another nodelet manager. ");
    NODELET_ERROR("Stamps: "
                  "rgb=%f->%f IMU=%f->%f MAG=%f->%f",
                  rgbStamp, rgb->header.stamp.toSec(), imuStamp, imu->header.stamp.toSec(), magStamp,
                  mag->header.stamp.toSec());
  }
}

}  // namespace zed_nodelets
