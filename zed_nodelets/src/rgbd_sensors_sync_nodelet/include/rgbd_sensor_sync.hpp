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

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "zed_interfaces/RGBDSensors.h"

namespace zed_nodelets
{
class RgbdSensorsSyncNodelet : public nodelet::Nodelet
{
public:
  RgbdSensorsSyncNodelet();
  virtual ~RgbdSensorsSyncNodelet();

protected:
  /*! \brief Initialization function called by the Nodelet base class
   */
  virtual void onInit();

  /*! \brief Reads parameters from the param server
   */
  void readParameters();

  /*! \brief Callback for full topics synchronization
   */
  void callbackFull(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth,
                    const sensor_msgs::CameraInfoConstPtr& rgbCameraInfo,
                    const sensor_msgs::CameraInfoConstPtr& depthCameraInfo, const sensor_msgs::ImuConstPtr& imu,
                    const sensor_msgs::MagneticFieldConstPtr& mag);

  /*! \brief Callback for RGBD topics synchronization
   */
  void callbackRGBD(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth,
                    const sensor_msgs::CameraInfoConstPtr& rgbCameraInfo,
                    const sensor_msgs::CameraInfoConstPtr& depthCameraInfo);

  /*! \brief Callback for RGBD + IMU topics synchronization
   */
  void callbackRGBDIMU(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth,
                       const sensor_msgs::CameraInfoConstPtr& rgbCameraInfo,
                       const sensor_msgs::CameraInfoConstPtr& depthCameraInfo, const sensor_msgs::ImuConstPtr& imu);

  /*! \brief Callback for RGBD + Mag topics synchronization
   */
  void callbackRGBDMag(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth,
                       const sensor_msgs::CameraInfoConstPtr& rgbCameraInfo,
                       const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
                       const sensor_msgs::MagneticFieldConstPtr& mag);

private:
  // Node handlers
  ros::NodeHandle mNh;   // Node handler
  ros::NodeHandle mNhP;  // Private Node handler

  // Publishers
  ros::Publisher mPubRaw;

  // Subscribers
  image_transport::SubscriberFilter mSubRgbImage;
  image_transport::SubscriberFilter mSubDepthImage;
  message_filters::Subscriber<sensor_msgs::CameraInfo> mSubRgbCamInfo;
  message_filters::Subscriber<sensor_msgs::CameraInfo> mSubDepthCamInfo;
  message_filters::Subscriber<sensor_msgs::Imu> mSubImu;
  message_filters::Subscriber<sensor_msgs::MagneticField> mSubMag;

  // Approx sync policies
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo, sensor_msgs::CameraInfo,
                                                          sensor_msgs::Imu, sensor_msgs::MagneticField>
      ApproxFullSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::Imu>
      ApproxRgbdImuSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo, sensor_msgs::CameraInfo,
                                                          sensor_msgs::MagneticField>
      ApproxRgbdMagSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
      ApproxRgbdSyncPolicy;
  message_filters::Synchronizer<ApproxFullSyncPolicy>* mApproxFullSync = nullptr;
  message_filters::Synchronizer<ApproxRgbdImuSyncPolicy>* mApproxRgbdImuSync = nullptr;
  message_filters::Synchronizer<ApproxRgbdMagSyncPolicy>* mApproxRgbdMagSync = nullptr;
  message_filters::Synchronizer<ApproxRgbdSyncPolicy>* mApproxRgbdSync = nullptr;

  // Exact sync policies
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo, sensor_msgs::Imu,
                                                    sensor_msgs::MagneticField>
      ExactFullSyncPolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo, sensor_msgs::Imu>
      ExactRgbdImuSyncPolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo, sensor_msgs::MagneticField>
      ExactRgbdMagSyncPolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo>
      ExactRgbdSyncPolicy;
  message_filters::Synchronizer<ExactFullSyncPolicy>* mExactFullSync = nullptr;
  message_filters::Synchronizer<ExactRgbdImuSyncPolicy>* mExactRgbdImuSync = nullptr;
  message_filters::Synchronizer<ExactRgbdMagSyncPolicy>* mExactRgbdMagSync = nullptr;
  message_filters::Synchronizer<ExactRgbdSyncPolicy>* mExactRgbdSync = nullptr;

  // Params
  std::string mZedNodeletName = "zed_node";
  bool mUseApproxSync = true;
  bool mUseImu = true;
  bool mUseMag = true;
  int mQueueSize = 50;
};

}  // namespace zed_nodelets

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_nodelets::RgbdSensorsSyncNodelet, nodelet::Nodelet)

#endif  // RGBD_SENSOR_SYNC_HPP
