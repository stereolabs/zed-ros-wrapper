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

#include "rgbd_sensor_demux.hpp"

#ifndef NDEBUG
#include <ros/console.h>
#endif

#include <chrono>

namespace zed_nodelets
{
RgbdSensorsDemuxNodelet::RgbdSensorsDemuxNodelet()
{
}

RgbdSensorsDemuxNodelet::~RgbdSensorsDemuxNodelet()
{
}

void RgbdSensorsDemuxNodelet::onInit()
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

  mSub = mNh.subscribe("rgbd_sens", 1, &RgbdSensorsDemuxNodelet::msgCallback, this);
  NODELET_INFO_STREAM(" * Subscribed to topic: " << mSub.getTopic().c_str());
}

void RgbdSensorsDemuxNodelet::msgCallback(const zed_interfaces::RGBDSensorsPtr &msg)
{
  if (!msg->rgb.header.stamp.isZero())
  {
    if (mPubRgb.getTopic().empty())
    {
      ros::NodeHandle rgb_pnh(mNhP, "rgb");
      image_transport::ImageTransport it(rgb_pnh);

      mPubRgb = it.advertiseCamera("image_rect_color", 1);  // rgb
      NODELET_INFO_STREAM("Advertised on topic " << mPubRgb.getTopic());
      NODELET_INFO_STREAM("Advertised on topic " << mPubRgb.getInfoTopic());
    }

    if (mPubRgb.getNumSubscribers() > 0)
    {
      mPubRgb.publish(msg->rgb, msg->rgbCameraInfo);
    }
  }

  if (!msg->depth.header.stamp.isZero())
  {
    if (mPubDepth.getTopic().empty())
    {
      ros::NodeHandle depth_pnh(mNhP, "depth");
      image_transport::ImageTransport it(depth_pnh);

      mPubDepth = it.advertiseCamera("depth_registered", 1);  // depth
      NODELET_INFO_STREAM("Advertised on topic " << mPubDepth.getTopic());
      NODELET_INFO_STREAM("Advertised on topic " << mPubDepth.getInfoTopic());
    }

    if (mPubDepth.getNumSubscribers() > 0)
    {
      mPubDepth.publish(msg->depth, msg->depthCameraInfo);
    }
  }

  if (!msg->imu.header.stamp.isZero())
  {
    if (mPubIMU.getTopic().empty())
    {
      ros::NodeHandle imu_pnh(mNhP, "imu");

      mPubIMU = imu_pnh.advertise<sensor_msgs::Imu>("data", 1);  // IMU
      NODELET_INFO_STREAM("Advertised on topic " << mPubIMU.getTopic());
    }

    if (mPubIMU.getNumSubscribers() > 0)
    {
      mPubIMU.publish(msg->imu);
    }
  }

  if (!msg->mag.header.stamp.isZero())
  {
    if (mPubMag.getTopic().empty())
    {
      ros::NodeHandle imu_pnh(mNhP, "imu");

      mPubMag = imu_pnh.advertise<sensor_msgs::MagneticField>("mag", 1);  // IMU
      NODELET_INFO_STREAM("Advertised on topic " << mPubMag.getTopic());
    }

    if (mPubMag.getNumSubscribers() > 0)
    {
      mPubMag.publish(msg->mag);
    }
  }
}

}  // namespace zed_nodelets
