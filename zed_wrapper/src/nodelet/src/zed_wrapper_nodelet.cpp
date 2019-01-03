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

#include "zed_wrapper_nodelet.hpp"

#ifndef NDEBUG
#include <ros/console.h>
#endif

#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>

using namespace std;

namespace zed_wrapper {

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

    ZEDWrapperNodelet::ZEDWrapperNodelet() : Nodelet() {}

    ZEDWrapperNodelet::~ZEDWrapperNodelet() {
        if (mDevicePollThread.joinable()) {
            mDevicePollThread.join();
        }

        if (mPcThread.joinable()) {
            mPcThread.join();
        }
    }

    void ZEDWrapperNodelet::onInit() {

        mStopNode = false;
        mPcDataReady = false;

#ifndef NDEBUG

        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                           ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

#endif
        // Launch file parameters
        mCamResol = sl::RESOLUTION_HD720;
        mCamQuality = sl::DEPTH_MODE_PERFORMANCE;
        mCamSensingMode = sl::SENSING_MODE_STANDARD;
        mCamFrameRate = 30;
        mGpuId = -1;
        mZedId = 0;
        mZedSerialNumber = 0;
        mOdometryDb = "";
        mImuPubRate = 100.0;
        mPathPubRate = 1.0;
        mInitialTrackPose.resize(6);
        mOpenniDepthMode = false;
        mTrackingReady = false;

        for (size_t i = 0; i < 6; i++) {
            mInitialTrackPose[i] = 0.0f;
        }

        mCamMatResizeFactor = 1.0;
        mVerbose = true;
        mNh = getMTNodeHandle();
        mNhNs = getMTPrivateNodeHandle();
        // Set  default coordinate frames
        mNhNs.param<std::string>("pose_frame", mMapFrameId, "map");
        mNhNs.param<std::string>("odometry_frame", mOdometryFrameId, "odom");
        mNhNs.param<std::string>("base_frame", mBaseFrameId, "base_link");
        mNhNs.param<std::string>("camera_frame", mCameraFrameId, "zed_camera_center");
        mNhNs.param<std::string>("imu_frame", mImuFrameId, "imu_link");
        mNhNs.param<std::string>("left_camera_frame", mLeftCamFrameId,
                                 "left_camera_frame");
        mNhNs.param<std::string>("left_camera_optical_frame", mLeftCamOptFrameId,
                                 "left_camera_optical_frame");
        mNhNs.param<std::string>("right_camera_frame", mRightCamFrameId,
                                 "right_camera_frame");
        mNhNs.param<std::string>("right_camera_optical_frame", mRightCamOptFrameId,
                                 "right_camera_optical_frame");
        mDepthFrameId = mLeftCamFrameId;
        mDepthOptFrameId = mLeftCamOptFrameId;

        // Note: Depth image frame id must match color image frame id
        mCloudFrameId = mDepthOptFrameId;
        mRgbFrameId = mDepthFrameId;
        mRgbOptFrameId = mCloudFrameId;
        mDisparityFrameId = mDepthFrameId;
        mDisparityOptFrameId = mDepthOptFrameId;
        mConfidenceFrameId = mDepthFrameId;
        mConfidenceOptFrameId = mDepthOptFrameId;

        // Get parameters from launch file
        mNhNs.getParam("resolution", mCamResol);
        mNhNs.getParam("frame_rate", mCamFrameRate);
        checkResolFps();
        mNhNs.getParam("verbose", mVerbose);
        mNhNs.getParam("quality", mCamQuality);
        mNhNs.getParam("sensing_mode", mCamSensingMode);
        mNhNs.getParam("openni_depth_mode", mOpenniDepthMode);
        mNhNs.getParam("gpu_id", mGpuId);
        mNhNs.getParam("zed_id", mZedId);
        mNhNs.getParam("depth_stabilization", mDepthStabilization);

        int tmp_sn = 0;
        mNhNs.getParam("serial_number", tmp_sn);

        if (tmp_sn > 0) {
            mZedSerialNumber = static_cast<int>(tmp_sn);
        }

        mNhNs.getParam("camera_model", mZedUserCamModel);

        mNhNs.getParam("publish_pose_covariance", mPublishPoseCovariance);

        // Publish odometry tf
        mNhNs.param<bool>("publish_tf", mPublishTf, true);
        mNhNs.param<bool>("publish_map_tf", mPublishMapTf, true);
        mNhNs.param<bool>("camera_flip", mCameraFlip, false);

        if (mZedSerialNumber > 0) {
            NODELET_INFO_STREAM("SN : " << mZedSerialNumber);
        }

        // Print order frames
        NODELET_INFO_STREAM("pose_frame \t\t   -> " << mMapFrameId);
        NODELET_INFO_STREAM("odometry_frame \t\t   -> " << mOdometryFrameId);
        NODELET_INFO_STREAM("base_frame \t\t   -> " << mBaseFrameId);
        NODELET_INFO_STREAM("camera_frame \t\t   -> " << mCameraFrameId);
        NODELET_INFO_STREAM("imu_link \t\t   -> " << mImuFrameId);
        NODELET_INFO_STREAM("left_camera_frame \t   -> " << mLeftCamFrameId);
        NODELET_INFO_STREAM("left_camera_optical_frame  -> " << mLeftCamOptFrameId);
        NODELET_INFO_STREAM("right_camera_frame \t   -> " << mRightCamFrameId);
        NODELET_INFO_STREAM("right_camera_optical_frame -> " << mRightCamOptFrameId);
        NODELET_INFO_STREAM("depth_frame \t\t   -> " << mDepthFrameId);
        NODELET_INFO_STREAM("depth_optical_frame \t   -> " << mDepthOptFrameId);
        NODELET_INFO_STREAM("disparity_frame \t   -> " << mDisparityFrameId);
        NODELET_INFO_STREAM("disparity_optical_frame    -> " << mDisparityOptFrameId);

        NODELET_INFO_STREAM("Camera Flip [" << (mCameraFlip ? "TRUE" : "FALSE") << "]");

        // Status of odometry TF
        NODELET_INFO_STREAM("Broadcasting " << mOdometryFrameId << " [" << (mPublishTf ? "TRUE" : "FALSE") << "]");
        // Status of map TF
        NODELET_INFO_STREAM("Broadcasting " << mMapFrameId << " [" << ((mPublishTf &&
                            mPublishMapTf) ? "TRUE" : "FALSE") << "]");

        std::string img_topic = "image_rect_color";
        std::string img_raw_topic = "image_raw_color";
        // Set the default topic names
        string left_topic = "left/" + img_topic;
        string left_raw_topic = "left/" + img_raw_topic;
        string left_cam_info_topic = "left/camera_info";
        string left_cam_info_raw_topic = "left/camera_info_raw";
        string right_topic = "right/" + img_topic;
        string right_raw_topic = "right/" + img_raw_topic;
        string right_cam_info_topic = "right/camera_info";
        string right_cam_info_raw_topic = "right/camera_info_raw";
        string rgb_topic = "rgb/" + img_topic;
        string rgb_raw_topic = "rgb/" + img_raw_topic;
        string rgb_cam_info_topic = "rgb/camera_info";
        string rgb_cam_info_raw_topic = "rgb/camera_info_raw";

        string depth_topic = "depth/";

        if (mOpenniDepthMode) {
            NODELET_INFO_STREAM("Openni depth mode activated");
            depth_topic += "depth_raw_registered";
        } else {
            depth_topic += "depth_registered";
        }

        string depth_cam_info_topic = "depth/camera_info";
        string disparity_topic = "disparity/disparity_image";
        string point_cloud_topic = "point_cloud/cloud_registered";
        string conf_img_topic = "confidence/confidence_image";
        string conf_map_topic = "confidence/confidence_map";
        string pose_topic = "pose";
        string pose_cov_topic = "pose_with_covariance";
        string odometry_topic = "odom";
        string odom_path_topic = "path_odom";
        string map_path_topic = "path_map";
        string imu_topic = "imu/data";
        string imu_topic_raw = "imu/data_raw";
        mNhNs.getParam("rgb_topic", rgb_topic);
        mNhNs.getParam("rgb_raw_topic", rgb_raw_topic);
        mNhNs.getParam("rgb_cam_info_topic", rgb_cam_info_topic);
        mNhNs.getParam("rgb_cam_info_raw_topic", rgb_cam_info_raw_topic);
        mNhNs.getParam("left_topic", left_topic);
        mNhNs.getParam("left_raw_topic", left_raw_topic);
        mNhNs.getParam("left_cam_info_topic", left_cam_info_topic);
        mNhNs.getParam("left_cam_info_raw_topic", left_cam_info_raw_topic);
        mNhNs.getParam("right_topic", right_topic);
        mNhNs.getParam("right_raw_topic", right_raw_topic);
        mNhNs.getParam("right_cam_info_topic", right_cam_info_topic);
        mNhNs.getParam("right_cam_info_raw_topic", right_cam_info_raw_topic);
        mNhNs.getParam("depth_topic", depth_topic);
        mNhNs.getParam("depth_cam_info_topic", depth_cam_info_topic);
        mNhNs.getParam("disparity_topic", disparity_topic);
        mNhNs.getParam("confidence_img_topic", conf_img_topic);
        mNhNs.getParam("confidence_map_topic", conf_map_topic);
        mNhNs.getParam("point_cloud_topic", point_cloud_topic);
        mNhNs.getParam("pose_topic", pose_topic);
        pose_cov_topic = pose_topic + "_with_covariance";
        mNhNs.getParam("odometry_topic", odometry_topic);
        mNhNs.getParam("imu_topic", imu_topic);
        mNhNs.getParam("imu_topic_raw", imu_topic_raw);
        mNhNs.getParam("imu_timestamp_sync", mImuTimestampSync);
        mNhNs.getParam("imu_pub_rate", mImuPubRate);
        mNhNs.getParam("path_pub_rate", mPathPubRate);
        mNhNs.getParam("path_max_count", mPathMaxCount);

        if (mPathMaxCount < 2 && mPathMaxCount != -1) {
            mPathMaxCount = 2;
        }

        // Create camera info
        sensor_msgs::CameraInfoPtr rgb_cam_info_msg_(new sensor_msgs::CameraInfo());
        sensor_msgs::CameraInfoPtr left_cam_info_msg_(new sensor_msgs::CameraInfo());
        sensor_msgs::CameraInfoPtr right_cam_info_msg_(new sensor_msgs::CameraInfo());
        sensor_msgs::CameraInfoPtr rgb_cam_info_raw_msg_(new sensor_msgs::CameraInfo());
        sensor_msgs::CameraInfoPtr left_cam_info_raw_msg_(new sensor_msgs::CameraInfo());
        sensor_msgs::CameraInfoPtr right_cam_info_raw_msg_(new sensor_msgs::CameraInfo());
        sensor_msgs::CameraInfoPtr depth_cam_info_msg_(new sensor_msgs::CameraInfo());
        mRgbCamInfoMsg = rgb_cam_info_msg_;
        mLeftCamInfoMsg = left_cam_info_msg_;
        mRightCamInfoMsg = right_cam_info_msg_;
        mRgbCamInfoRawMsg = rgb_cam_info_raw_msg_;
        mLeftCamInfoRawMsg = left_cam_info_raw_msg_;
        mRightCamInfoRawMsg = right_cam_info_raw_msg_;
        mDepthCamInfoMsg = depth_cam_info_msg_;

        // SVO
        mNhNs.param<std::string>("svo_filepath", mSvoFilepath, std::string());

        // Initialization transformation listener
        mTfBuffer.reset(new tf2_ros::Buffer);
        mTfListener.reset(new tf2_ros::TransformListener(*mTfBuffer));

        // Initialize tf2 transformation
        mNhNs.getParam("initial_tracking_pose", mInitialTrackPose);
        set_pose(mInitialTrackPose[0], mInitialTrackPose[1], mInitialTrackPose[2],
                 mInitialTrackPose[3], mInitialTrackPose[4], mInitialTrackPose[5]);


        // Try to initialize the ZED
        if (!mSvoFilepath.empty()) {
            mZedParams.svo_input_filename = mSvoFilepath.c_str();
            mZedParams.svo_real_time_mode = true;

            //            mPubClock = mNh.advertise<rosgraph_msgs::Clock>("/clock", 1);
            //            NODELET_INFO("Advertised on topic /clock");
            mSvoMode = true;
        } else {
            mZedParams.camera_fps = mCamFrameRate;
            mZedParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);

            if (mZedSerialNumber == 0) {
                mZedParams.camera_linux_id = mZedId;
            } else {
                bool waiting_for_camera = true;

                while (waiting_for_camera) {
                    // Ctrl+C check
                    if (!mNhNs.ok()) {
                        mStopNode = true; // Stops other threads

                        std::lock_guard<std::mutex> lock(mCloseZedMutex);
                        NODELET_DEBUG("Closing ZED");
                        mZed.close();

                        NODELET_DEBUG("ZED pool thread finished");
                        return;
                    }

                    sl::DeviceProperties prop = sl_tools::getZEDFromSN(mZedSerialNumber);

                    if (prop.id < -1 ||
                        prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                        std::string msg = "ZED SN" + to_string(mZedSerialNumber) +
                                          " not detected ! Please connect this ZED";
                        NODELET_INFO_STREAM(msg.c_str());
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    } else {
                        waiting_for_camera = false;
                        mZedParams.camera_linux_id = prop.id;
                    }
                }
            }
        }

        std::string ver = sl_tools::getSDKVersion(mVerMajor, mVerMinor, mVerSubMinor);
        NODELET_INFO_STREAM("SDK version : " << ver);

#if (ZED_SDK_MAJOR_VERSION<2)
        NODELET_WARN_STREAM("Please consider to upgrade to latest SDK version to "
                            "get better performances");

        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;

        NODELET_INFO_STREAM("Camera coordinate system : COORDINATE_SYSTEM_IMAGE");
        mIdxX = 2;
        mIdxY = 0;
        mIdxZ = 1;
        mSignX = 1;
        mSignY = -1;
        mSignZ = -1;
#elif (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION<5)
        NODELET_WARN_STREAM("Please consider to upgrade to latest SDK version to "
                            "get latest features");

        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;

        NODELET_INFO_STREAM("Camera coordinate system : COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP");
        mIdxX = 1;
        mIdxY = 0;
        mIdxZ = 2;
        mSignX = 1;
        mSignY = -1;
        mSignZ = 1;
#else
        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;

        NODELET_INFO_STREAM("Camera coordinate system : COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD");
        mIdxX = 0;
        mIdxY = 1;
        mIdxZ = 2;
        mSignX = 1;
        mSignY = 1;
        mSignZ = 1;
#endif

        mZedParams.coordinate_units = sl::UNIT_METER;
        mZedParams.depth_mode = static_cast<sl::DEPTH_MODE>(mCamQuality);
        mZedParams.sdk_verbose = mVerbose;
        mZedParams.sdk_gpu_id = mGpuId;
        mZedParams.depth_stabilization = mDepthStabilization;
        mZedParams.camera_image_flip = mCameraFlip;

        mDiagUpdater.add("ZED Diagnostic", this, &ZEDWrapperNodelet::updateDiagnostic);
        mDiagUpdater.setHardwareID("ZED camera");

        mConnStatus = sl::ERROR_CODE_CAMERA_NOT_DETECTED;

        while (mConnStatus != sl::SUCCESS) {
            mConnStatus = mZed.open(mZedParams);
            NODELET_INFO_STREAM(toString(mConnStatus));
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            if (!mNhNs.ok()) {
                mStopNode = true; // Stops other threads

                std::lock_guard<std::mutex> lock(mCloseZedMutex);
                NODELET_DEBUG("Closing ZED");
                mZed.close();

                NODELET_DEBUG("ZED pool thread finished");
                return;
            }

            mDiagUpdater.update();
        }

        mZedRealCamModel = mZed.getCameraInformation().camera_model;
        std::string camModelStr = "LAST";

        if (mZedRealCamModel == sl::MODEL_ZED) {
            camModelStr = "ZED";

            if (mZedUserCamModel != 0) {
                NODELET_WARN("Camera model does not match user parameter. Please modify "
                             "the value of the parameter 'camera_model' to 0");
            }
        } else if (mZedRealCamModel == sl::MODEL_ZED_M) {
            camModelStr = "ZED M";

            if (mZedUserCamModel != 1) {
                NODELET_WARN("Camera model does not match user parameter. Please modify "
                             "the value of the parameter 'camera_model' to 1");
            }
        }

        NODELET_INFO_STREAM("CAMERA MODEL : " << mZedRealCamModel);
        mZedSerialNumber = mZed.getCameraInformation().serial_number;

        mDiagUpdater.setHardwareIDf("%s-%d", sl::toString(mZedRealCamModel).c_str(), mZedSerialNumber);

        // Dynamic Reconfigure parameters
        mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<zed_wrapper::ZedConfig>>();
        dynamic_reconfigure::Server<zed_wrapper::ZedConfig>::CallbackType f;
        f = boost::bind(&ZEDWrapperNodelet::dynamicReconfCallback, this, _1, _2);
        mDynRecServer->setCallback(f);

        mNhNs.getParam("mat_resize_factor", mCamMatResizeFactor);

        if (mCamMatResizeFactor < 0.1) {
            mCamMatResizeFactor = 0.1;
            NODELET_WARN_STREAM(
                "Minimum allowed values for 'mat_resize_factor' is 0.1");
        }

        if (mCamMatResizeFactor > 1.0) {
            mCamMatResizeFactor = 1.0;
            NODELET_WARN_STREAM(
                "Maximum allowed values for 'mat_resize_factor' is 1.0");
        }

        mNhNs.getParam("confidence", mCamConfidence);
        mNhNs.getParam("max_depth", mCamMaxDepth);
        mNhNs.getParam("exposure", mCamExposure);
        mNhNs.getParam("gain", mCamGain);
        mNhNs.getParam("auto_exposure", mCamAutoExposure);

        if (mCamAutoExposure) {
            mTriggerAutoExposure = true;
        }

        // Create all the publishers
        // Image publishers
        image_transport::ImageTransport it_zed(mNh);
        mPubRgb = it_zed.advertise(rgb_topic, 1); // rgb
        NODELET_INFO_STREAM("Advertised on topic " << rgb_topic);
        mPubRawRgb = it_zed.advertise(rgb_raw_topic, 1); // rgb raw
        NODELET_INFO_STREAM("Advertised on topic " << rgb_raw_topic);
        mPubLeft = it_zed.advertise(left_topic, 1); // left
        NODELET_INFO_STREAM("Advertised on topic " << left_topic);
        mPubRawLeft = it_zed.advertise(left_raw_topic, 1); // left raw
        NODELET_INFO_STREAM("Advertised on topic " << left_raw_topic);
        mPubRight = it_zed.advertise(right_topic, 1); // right
        NODELET_INFO_STREAM("Advertised on topic " << right_topic);
        mPubRawRight = it_zed.advertise(right_raw_topic, 1); // right raw
        NODELET_INFO_STREAM("Advertised on topic " << right_raw_topic);
        mPubDepth = it_zed.advertise(depth_topic, 1); // depth
        NODELET_INFO_STREAM("Advertised on topic " << depth_topic);
        mPubConfImg = it_zed.advertise(conf_img_topic, 1); // confidence image
        NODELET_INFO_STREAM("Advertised on topic " << conf_img_topic);

        // Confidence Map publisher
        mPubConfMap = mNh.advertise<sensor_msgs::Image>(conf_map_topic, 1); // confidence map
        NODELET_INFO_STREAM("Advertised on topic " << conf_map_topic);

        // Disparity publisher
        mPubDisparity = mNh.advertise<stereo_msgs::DisparityImage>(disparity_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << disparity_topic);

        // PointCloud publisher
        mPointcloudMsg.reset(new sensor_msgs::PointCloud2);
        mPubCloud = mNh.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << point_cloud_topic);

        // Camera info publishers
        mPubRgbCamInfo = mNh.advertise<sensor_msgs::CameraInfo>(rgb_cam_info_topic, 1); // rgb
        NODELET_INFO_STREAM("Advertised on topic " << rgb_cam_info_topic);
        mPubLeftCamInfo = mNh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1); // left
        NODELET_INFO_STREAM("Advertised on topic " << left_cam_info_topic);
        mPubRightCamInfo = mNh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1); // right
        NODELET_INFO_STREAM("Advertised on topic " << right_cam_info_topic);
        mPubDepthCamInfo = mNh.advertise<sensor_msgs::CameraInfo>(depth_cam_info_topic, 1); // depth
        NODELET_INFO_STREAM("Advertised on topic " << depth_cam_info_topic);

        // Raw
        mPubRgbCamInfoRaw = mNh.advertise<sensor_msgs::CameraInfo>(rgb_cam_info_raw_topic, 1); // raw rgb
        NODELET_INFO_STREAM("Advertised on topic " << rgb_cam_info_raw_topic);
        mPubLeftCamInfoRaw = mNh.advertise<sensor_msgs::CameraInfo>(left_cam_info_raw_topic, 1); // raw left
        NODELET_INFO_STREAM("Advertised on topic " << left_cam_info_raw_topic);
        mPubRightCamInfoRaw = mNh.advertise<sensor_msgs::CameraInfo>(right_cam_info_raw_topic, 1); // raw right
        NODELET_INFO_STREAM("Advertised on topic " << right_cam_info_raw_topic);

        // Odometry and Map publisher
        mPubPose = mNh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << pose_topic);

        if (mPublishPoseCovariance) {
            mPubPoseCov = mNh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_cov_topic, 1);
            NODELET_INFO_STREAM("Advertised on topic " << pose_cov_topic);
        }

        mPubOdom = mNh.advertise<nav_msgs::Odometry>(odometry_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << odometry_topic);

        // Camera Path
        if (mPathPubRate > 0) {
            mPubOdomPath = mNh.advertise<nav_msgs::Path>(odom_path_topic, 1, true);
            NODELET_INFO_STREAM("Advertised on topic " << odom_path_topic);
            mPubMapPath = mNh.advertise<nav_msgs::Path>(map_path_topic, 1, true);
            NODELET_INFO_STREAM("Advertised on topic " << map_path_topic);

            mPathTimer = mNhNs.createTimer(ros::Duration(1.0 / mPathPubRate),
                                           &ZEDWrapperNodelet::pathPubCallback, this);

            if (mPathMaxCount != -1) {
                NODELET_DEBUG_STREAM("Path vectors reserved " << mPathMaxCount << " poses.");
                mOdomPath.reserve(mPathMaxCount);
                mMapPath.reserve(mPathMaxCount);

                NODELET_DEBUG_STREAM("Path vector sizes: " << mOdomPath.size() << " " << mMapPath.size());
            }
        } else {
            NODELET_INFO_STREAM("Path topics not published -> mPathPubRate: " << mPathPubRate);
        }

        // Imu publisher
        if (mImuPubRate > 0 && mZedRealCamModel == sl::MODEL_ZED_M) {
            mPubImu = mNh.advertise<sensor_msgs::Imu>(imu_topic, 500);
            NODELET_INFO_STREAM("Advertised on topic " << imu_topic << " @ "
                                << mImuPubRate << " Hz");
            mPubImuRaw = mNh.advertise<sensor_msgs::Imu>(imu_topic_raw, 500);
            NODELET_INFO_STREAM("Advertised on topic " << imu_topic_raw << " @ "
                                << mImuPubRate << " Hz");
            mFrameTimestamp = ros::Time::now();
            mImuTimer = mNhNs.createTimer(ros::Duration(1.0 / mImuPubRate),
                                          &ZEDWrapperNodelet::imuPubCallback, this);
            mImuPeriodMean_usec.reset(new sl_tools::CSmartMean(mImuPubRate / 2));
        } else if (mImuPubRate > 0 && mZedRealCamModel == sl::MODEL_ZED) {
            NODELET_WARN_STREAM(
                "'imu_pub_rate' set to "
                << mImuPubRate << " Hz"
                << " but ZED camera model does not support IMU data publishing.");
        }

        // Services
        mSrvSetInitPose = mNh.advertiseService("set_initial_pose", &ZEDWrapperNodelet::on_set_pose, this);
        mSrvResetOdometry = mNh.advertiseService("reset_odometry", &ZEDWrapperNodelet::on_reset_odometry, this);
        mSrvResetTracking = mNh.advertiseService("reset_tracking", &ZEDWrapperNodelet::on_reset_tracking, this);

        // Start Pointcloud thread
        mPcThread = std::thread(&ZEDWrapperNodelet::pointcloud_thread_func, this);

        // Start pool thread
        mDevicePollThread = std::thread(&ZEDWrapperNodelet::device_poll_thread_func, this);


    }

    void ZEDWrapperNodelet::checkResolFps() {
        switch (mCamResol) {
        case sl::RESOLUTION_HD2K:
            if (mCamFrameRate != 15) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD2K. Set to 15 FPS.");
                mCamFrameRate = 15;
            }

            break;

        case sl::RESOLUTION_HD1080:
            if (mCamFrameRate == 15 || mCamFrameRate == 30) {
                break;
            }

            if (mCamFrameRate > 15 && mCamFrameRate < 30) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD1080. Set to 15 FPS.");
                mCamFrameRate = 15;
            } else if (mCamFrameRate > 30) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD1080. Set to 30 FPS.");
                mCamFrameRate = 30;
            } else {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD1080. Set to 15 FPS.");
                mCamFrameRate = 15;
            }

            break;

        case sl::RESOLUTION_HD720:
            if (mCamFrameRate == 15 || mCamFrameRate == 30 || mCamFrameRate == 60) {
                break;
            }

            if (mCamFrameRate > 15 && mCamFrameRate < 30) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD720. Set to 15 FPS.");
                mCamFrameRate = 15;
            } else if (mCamFrameRate > 30 && mCamFrameRate < 60) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD720. Set to 30 FPS.");
                mCamFrameRate = 30;
            } else if (mCamFrameRate > 60) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD720. Set to 60 FPS.");
                mCamFrameRate = 60;
            } else {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution HD720. Set to 15 FPS.");
                mCamFrameRate = 15;
            }

            break;

        case sl::RESOLUTION_VGA:
            if (mCamFrameRate == 15 || mCamFrameRate == 30 || mCamFrameRate == 60 ||
                mCamFrameRate == 100) {
                break;
            }

            if (mCamFrameRate > 15 && mCamFrameRate < 30) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution VGA. Set to 15 FPS.");
                mCamFrameRate = 15;
            } else if (mCamFrameRate > 30 && mCamFrameRate < 60) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution VGA. Set to 30 FPS.");
                mCamFrameRate = 30;
            } else if (mCamFrameRate > 60 && mCamFrameRate < 100) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution VGA. Set to 60 FPS.");
                mCamFrameRate = 60;
            } else if (mCamFrameRate > 100) {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution VGA. Set to 100 FPS.");
                mCamFrameRate = 100;
            } else {
                NODELET_WARN_STREAM("Wrong FrameRate ("
                                    << mCamFrameRate
                                    << ") for the resolution VGA. Set to 15 FPS.");
                mCamFrameRate = 15;
            }

            break;

        default:
            NODELET_WARN_STREAM("Invalid resolution. Set to HD720 @ 30 FPS");
            mCamResol = 2;
            mCamFrameRate = 30;
        }
    }

    void ZEDWrapperNodelet::initTransforms() {
        // Dynamic transforms
        mOdom2BaseTransf.setIdentity();
        mMap2OdomTransf.setIdentity();

        // Static transforms
        // Sensor to Base link
        try {
            // Save the transformation from base to frame
            geometry_msgs::TransformStamped s2b =
                mTfBuffer->lookupTransform(mDepthFrameId, mBaseFrameId, mFrameTimestamp); // Coordinates of the base in sensor frame
            // Get the TF2 transformation
            tf2::fromMsg(s2b.transform, mSensor2BaseTransf);

#if 0 //#ifndef NDEBUG // Enable for TF checking
            double roll, pitch, yaw;
            tf2::Matrix3x3(mSensor2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

            NODELET_DEBUG("Sensor2Base [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                          mDepthFrameId.c_str(), mBaseFrameId.c_str(),
                          mSensor2BaseTransf.getOrigin().x(), mSensor2BaseTransf.getOrigin().y(), mSensor2BaseTransf.getOrigin().z(),
                          roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif


        } catch (tf2::TransformException& ex) {
            NODELET_WARN_THROTTLE(
                10.0, "The tf from '%s' to '%s' does not seem to be available, "
                "will assume it as identity!",
                mDepthFrameId.c_str(), mBaseFrameId.c_str());
            NODELET_DEBUG("Transform error: %s", ex.what());
            mSensor2BaseTransf.setIdentity();
        }

    }

    void ZEDWrapperNodelet::set_pose(float xt, float yt, float zt, float rr,
                                     float pr, float yr) {
        initTransforms();

        // Apply Base to sensor transform
        tf2::Transform initPose;
        tf2::Vector3 origin(xt, yt, zt);
        initPose.setOrigin(origin);
        tf2::Quaternion quat;
        quat.setRPY(rr, pr, yr);
        initPose.setRotation(quat);

        initPose = initPose * mSensor2BaseTransf.inverse();

        // SL pose
        sl::float3 t_vec;
        t_vec[0] = initPose.getOrigin().x();
        t_vec[1] = initPose.getOrigin().y();
        t_vec[2] = initPose.getOrigin().z();

        sl::float4 q_vec;
        q_vec[0] = initPose.getRotation().x();
        q_vec[1] = initPose.getRotation().y();
        q_vec[2] = initPose.getRotation().z();
        q_vec[3] = initPose.getRotation().w();

        sl::Translation trasl(t_vec);
        sl::Orientation orient(q_vec);
        mInitialPoseSl.setTranslation(trasl);
        mInitialPoseSl.setOrientation(orient);
    }

    bool ZEDWrapperNodelet::on_set_pose(
        zed_wrapper::set_initial_pose::Request& req,
        zed_wrapper::set_initial_pose::Response& res) {
        mInitialTrackPose.resize(6);
        mInitialTrackPose[0] = req.x;
        mInitialTrackPose[1] = req.y;
        mInitialTrackPose[2] = req.z;
        mInitialTrackPose[3] = req.R;
        mInitialTrackPose[4] = req.P;
        mInitialTrackPose[5] = req.Y;

        set_pose(mInitialTrackPose[0], mInitialTrackPose[1], mInitialTrackPose[2],
                 mInitialTrackPose[3], mInitialTrackPose[4], mInitialTrackPose[5]);

        if (mTrackingActivated) {
            mZed.resetTracking(mInitialPoseSl);
        }

        res.done = true;
        return true;
    }

    bool ZEDWrapperNodelet::on_reset_tracking(
        zed_wrapper::reset_tracking::Request& req,
        zed_wrapper::reset_tracking::Response& res) {
        if (!mTrackingActivated) {
            res.reset_done = false;
            return false;
        }

        mNhNs.getParam("initial_tracking_pose", mInitialTrackPose);

        if (mInitialTrackPose.size() != 6) {
            NODELET_WARN_STREAM("Invalid Initial Pose size (" << mInitialTrackPose.size()
                                << "). Using Identity");
            mInitialPoseSl.setIdentity();
        }

        set_pose(mInitialTrackPose[0], mInitialTrackPose[1], mInitialTrackPose[2],
                 mInitialTrackPose[3], mInitialTrackPose[4], mInitialTrackPose[5]);

        if (mZed.resetTracking(mInitialPoseSl) == sl::SUCCESS) {
            return true;
        }

        return false;
    }

    bool ZEDWrapperNodelet::on_reset_odometry(
        zed_wrapper::reset_odometry::Request& req,
        zed_wrapper::reset_odometry::Response& res) {
        mResetOdom = true;
        res.reset_done = true;
        return true;
    }

    void ZEDWrapperNodelet::start_tracking() {
        NODELET_INFO_STREAM("*** Starting Positional Tracking ***");
        mNhNs.getParam("odometry_DB", mOdometryDb);
        mNhNs.getParam("pose_smoothing", mPoseSmoothing);
        mNhNs.getParam("spatial_memory", mSpatialMemory);
        mNhNs.getParam("floor_alignment", mFloorAlignment);
        mNhNs.getParam("init_odom_with_first_valid_pose", mInitOdomWithPose);
        NODELET_INFO_STREAM("Init Odometry with first valid pose data : " << (mInitOdomWithPose ? "ENABLED" : "DISABLED"));

        if (mInitialTrackPose.size() != 6) {
            NODELET_WARN_STREAM("Invalid Initial Pose size (" << mInitialTrackPose.size()
                                << "). Using Identity");
            mInitialPoseSl.setIdentity();
        }

        set_pose(mInitialTrackPose[0], mInitialTrackPose[1], mInitialTrackPose[2],
                 mInitialTrackPose[3], mInitialTrackPose[4], mInitialTrackPose[5]);

        if (mOdometryDb != "" && !sl_tools::file_exist(mOdometryDb)) {
            mOdometryDb = "";
            NODELET_WARN("odometry_DB path doesn't exist or is unreachable.");
        }

        // Tracking parameters
        sl::TrackingParameters trackParams;
        trackParams.area_file_path = mOdometryDb.c_str();
        trackParams.enable_pose_smoothing = mPoseSmoothing;
        NODELET_INFO_STREAM("Pose Smoothing : " << (trackParams.enable_pose_smoothing ? "ENABLED" : "DISABLED"));
        trackParams.enable_spatial_memory = mSpatialMemory;
        NODELET_INFO_STREAM("Spatial Memory : " << (trackParams.enable_spatial_memory ? "ENABLED" : "DISABLED"));
        trackParams.initial_world_transform = mInitialPoseSl;

#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=6))
        trackParams.set_floor_as_origin = mFloorAlignment;
        NODELET_INFO_STREAM("Floor Alignment : " << (trackParams.set_floor_as_origin ? "ENABLED" : "DISABLED"));
#else

        if (mFloorAlignment)  {
            NODELET_WARN("Floor Alignment is available with ZED SDK >= v2.6");
        }

#endif

        mZed.enableTracking(trackParams);
        mTrackingActivated = true;
        NODELET_INFO("Tracking ENABLED");
    }

    void ZEDWrapperNodelet::publishOdom(tf2::Transform odom2baseTransf, sl::Pose& slPose, ros::Time t) {
        nav_msgs::Odometry odom;
        odom.header.stamp = t;
        odom.header.frame_id = mOdometryFrameId; // frame
        odom.child_frame_id = mBaseFrameId;      // camera_frame
        // conversion from Tranform to message
        geometry_msgs::Transform base2odom = tf2::toMsg(odom2baseTransf);
        // Add all value in odometry message
        odom.pose.pose.position.x = base2odom.translation.x;
        odom.pose.pose.position.y = base2odom.translation.y;
        odom.pose.pose.position.z = base2odom.translation.z;
        odom.pose.pose.orientation.x = base2odom.rotation.x;
        odom.pose.pose.orientation.y = base2odom.rotation.y;
        odom.pose.pose.orientation.z = base2odom.rotation.z;
        odom.pose.pose.orientation.w = base2odom.rotation.w;

        // Odometry pose covariance if available
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=6))

        if (!mSpatialMemory && mPublishPoseCovariance) {
            for (size_t i = 0; i < odom.pose.covariance.size(); i++) {
                // odom.pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]); // TODO USE THIS WHEN STEP BY STEP COVARIANCE WILL BE AVAILABLE IN CAMERA_FRAME
                odom.pose.covariance[i] = static_cast<double>(mLastZedPose.pose_covariance[i]);
            }
        }

#endif

        // Publish odometry message
        mPubOdom.publish(odom);
    }

    void ZEDWrapperNodelet::publishPose(ros::Time t) {
        tf2::Transform base_pose;
        base_pose.setIdentity();

        if (mPublishMapTf) {
            base_pose = mMap2BaseTransf;
        } else if (mPublishTf) {
            base_pose = mOdom2BaseTransf;
        }

        std_msgs::Header header;
        header.stamp = mFrameTimestamp;
        header.frame_id = mPublishMapTf ? mMapFrameId : mOdometryFrameId; // frame

        geometry_msgs::Pose pose;

        // conversion from Tranform to message
        geometry_msgs::Transform base2frame = tf2::toMsg(base_pose);

        // Add all value in Pose message
        pose.position.x = base2frame.translation.x;
        pose.position.y = base2frame.translation.y;
        pose.position.z = base2frame.translation.z;
        pose.orientation.x = base2frame.rotation.x;
        pose.orientation.y = base2frame.rotation.y;
        pose.orientation.z = base2frame.rotation.z;
        pose.orientation.w = base2frame.rotation.w;

        if (mPubPose.getNumSubscribers() > 0) {

            geometry_msgs::PoseStamped poseNoCov;

            poseNoCov.header = header;
            poseNoCov.pose = pose;

            // Publish pose stamped message
            mPubPose.publish(poseNoCov);
        }

        if (mPublishPoseCovariance) {
            if (mPubPoseCov.getNumSubscribers() > 0) {
                geometry_msgs::PoseWithCovarianceStamped poseCov;

                poseCov.header = header;
                poseCov.pose.pose = pose;

                // Odometry pose covariance if available
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=6))

                if (!mSpatialMemory) {
                    for (size_t i = 0; i < poseCov.pose.covariance.size(); i++) {
                        // odom.pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]); // TODO USE THIS WHEN STEP BY STEP COVARIANCE WILL BE AVAILABLE IN CAMERA_FRAME
                        poseCov.pose.covariance[i] = static_cast<double>(mLastZedPose.pose_covariance[i]);
                    }
                }

#endif

                // Publish pose with covariance stamped message
                mPubPoseCov.publish(poseCov);
            }
        }
    }

    void ZEDWrapperNodelet::publishOdomFrame(tf2::Transform odomTransf, ros::Time t) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = t;
        transformStamped.header.frame_id = mOdometryFrameId;
        transformStamped.child_frame_id = mBaseFrameId;
        // conversion from Tranform to message
        transformStamped.transform = tf2::toMsg(odomTransf);
        // Publish transformation
        mTransformOdomBroadcaster.sendTransform(transformStamped);
    }

    void ZEDWrapperNodelet::publishPoseFrame(tf2::Transform baseTransform, ros::Time t) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = t;
        transformStamped.header.frame_id = mMapFrameId;
        transformStamped.child_frame_id = mOdometryFrameId;
        // conversion from Tranform to message
        transformStamped.transform = tf2::toMsg(baseTransform);
        // Publish transformation
        mTransformPoseBroadcaster.sendTransform(transformStamped);
    }

    void ZEDWrapperNodelet::publishImuFrame(tf2::Transform imuTransform, ros::Time t) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = t;
        transformStamped.header.frame_id = mCameraFrameId;
        transformStamped.child_frame_id = mImuFrameId;
        // conversion from Tranform to message
        transformStamped.transform = tf2::toMsg(imuTransform);
        // Publish transformation
        mTransformImuBroadcaster.sendTransform(transformStamped);
    }

    void ZEDWrapperNodelet::publishImage(sl::Mat img,
                                         image_transport::Publisher& pubImg,
                                         string imgFrameId, ros::Time t) {
        pubImg.publish(sl_tools::imageToROSmsg(img, imgFrameId, t));
    }

    void ZEDWrapperNodelet::publishDepth(sl::Mat depth, ros::Time t) {

        if (!mOpenniDepthMode) {
            mPubDepth.publish(sl_tools::imageToROSmsg(depth, mDepthOptFrameId, t));
            return;
        }

        // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
        sensor_msgs::ImagePtr depthMessage = boost::make_shared<sensor_msgs::Image>();

        depthMessage->header.stamp = t;
        depthMessage->header.frame_id = mDepthOptFrameId;
        depthMessage->height = depth.getHeight();
        depthMessage->width = depth.getWidth();

        int num = 1; // for endianness detection
        depthMessage->is_bigendian = !(*(char*)&num == 1);

        depthMessage->step = depthMessage->width * sizeof(uint16_t);
        depthMessage->encoding = sensor_msgs::image_encodings::MONO16;

        size_t size = depthMessage->step * depthMessage->height;
        depthMessage->data.resize(size);

        uint16_t* data = (uint16_t*)(&depthMessage->data[0]);

        int dataSize = depthMessage->width * depthMessage->height;
        sl::float1* depthDataPtr = depth.getPtr<sl::float1>();

        for (int i = 0; i < dataSize; i++) {
            *(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++) * 1000));    // in mm, rounded
        }

        mPubDepth.publish(depthMessage);
    }

    void ZEDWrapperNodelet::publishDisparity(sl::Mat disparity, ros::Time t) {

        sl::CameraInformation zedParam =
            mZed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight));

        sensor_msgs::ImagePtr disparity_image = sl_tools::imageToROSmsg(disparity, mDisparityFrameId, t);

        stereo_msgs::DisparityImage msg;
        msg.image = *disparity_image;
        msg.header = msg.image.header;
        msg.f = zedParam.calibration_parameters.left_cam.fx;
        msg.T = zedParam.calibration_parameters.T.x;

        if (msg.T > 0) {
            msg.T *= -1.0f;
        }

        msg.min_disparity = msg.f * msg.T / mZed.getDepthMinRangeValue();
        msg.max_disparity = msg.f * msg.T / mZed.getDepthMaxRangeValue();
        mPubDisparity.publish(msg);
    }

    void ZEDWrapperNodelet::pointcloud_thread_func() {
        std::unique_lock<std::mutex> lock(mPcMutex);

        while (!mStopNode) {
            while (!mPcDataReady) {  // loop to avoid spurious wakeups
                if (mPcDataReadyCondVar.wait_for(lock, std::chrono::milliseconds(500)) == std::cv_status::timeout) {
                    // Check thread stopping
                    if (mStopNode) {
                        return;
                    } else {
                        continue;
                    }
                }
            }

            publishPointCloud();
            mPcDataReady = false;
        }

        NODELET_DEBUG("Pointcloud thread finished");
    }

    void ZEDWrapperNodelet::publishPointCloud() {
        // Publish freq calculation
        static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

        double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        last_time = now;

        mPcPeriodMean_usec->addValue(elapsed_usec);

        // Initialize Point Cloud message
        // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

        int ptsCount = mMatWidth * mMatHeight;
      
        mPointcloudMsg->header.stamp = mPointCloudTime;

        if (mPointcloudMsg->width != mMatWidth || mPointcloudMsg->height != mMatHeight) {
            mPointcloudMsg->header.frame_id = mPointCloudFrameId; // Set the header values of the ROS message

            mPointcloudMsg->is_bigendian = false;
            mPointcloudMsg->is_dense = false;

            mPointcloudMsg->width = mMatWidth;
            mPointcloudMsg->height = mMatHeight;

            sensor_msgs::PointCloud2Modifier modifier(*mPointcloudMsg);
            modifier.setPointCloud2Fields(4,
                                          "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "rgb", 1, sensor_msgs::PointField::FLOAT32);
        }

        // Data copy
        sl::Vector4<float>* cpu_cloud = mCloud.getPtr<sl::float4>();
        float* ptCloudPtr = (float*)(&mPointcloudMsg->data[0]);

#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=5) )
        memcpy(ptCloudPtr, (float*)cpu_cloud,
               4 * ptsCount * sizeof(float)); // We can do a direct memcpy since data organization is the same
#else
        #pragma omp parallel for

        for (size_t i = 0; i < ptsCount; ++i) {
            ptCloudPtr[i * 4 + 0] = mSignX * cpu_cloud[i][mIdxX];
            ptCloudPtr[i * 4 + 1] = mSignY * cpu_cloud[i][mIdxY];
            ptCloudPtr[i * 4 + 2] = mSignZ * cpu_cloud[i][mIdxZ];
            ptCloudPtr[i * 4 + 3] = cpu_cloud[i][3];
        }

#endif

        // Pointcloud publishing
        mPubCloud.publish(mPointcloudMsg);
    }

    void ZEDWrapperNodelet::publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg,
                                           ros::Publisher pubCamInfo, ros::Time t) {
        static int seq = 0;
        camInfoMsg->header.stamp = t;
        camInfoMsg->header.seq = seq;
        pubCamInfo.publish(camInfoMsg);
        seq++;
    }

    void ZEDWrapperNodelet::fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr leftCamInfoMsg,
                                        sensor_msgs::CameraInfoPtr rightCamInfoMsg, string leftFrameId,
                                        string rightFrameId, bool rawParam /*= false*/) {
        sl::CalibrationParameters zedParam;

        if (rawParam) {
            zedParam = zed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight))
                       .calibration_parameters_raw;
        } else {
            zedParam = zed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight))
                       .calibration_parameters;
        }

        float baseline = zedParam.T[0];
        leftCamInfoMsg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
        rightCamInfoMsg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
        leftCamInfoMsg->D.resize(5);
        rightCamInfoMsg->D.resize(5);
        leftCamInfoMsg->D[0] = zedParam.left_cam.disto[0];   // k1
        leftCamInfoMsg->D[1] = zedParam.left_cam.disto[1];   // k2
        leftCamInfoMsg->D[2] = zedParam.left_cam.disto[4];   // k3
        leftCamInfoMsg->D[3] = zedParam.left_cam.disto[2];   // p1
        leftCamInfoMsg->D[4] = zedParam.left_cam.disto[3];   // p2
        rightCamInfoMsg->D[0] = zedParam.right_cam.disto[0]; // k1
        rightCamInfoMsg->D[1] = zedParam.right_cam.disto[1]; // k2
        rightCamInfoMsg->D[2] = zedParam.right_cam.disto[4]; // k3
        rightCamInfoMsg->D[3] = zedParam.right_cam.disto[2]; // p1
        rightCamInfoMsg->D[4] = zedParam.right_cam.disto[3]; // p2
        leftCamInfoMsg->K.fill(0.0);
        rightCamInfoMsg->K.fill(0.0);
        leftCamInfoMsg->K[0] = static_cast<double>(zedParam.left_cam.fx);
        leftCamInfoMsg->K[2] = static_cast<double>(zedParam.left_cam.cx);
        leftCamInfoMsg->K[4] = static_cast<double>(zedParam.left_cam.fy);
        leftCamInfoMsg->K[5] = static_cast<double>(zedParam.left_cam.cy);
        leftCamInfoMsg->K[8] = 1.0;
        rightCamInfoMsg->K[0] = static_cast<double>(zedParam.right_cam.fx);
        rightCamInfoMsg->K[2] = static_cast<double>(zedParam.right_cam.cx);
        rightCamInfoMsg->K[4] = static_cast<double>(zedParam.right_cam.fy);
        rightCamInfoMsg->K[5] = static_cast<double>(zedParam.right_cam.cy);
        rightCamInfoMsg->K[8] = 1.0;
        leftCamInfoMsg->R.fill(0.0);
        rightCamInfoMsg->R.fill(0.0);

        for (size_t i = 0; i < 3; i++) {
            // identity
            rightCamInfoMsg->R[i + i * 3] = 1;
            leftCamInfoMsg->R[i + i * 3] = 1;
        }

        if (rawParam) {
            std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
            float* p = R_.data();

            for (int i = 0; i < 9; i++) {
                rightCamInfoMsg->R[i] = p[i];
            }
        }

        leftCamInfoMsg->P.fill(0.0);
        rightCamInfoMsg->P.fill(0.0);
        leftCamInfoMsg->P[0] = static_cast<double>(zedParam.left_cam.fx);
        leftCamInfoMsg->P[2] = static_cast<double>(zedParam.left_cam.cx);
        leftCamInfoMsg->P[5] = static_cast<double>(zedParam.left_cam.fy);
        leftCamInfoMsg->P[6] = static_cast<double>(zedParam.left_cam.cy);
        leftCamInfoMsg->P[10] = 1.0;
        // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
        rightCamInfoMsg->P[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
        rightCamInfoMsg->P[0] = static_cast<double>(zedParam.right_cam.fx);
        rightCamInfoMsg->P[2] = static_cast<double>(zedParam.right_cam.cx);
        rightCamInfoMsg->P[5] = static_cast<double>(zedParam.right_cam.fy);
        rightCamInfoMsg->P[6] = static_cast<double>(zedParam.right_cam.cy);
        rightCamInfoMsg->P[10] = 1.0;
        leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatWidth);
        leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatHeight);
        leftCamInfoMsg->header.frame_id = leftFrameId;
        rightCamInfoMsg->header.frame_id = rightFrameId;
    }

    void ZEDWrapperNodelet::dynamicReconfCallback(zed_wrapper::ZedConfig& config,
            uint32_t level) {
        switch (level) {
        case 0:
            mCamConfidence = config.confidence;
            NODELET_INFO("Reconfigure confidence : %d", mCamConfidence);
            break;

        case 1:
            mCamExposure = config.exposure;
            NODELET_INFO("Reconfigure exposure : %d", mCamExposure);
            break;

        case 2:
            mCamGain = config.gain;
            NODELET_INFO("Reconfigure gain : %d", mCamGain);
            break;

        case 3:
            mCamAutoExposure = config.auto_exposure;

            if (mCamAutoExposure) {
                mTriggerAutoExposure = true;
            }

            NODELET_INFO("Reconfigure auto control of exposure and gain : %s",
                         mCamAutoExposure ? "Enable" : "Disable");
            break;

        case 4:
            mCamMatResizeFactor = config.mat_resize_factor;
            NODELET_INFO("Reconfigure mat_resize_factor: %g", mCamMatResizeFactor);
            mCamDataMutex.lock();
            mMatWidth = static_cast<size_t>(mCamWidth * mCamMatResizeFactor);
            mMatHeight = static_cast<size_t>(mCamHeight * mCamMatResizeFactor);
            NODELET_DEBUG_STREAM("Data Mat size : " << mMatWidth << "x" << mMatHeight);

            // Update Camera Info
            fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId,
                        mRightCamOptFrameId);
            fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId,
                        mRightCamOptFrameId, true);
            mRgbCamInfoMsg = mDepthCamInfoMsg = mLeftCamInfoMsg; // the reference camera is
            // the Left one (next to
            // the ZED logo)
            mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
            mCamDataMutex.unlock();
            break;

        case 5:
            mCamMaxDepth = config.max_depth;
            NODELET_INFO("Reconfigure max depth : %g", mCamMaxDepth);
            break;
        }
    }

    void ZEDWrapperNodelet::pathPubCallback(const ros::TimerEvent& e) {
        uint32_t mapPathSub = mPubMapPath.getNumSubscribers();
        uint32_t odomPathSub = mPubOdomPath.getNumSubscribers();

        tf2::Transform base_to_odom;
        base_to_odom.setIdentity();
        tf2::Transform base_to_map;
        base_to_map.setIdentity();

        // Look up the transformation from base frame to odom
        if (mPublishTf) {
            try {
                // Save the transformation from base to frame
                geometry_msgs::TransformStamped b2o =
                    mTfBuffer->lookupTransform(mOdometryFrameId, mBaseFrameId, ros::Time(0));
                // Get the TF2 transformation
                tf2::fromMsg(b2o.transform, base_to_odom);
            } catch (tf2::TransformException& ex) {
                NODELET_WARN_THROTTLE(
                    10.0, "The tf from '%s' to '%s' does not seem to be available, "
                    "will assume it as identity!",
                    mBaseFrameId.c_str(), mOdometryFrameId.c_str());
                NODELET_DEBUG("Transform error: %s", ex.what());
            }
        }

        if (mPublishMapTf) {
            // Look up the transformation from base frame to map
            try {
                // Save the transformation from base to frame
                geometry_msgs::TransformStamped b2m =
                    mTfBuffer->lookupTransform(mMapFrameId, mBaseFrameId, ros::Time(0));
                // Get the TF2 transformation
                tf2::fromMsg(b2m.transform, base_to_map);
            } catch (tf2::TransformException& ex) {
                NODELET_WARN_THROTTLE(
                    10.0, "The tf from '%s' to '%s' does not seem to be available, "
                    "will assume it as identity!",
                    mBaseFrameId.c_str(), mOdometryFrameId.c_str());
                NODELET_DEBUG("Transform error: %s", ex.what());
            }
        }

        geometry_msgs::PoseStamped odomPose;
        geometry_msgs::PoseStamped mapPose;

        odomPose.header.stamp = mFrameTimestamp;
        odomPose.header.frame_id = mPublishMapTf ? mMapFrameId : mOdometryFrameId; // frame
        // conversion from Tranform to message
        geometry_msgs::Transform base2odom = tf2::toMsg(base_to_odom);
        // Add all value in Pose message
        odomPose.pose.position.x = base2odom.translation.x;
        odomPose.pose.position.y = base2odom.translation.y;
        odomPose.pose.position.z = base2odom.translation.z;
        odomPose.pose.orientation.x = base2odom.rotation.x;
        odomPose.pose.orientation.y = base2odom.rotation.y;
        odomPose.pose.orientation.z = base2odom.rotation.z;
        odomPose.pose.orientation.w = base2odom.rotation.w;

        if (mPublishMapTf) {
            mapPose.header.stamp = mFrameTimestamp;
            mapPose.header.frame_id = mMapFrameId; // map_frame
            // conversion from Tranform to message
            geometry_msgs::Transform base2map = tf2::toMsg(base_to_map);
            // Add all value in Pose message
            mapPose.pose.position.x = base2map.translation.x;
            mapPose.pose.position.y = base2map.translation.y;
            mapPose.pose.position.z = base2map.translation.z;
            mapPose.pose.orientation.x = base2map.rotation.x;
            mapPose.pose.orientation.y = base2map.rotation.y;
            mapPose.pose.orientation.z = base2map.rotation.z;
            mapPose.pose.orientation.w = base2map.rotation.w;
        }

        // Circular vector
        if (mPathMaxCount != -1) {
            if (mOdomPath.size() == mPathMaxCount) {
                NODELET_DEBUG("Path vectors full: rotating ");
                std::rotate(mOdomPath.begin(), mOdomPath.begin() + 1, mOdomPath.end());
                std::rotate(mMapPath.begin(), mMapPath.begin() + 1, mMapPath.end());

                mMapPath[mPathMaxCount - 1] = mapPose;
                mOdomPath[mPathMaxCount - 1] = odomPose;
            } else {
                //NODELET_DEBUG_STREAM("Path vectors adding last available poses");
                mMapPath.push_back(mapPose);
                mOdomPath.push_back(odomPose);
            }
        } else {
            //NODELET_DEBUG_STREAM("No limit path vectors, adding last available poses");
            mMapPath.push_back(mapPose);
            mOdomPath.push_back(odomPose);
        }

        if (mapPathSub > 0 &&  mPublishMapTf) {
            nav_msgs::Path mapPath;
            mapPath.header.frame_id = mMapFrameId;
            mapPath.header.stamp = mFrameTimestamp;
            mapPath.poses = mMapPath;

            mPubMapPath.publish(mapPath);
        }

        if (odomPathSub > 0) {
            nav_msgs::Path odomPath;
            odomPath.header.frame_id = mPublishMapTf ? mMapFrameId : mOdometryFrameId;
            odomPath.header.stamp = mFrameTimestamp;
            odomPath.poses = mOdomPath;

            mPubOdomPath.publish(odomPath);
        }

    }

    void ZEDWrapperNodelet::imuPubCallback(const ros::TimerEvent& e) {

        std::lock_guard<std::mutex> lock(mCloseZedMutex);

        if (!mZed.isOpened()) {
            return;
        }

        uint32_t imu_SubNumber = mPubImu.getNumSubscribers();
        uint32_t imu_RawSubNumber = mPubImuRaw.getNumSubscribers();

        if (imu_SubNumber < 1 && imu_RawSubNumber < 1) {
            return;
        }

        ros::Time t;

        if (mSvoMode) {
            t = ros::Time::now();
        } else {
            if (mImuTimestampSync && mGrabActive) {
                t = mFrameTimestamp;
            } else {
                t = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));
            }
        }

        sl::IMUData imu_data;

        if (mImuTimestampSync && mGrabActive) {
            mZed.getIMUData(imu_data, sl::TIME_REFERENCE_IMAGE);
        } else {
            mZed.getIMUData(imu_data, sl::TIME_REFERENCE_CURRENT);
        }

        if (imu_SubNumber > 0 || imu_RawSubNumber > 0) {
            // Publish freq calculation
            static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

            double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
            last_time = now;

            mImuPeriodMean_usec->addValue(elapsed_usec);

            mImuPublishing = true;
        } else {
            mImuPublishing = false;
        }

        if (imu_SubNumber > 0) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = t;
            imu_msg.header.frame_id = mImuFrameId;

            imu_msg.orientation.x = mSignX * imu_data.getOrientation()[mIdxX];
            imu_msg.orientation.y = mSignY * imu_data.getOrientation()[mIdxY];
            imu_msg.orientation.z = mSignZ * imu_data.getOrientation()[mIdxZ];
            imu_msg.orientation.w = imu_data.getOrientation()[3];

            imu_msg.angular_velocity.x = mSignX * imu_data.angular_velocity[mIdxX] * DEG2RAD;
            imu_msg.angular_velocity.y = mSignY * imu_data.angular_velocity[mIdxY] * DEG2RAD;
            imu_msg.angular_velocity.z = mSignZ * imu_data.angular_velocity[mIdxZ] * DEG2RAD;

            imu_msg.linear_acceleration.x = mSignX * imu_data.linear_acceleration[mIdxX];
            imu_msg.linear_acceleration.y = mSignY * imu_data.linear_acceleration[mIdxY];
            imu_msg.linear_acceleration.z = mSignZ * imu_data.linear_acceleration[mIdxZ];

            for (int i = 0; i < 3; ++i) {

                int r = 0;

                if (i == 0) {
                    r = mIdxX;
                } else if (i == 1) {
                    r = mIdxY;
                } else {
                    r = mIdxZ;
                }

                imu_msg.orientation_covariance[i * 3 + 0] =
                    imu_data.orientation_covariance.r[r * 3 + mIdxX] * DEG2RAD * DEG2RAD;
                imu_msg.orientation_covariance[i * 3 + 1] =
                    imu_data.orientation_covariance.r[r * 3 + mIdxY] * DEG2RAD * DEG2RAD;
                imu_msg.orientation_covariance[i * 3 + 2] =
                    imu_data.orientation_covariance.r[r * 3 + mIdxZ] * DEG2RAD * DEG2RAD;

                imu_msg.linear_acceleration_covariance[i * 3 + 0] =
                    imu_data.linear_acceleration_convariance.r[r * 3 + mIdxX];
                imu_msg.linear_acceleration_covariance[i * 3 + 1] =
                    imu_data.linear_acceleration_convariance.r[r * 3 + mIdxY];
                imu_msg.linear_acceleration_covariance[i * 3 + 2] =
                    imu_data.linear_acceleration_convariance.r[r * 3 + mIdxZ];

                imu_msg.angular_velocity_covariance[i * 3 + 0] =
                    imu_data.angular_velocity_convariance.r[r * 3 + mIdxX] * DEG2RAD * DEG2RAD;
                imu_msg.angular_velocity_covariance[i * 3 + 1] =
                    imu_data.angular_velocity_convariance.r[r * 3 + mIdxY] * DEG2RAD * DEG2RAD;
                imu_msg.angular_velocity_covariance[i * 3 + 2] =
                    imu_data.angular_velocity_convariance.r[r * 3 + mIdxZ] * DEG2RAD * DEG2RAD;
            }

            mPubImu.publish(imu_msg);
        }

        if (imu_RawSubNumber > 0) {
            sensor_msgs::Imu imu_raw_msg;
            imu_raw_msg.header.stamp = mFrameTimestamp; // t;
            imu_raw_msg.header.frame_id = mImuFrameId;
            imu_raw_msg.angular_velocity.x = mSignX * imu_data.angular_velocity[mIdxX] * DEG2RAD;
            imu_raw_msg.angular_velocity.y = mSignY * imu_data.angular_velocity[mIdxY] * DEG2RAD;
            imu_raw_msg.angular_velocity.z = mSignZ * imu_data.angular_velocity[mIdxZ] * DEG2RAD;
            imu_raw_msg.linear_acceleration.x =
                mSignX * imu_data.linear_acceleration[mIdxX];
            imu_raw_msg.linear_acceleration.y =
                mSignY * imu_data.linear_acceleration[mIdxY];
            imu_raw_msg.linear_acceleration.z =
                mSignZ * imu_data.linear_acceleration[mIdxZ];

            for (int i = 0; i < 3; ++i) {

                int r = 0;

                if (i == 0) {
                    r = mIdxX;
                } else if (i == 1) {
                    r = mIdxY;
                } else {
                    r = mIdxZ;
                }

                imu_raw_msg.linear_acceleration_covariance[i * 3 + 0] =
                    imu_data.linear_acceleration_convariance.r[r * 3 + mIdxX];
                imu_raw_msg.linear_acceleration_covariance[i * 3 + 1] =
                    imu_data.linear_acceleration_convariance.r[r * 3 + mIdxY];
                imu_raw_msg.linear_acceleration_covariance[i * 3 + 2] =
                    imu_data.linear_acceleration_convariance.r[r * 3 + mIdxZ];
                imu_raw_msg.angular_velocity_covariance[i * 3 + 0] =
                    imu_data.angular_velocity_convariance.r[r * 3 + mIdxX] * DEG2RAD * DEG2RAD;
                imu_raw_msg.angular_velocity_covariance[i * 3 + 1] =
                    imu_data.angular_velocity_convariance.r[r * 3 + mIdxY] * DEG2RAD * DEG2RAD;
                imu_raw_msg.angular_velocity_covariance[i * 3 + 2] =
                    imu_data.angular_velocity_convariance.r[r * 3 + mIdxZ] * DEG2RAD * DEG2RAD;
            }

            imu_raw_msg.orientation_covariance[0] =
                -1; // Orientation data is not available in "data_raw" -> See ROS REP145
            // http://www.ros.org/reps/rep-0145.html#topics
            mPubImuRaw.publish(imu_raw_msg);
        }

        // Publish IMU tf only if enabled
        if (mPublishTf) {
            // Camera to pose transform from TF buffer
            tf2::Transform cam_to_pose;

            std::string poseFrame;

            // Look up the transformation from base frame to map link
            try {
                poseFrame = mPublishMapTf ? mMapFrameId : mOdometryFrameId;

                // Save the transformation from base to frame
                geometry_msgs::TransformStamped c2p =
                    mTfBuffer->lookupTransform(poseFrame, mCameraFrameId, ros::Time(0));
                // Get the TF2 transformation
                tf2::fromMsg(c2p.transform, cam_to_pose);
            } catch (tf2::TransformException& ex) {
                NODELET_WARN_THROTTLE(
                    10.0, "The tf from '%s' to '%s' does not seem to be available. "
                    "IMU TF not published!",
                    mCameraFrameId.c_str(), mMapFrameId.c_str());
                NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
                return;
            }

            // IMU Quaternion in Map frame
            tf2::Quaternion imu_q;
            imu_q.setX(mSignX * imu_data.getOrientation()[mIdxX]);
            imu_q.setY(mSignY * imu_data.getOrientation()[mIdxY]);
            imu_q.setZ(mSignZ * imu_data.getOrientation()[mIdxZ]);
            imu_q.setW(imu_data.getOrientation()[3]);
            // Pose Quaternion from ZED Camera
            tf2::Quaternion map_q = cam_to_pose.getRotation();
            // Difference between IMU and ZED Quaternion
            tf2::Quaternion delta_q = imu_q * map_q.inverse();
            tf2::Transform imu_pose;
            imu_pose.setIdentity();
            imu_pose.setRotation(delta_q);
            // Note, the frame is published, but its values will only change if someone
            // has subscribed to IMU
            publishImuFrame(imu_pose, mFrameTimestamp); // publish the imu Frame
        }
    }

    void ZEDWrapperNodelet::device_poll_thread_func() {
        ros::Rate loop_rate(mCamFrameRate);

        mElabPeriodMean_sec.reset(new sl_tools::CSmartMean(mCamFrameRate));
        mGrabPeriodMean_usec.reset(new sl_tools::CSmartMean(mCamFrameRate));
        mPcPeriodMean_usec.reset(new sl_tools::CSmartMean(mCamFrameRate));

        // Timestamp initialization
        if (mSvoMode) {
            mFrameTimestamp = ros::Time::now();
        } else {
            mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));
        }

        mPrevFrameTimestamp = mFrameTimestamp;

        mTrackingActivated = false;
        // Get the parameters of the ZED images
        mCamWidth = mZed.getResolution().width;
        mCamHeight = mZed.getResolution().height;
        NODELET_DEBUG_STREAM("Camera Frame size : " << mCamWidth << "x" << mCamHeight);
        mMatWidth = static_cast<int>(mCamWidth * mCamMatResizeFactor);
        mMatHeight = static_cast<int>(mCamHeight * mCamMatResizeFactor);
        NODELET_DEBUG_STREAM("Data Mat size : " << mMatWidth << "x" << mMatHeight);

        // Create and fill the camera information messages
        fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId,
                    mRightCamOptFrameId);
        fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId,
                    mRightCamOptFrameId, true);
        mRgbCamInfoMsg = mDepthCamInfoMsg = mLeftCamInfoMsg; // the reference camera is
        // the Left one (next to the
        // ZED logo)
        mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
        sl::RuntimeParameters runParams;
        runParams.sensing_mode = static_cast<sl::SENSING_MODE>(mCamSensingMode);
        sl::Mat leftZEDMat, rightZEDMat, depthZEDMat, disparityZEDMat, confImgZEDMat, confMapZEDMat;

        // Main loop
        while (mNhNs.ok()) {
            // Check for subscribers
            uint32_t rgbSubnumber = mPubRgb.getNumSubscribers();
            uint32_t rgbRawSubnumber = mPubRawRgb.getNumSubscribers();
            uint32_t leftSubnumber = mPubLeft.getNumSubscribers();
            uint32_t leftRawSubnumber = mPubRawLeft.getNumSubscribers();
            uint32_t rightSubnumber = mPubRight.getNumSubscribers();
            uint32_t rightRawSubnumber = mPubRawRight.getNumSubscribers();
            uint32_t depthSubnumber = mPubDepth.getNumSubscribers();
            uint32_t disparitySubnumber = mPubDisparity.getNumSubscribers();
            uint32_t cloudSubnumber = mPubCloud.getNumSubscribers();
            uint32_t poseSubnumber = mPubPose.getNumSubscribers();
            uint32_t poseCovSubnumber = mPubPoseCov.getNumSubscribers();
            uint32_t odomSubnumber = mPubOdom.getNumSubscribers();
            uint32_t confImgSubnumber = mPubConfImg.getNumSubscribers();
            uint32_t confMapSubnumber = mPubConfMap.getNumSubscribers();
            uint32_t imuSubnumber = mPubImu.getNumSubscribers();
            uint32_t imuRawsubnumber = mPubImuRaw.getNumSubscribers();
            uint32_t pathSubNumber = mPubMapPath.getNumSubscribers() + mPubOdomPath.getNumSubscribers();
            mGrabActive = ((rgbSubnumber + rgbRawSubnumber + leftSubnumber +
                            leftRawSubnumber + rightSubnumber + rightRawSubnumber +
                            depthSubnumber + disparitySubnumber + cloudSubnumber +
                            poseSubnumber + poseCovSubnumber + odomSubnumber + confImgSubnumber +
                            confMapSubnumber /*+ imuSubnumber + imuRawsubnumber*/ + pathSubNumber) > 0);

            runParams.enable_point_cloud = false;

            if (cloudSubnumber > 0) {
                runParams.enable_point_cloud = true;
            }

            // Run the loop only if there is some subscribers
            if (mGrabActive) {
                bool startTracking = (mDepthStabilization || poseSubnumber > 0 || poseCovSubnumber > 0 ||
                                      odomSubnumber > 0 || cloudSubnumber > 0 || depthSubnumber > 0 || pathSubNumber > 0);

                // Detect if one of the subscriber need to have the depth information
                mComputeDepth = mCamQuality != sl::DEPTH_MODE_NONE && ((depthSubnumber + disparitySubnumber + cloudSubnumber +
                                poseSubnumber + poseCovSubnumber + odomSubnumber + confImgSubnumber +
                                confMapSubnumber) > 0);

                if ((startTracking) && !mTrackingActivated && mComputeDepth) { // Start the tracking
                    start_tracking();
                } else if (!mDepthStabilization && poseSubnumber == 0 && poseCovSubnumber == 0 &&
                           odomSubnumber == 0 &&
                           mTrackingActivated) { // Stop the tracking
                    mZed.disableTracking();
                    mTrackingActivated = false;
                }

                if (mComputeDepth) {
                    int actual_confidence = mZed.getConfidenceThreshold();

                    if (actual_confidence != mCamConfidence) {
                        mZed.setConfidenceThreshold(mCamConfidence);
                    }

                    double actual_max_depth = static_cast<double>(mZed.getDepthMaxRangeValue());

                    if (actual_max_depth != mCamMaxDepth) {
                        mZed.setDepthMaxRangeValue(static_cast<double>(mCamMaxDepth));
                    }

                    runParams.enable_depth = true; // Ask to compute the depth
                } else {
                    runParams.enable_depth = false;
                }

                mGrabStatus = mZed.grab(runParams); // Ask to not compute the depth

                // cout << toString(grab_status) << endl;
                if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
                    // Detect if a error occurred (for example:
                    // the zed have been disconnected) and
                    // re-initialize the ZED
                    if (mGrabStatus != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                        NODELET_INFO_STREAM_ONCE(toString(mGrabStatus));
                    }

                    //                    if ( mSvoMode && mPubClock.getNumSubscribers() > 0) {
                    //                        rosgraph_msgs::Clock clkMsg;
                    //                        clkMsg.clock = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_IMAGE));

                    //                        mPubClock.publish(clkMsg);
                    //                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(2));

                    if ((ros::Time::now() - mPrevFrameTimestamp).toSec() > 5 && !mSvoMode) {
                        mCloseZedMutex.lock();
                        mZed.close();
                        mCloseZedMutex.unlock();

                        mConnStatus = sl::ERROR_CODE_CAMERA_NOT_DETECTED;

                        while (mConnStatus != sl::SUCCESS) {
                            if (!mNhNs.ok()) {
                                mStopNode = true;

                                std::lock_guard<std::mutex> lock(mCloseZedMutex);
                                NODELET_DEBUG("Closing ZED");
                                mZed.close();

                                NODELET_DEBUG("ZED pool thread finished");
                                return;
                            }

                            int id = sl_tools::checkCameraReady(mZedSerialNumber);

                            if (id >= 0) {
                                mZedParams.camera_linux_id = id;
                                mConnStatus = mZed.open(mZedParams); // Try to initialize the ZED
                                NODELET_INFO_STREAM(toString(mConnStatus));
                            } else {
                                NODELET_INFO_STREAM("Waiting for the ZED (S/N " << mZedSerialNumber << ") to be re-connected");
                            }

                            mDiagUpdater.force_update();
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        }

                        mTrackingActivated = false;

                        startTracking = mDepthStabilization || poseSubnumber > 0 || poseCovSubnumber > 0 ||
                                        odomSubnumber > 0;

                        if (startTracking) {  // Start the tracking
                            start_tracking();
                        }
                    }

                    mDiagUpdater.update();

                    continue;
                }

                mPrevFrameTimestamp = mFrameTimestamp;

                // Publish freq calculation
                static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
                std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

                double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
                last_time = now;

                mGrabPeriodMean_usec->addValue(elapsed_usec);

                // Timestamp
                if (mSvoMode) {
                    mFrameTimestamp = ros::Time::now();
                } else {
                    mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_IMAGE));
                }

                if (mCamAutoExposure) {
                    // getCameraSettings() can't check status of auto exposure
                    // triggerAutoExposure is used to execute setCameraSettings() only once
                    if (mTriggerAutoExposure) {
                        mZed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, 0, true);
                        mTriggerAutoExposure = false;
                    }
                } else {
                    int actual_exposure =
                        mZed.getCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE);

                    if (actual_exposure != mCamExposure) {
                        mZed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, mCamExposure);
                    }

                    int actual_gain = mZed.getCameraSettings(sl::CAMERA_SETTINGS_GAIN);

                    if (actual_gain != mCamGain) {
                        mZed.setCameraSettings(sl::CAMERA_SETTINGS_GAIN, mCamGain);
                    }
                }

                mCamDataMutex.lock();

                // Publish the left == rgb image if someone has subscribed to
                if (leftSubnumber > 0 || rgbSubnumber > 0) {
                    // Retrieve RGBA Left image
                    mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT, sl::MEM_CPU, mMatWidth, mMatHeight);

                    if (leftSubnumber > 0) {
                        publishCamInfo(mLeftCamInfoMsg, mPubLeftCamInfo, mFrameTimestamp);
                        publishImage(leftZEDMat, mPubLeft, mLeftCamOptFrameId, mFrameTimestamp);
                    }

                    if (rgbSubnumber > 0) {
                        publishCamInfo(mRgbCamInfoMsg, mPubRgbCamInfo, mFrameTimestamp);
                        publishImage(leftZEDMat, mPubRgb, mDepthOptFrameId, mFrameTimestamp); // rgb is the left image
                    }
                }

                // Publish the left_raw == rgb_raw image if someone has subscribed to
                if (leftRawSubnumber > 0 || rgbRawSubnumber > 0) {
                    // Retrieve RGBA Left image
                    mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);

                    if (leftRawSubnumber > 0) {
                        publishCamInfo(mLeftCamInfoRawMsg, mPubLeftCamInfoRaw, mFrameTimestamp);
                        publishImage(leftZEDMat, mPubRawLeft, mLeftCamOptFrameId, mFrameTimestamp);
                    }

                    if (rgbRawSubnumber > 0) {
                        publishCamInfo(mRgbCamInfoRawMsg, mPubRgbCamInfoRaw, mFrameTimestamp);
                        publishImage(leftZEDMat, mPubRawRgb, mDepthOptFrameId, mFrameTimestamp);
                    }
                }

                // Publish the right image if someone has subscribed to
                if (rightSubnumber > 0) {
                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT, sl::MEM_CPU, mMatWidth, mMatHeight);

                    publishCamInfo(mRightCamInfoMsg, mPubRightCamInfo, mFrameTimestamp);
                    publishImage(rightZEDMat, mPubRight, mRightCamOptFrameId, mFrameTimestamp);
                }

                // Publish the right image if someone has subscribed to
                if (rightRawSubnumber > 0) {
                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);

                    publishCamInfo(mRightCamInfoRawMsg, mPubRightCamInfoRaw, mFrameTimestamp);
                    publishImage(rightZEDMat, mPubRawRight, mRightCamOptFrameId, mFrameTimestamp);
                }

                // Publish the depth image if someone has subscribed to
                if (depthSubnumber > 0 || disparitySubnumber > 0) {
                    mZed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH, sl::MEM_CPU, mMatWidth, mMatHeight);
                    publishCamInfo(mDepthCamInfoMsg, mPubDepthCamInfo, mFrameTimestamp);
                    publishDepth(depthZEDMat, mFrameTimestamp); // in meters
                }

                // Publish the disparity image if someone has subscribed to
                if (disparitySubnumber > 0) {
                    mZed.retrieveMeasure(disparityZEDMat, sl::MEASURE_DISPARITY, sl::MEM_CPU, mMatWidth, mMatHeight);

                    publishDisparity(disparityZEDMat, mFrameTimestamp);
                }

                // Publish the confidence image if someone has subscribed to
                if (confImgSubnumber > 0) {
                    mZed.retrieveImage(confImgZEDMat, sl::VIEW_CONFIDENCE, sl::MEM_CPU, mMatWidth, mMatHeight);
                    publishImage(confImgZEDMat, mPubConfImg, mConfidenceOptFrameId, mFrameTimestamp);
                }

                // Publish the confidence map if someone has subscribed to
                if (confMapSubnumber > 0) {
                    mZed.retrieveMeasure(confMapZEDMat, sl::MEASURE_CONFIDENCE, sl::MEM_CPU, mMatWidth, mMatHeight);


                    mPubConfMap.publish(sl_tools::imageToROSmsg(confMapZEDMat, mConfidenceOptFrameId, mFrameTimestamp));
                }

                // Publish the point cloud if someone has subscribed to
                if (cloudSubnumber > 0) {
                    // Run the point cloud conversion asynchronously to avoid slowing down
                    // all the program
                    // Retrieve raw pointCloud data if latest Pointcloud is ready
                    std::unique_lock<std::mutex> lock(mPcMutex, std::defer_lock);

                    if (lock.try_lock()) {
                        mZed.retrieveMeasure(mCloud, sl::MEASURE_XYZBGRA, sl::MEM_CPU, mMatWidth, mMatHeight);

                        mPointCloudFrameId = mDepthFrameId;
                        mPointCloudTime = mFrameTimestamp;

                        // Signal Pointcloud thread that a new pointcloud is ready
                        mPcDataReadyCondVar.notify_one();
                        mPcDataReady = true;
                        mPcPublishing = true;
                    }
                } else {
                    mPcPublishing = false;
                }

                mCamDataMutex.unlock();

                // Publish the odometry if someone has subscribed to
                if (poseSubnumber > 0 || poseCovSubnumber > 0 || odomSubnumber > 0 || cloudSubnumber > 0 ||
                    depthSubnumber > 0 || imuSubnumber > 0 || imuRawsubnumber > 0 || pathSubNumber > 0) {
                    if (!mInitOdomWithPose) {
                        sl::Pose deltaOdom;
                        mTrackingStatus = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME_CAMERA);

                        sl::Translation translation = deltaOdom.getTranslation();
                        sl::Orientation quat = deltaOdom.getOrientation();

                        NODELET_DEBUG("delta ODOM [%s] - %.2f,%.2f,%.2f %.2f,%.2f,%.2f,%.2f",
                                      sl::toString(mTrackingStatus).c_str(),
                                      translation(mIdxX), translation(mIdxY), translation(mIdxZ),
                                      quat(mIdxX), quat(mIdxY), quat(mIdxZ), quat(3));

                        NODELET_DEBUG_STREAM("ODOM -> Tracking Status: " << sl::toString(mTrackingStatus));

                        if (mTrackingStatus == sl::TRACKING_STATE_OK || mTrackingStatus == sl::TRACKING_STATE_SEARCHING ||
                            mTrackingStatus == sl::TRACKING_STATE_FPS_TOO_LOW) {
                            // Transform ZED delta odom pose in TF2 Transformation
                            geometry_msgs::Transform deltaTransf;
                            deltaTransf.translation.x = mSignX * translation(mIdxX);
                            deltaTransf.translation.y = mSignY * translation(mIdxY);
                            deltaTransf.translation.z = mSignZ * translation(mIdxZ);
                            deltaTransf.rotation.x = mSignX * quat(mIdxX);
                            deltaTransf.rotation.y = mSignY * quat(mIdxY);
                            deltaTransf.rotation.z = mSignZ * quat(mIdxZ);
                            deltaTransf.rotation.w = quat(3);
                            tf2::Transform deltaOdomTf;
                            tf2::fromMsg(deltaTransf, deltaOdomTf);
                            // delta odom from sensor to base frame
                            tf2::Transform deltaOdomTf_base =
                                mSensor2BaseTransf.inverse() * deltaOdomTf * mSensor2BaseTransf;

                            // Propagate Odom transform in time
                            mOdom2BaseTransf = mOdom2BaseTransf * deltaOdomTf_base;

#if 0 //#ifndef NDEBUG // Enable for TF checking
                            double roll, pitch, yaw;
                            tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                            NODELET_DEBUG("+++ Odometry [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                          mOdometryFrameId.c_str(), mBaseFrameId.c_str(),
                                          mOdom2BaseTransf.getOrigin().x(), mOdom2BaseTransf.getOrigin().y(), mOdom2BaseTransf.getOrigin().z(),
                                          roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

                            // Publish odometry message
                            publishOdom(mOdom2BaseTransf, deltaOdom, mFrameTimestamp);
                            mTrackingReady = true;
                        }
                    } else if (mFloorAlignment) {
                        NODELET_WARN_THROTTLE(5.0, "Odometry will be published as soon as the floor as been detected for the first time");
                    }

                }

                // Publish the zed camera pose if someone has subscribed to
                if (poseSubnumber > 0 || odomSubnumber > 0 || poseCovSubnumber > 0 || cloudSubnumber > 0 ||
                    depthSubnumber > 0 || imuSubnumber > 0 || imuRawsubnumber > 0 || pathSubNumber > 0) {

                    static sl::TRACKING_STATE oldStatus;
                    mTrackingStatus = mZed.getPosition(mLastZedPose, sl::REFERENCE_FRAME_WORLD);

                    sl::Translation translation = mLastZedPose.getTranslation();
                    sl::Orientation quat = mLastZedPose.getOrientation();

#if 0 //#ifndef NDEBUG // Enable for TF checking
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(tf2::Quaternion(quat.ox, quat.oy, quat.oz, quat.ow)).getRPY(roll, pitch, yaw);

                    NODELET_DEBUG("Sensor POSE [%s -> %s] - {%.2f,%.2f,%.2f} {%.2f,%.2f,%.2f}",
                                  mLeftCamFrameId.c_str(), mMapFrameId.c_str(),
                                  translation.x, translation.y, translation.z,
                                  roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

                    NODELET_DEBUG_STREAM("MAP -> Tracking Status: " << sl::toString(mTrackingStatus));

                    if (mTrackingStatus == sl::TRACKING_STATE_OK ||
                        mTrackingStatus == sl::TRACKING_STATE_SEARCHING /*|| status == sl::TRACKING_STATE_FPS_TOO_LOW*/) {
                        // Transform ZED pose in TF2 Transformation
                        geometry_msgs::Transform map2sensTransf;

                        map2sensTransf.translation.x = mSignX * translation(mIdxX);
                        map2sensTransf.translation.y = mSignY * translation(mIdxY);
                        map2sensTransf.translation.z = mSignZ * translation(mIdxZ);
                        map2sensTransf.rotation.x = mSignX * quat(mIdxX);
                        map2sensTransf.rotation.y = mSignY * quat(mIdxY);
                        map2sensTransf.rotation.z = mSignZ * quat(mIdxZ);
                        map2sensTransf.rotation.w = quat(3);
                        tf2::Transform map_to_sens_transf;
                        tf2::fromMsg(map2sensTransf, map_to_sens_transf);

                        mMap2BaseTransf = map_to_sens_transf * mSensor2BaseTransf; // Base position in map frame

#if 0 //#ifndef NDEBUG // Enable for TF checking
                        double roll, pitch, yaw;
                        tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                        NODELET_DEBUG("*** Base POSE [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                      mMapFrameId.c_str(), mBaseFrameId.c_str(),
                                      mMap2BaseTransf.getOrigin().x(), mMap2BaseTransf.getOrigin().y(), mMap2BaseTransf.getOrigin().z(),
                                      roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

                        bool initOdom = false;

                        if (!(mFloorAlignment)) {
                            initOdom = mInitOdomWithPose;
                        } else {
                            initOdom = (mTrackingStatus == sl::TRACKING_STATE_OK) & mInitOdomWithPose;
                        }

                        if (initOdom || mResetOdom) {

                            ROS_INFO("Odometry aligned to last tracking pose");

                            // Propagate Odom transform in time
                            mOdom2BaseTransf = mMap2BaseTransf;
                            mMap2BaseTransf.setIdentity();

                            if (odomSubnumber > 0) {
                                // Publish odometry message
                                publishOdom(mOdom2BaseTransf, mLastZedPose, mFrameTimestamp);
                            }

                            mInitOdomWithPose = false;
                            mResetOdom = false;
                        } else {
                            // Transformation from map to odometry frame
                            //mMap2OdomTransf = mOdom2BaseTransf.inverse() * mMap2BaseTransf;
                            mMap2OdomTransf = mMap2BaseTransf * mOdom2BaseTransf.inverse();

#if 0 //#ifndef NDEBUG // Enable for TF checking
                            double roll, pitch, yaw;
                            tf2::Matrix3x3(mMap2OdomTransf.getRotation()).getRPY(roll, pitch, yaw);

                            NODELET_DEBUG("+++ Diff [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                          mMapFrameId.c_str(), mOdometryFrameId.c_str(),
                                          mMap2OdomTransf.getOrigin().x(), mMap2OdomTransf.getOrigin().y(), mMap2OdomTransf.getOrigin().z(),
                                          roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif
                        }

                        // Publish Pose message
                        publishPose(mFrameTimestamp);
                        mTrackingReady = true;
                    }

                    oldStatus = mTrackingStatus;
                }



                // Publish pose tf only if enabled
                if (mPublishTf) {
                    // Note, the frame is published, but its values will only change if
                    // someone has subscribed to odom
                    publishOdomFrame(mOdom2BaseTransf, mFrameTimestamp); // publish the base Frame in odometry frame

                    if (mPublishMapTf) {
                        // Note, the frame is published, but its values will only change if
                        // someone has subscribed to map
                        publishPoseFrame(mMap2OdomTransf, mFrameTimestamp); // publish the odometry Frame in map frame
                    }
                }

#if 0 //#ifndef NDEBUG // Enable for TF checking
                // Double check: map_to_pose must be equal to mMap2BaseTransf

                tf2::Transform map_to_base;

                try {
                    // Save the transformation from base to frame
                    geometry_msgs::TransformStamped b2m =
                        mTfBuffer->lookupTransform(mMapFrameId, mBaseFrameId, ros::Time(0));
                    // Get the TF2 transformation
                    tf2::fromMsg(b2m.transform, map_to_base);
                } catch (tf2::TransformException& ex) {
                    NODELET_DEBUG("The tf from '%s' to '%s' does not seem to be available, "
                                  "will assume it as identity!",
                                  mMapFrameId.c_str(), mBaseFrameId.c_str());
                    NODELET_DEBUG("Transform error: %s", ex.what());
                }

                double roll, pitch, yaw;
                tf2::Matrix3x3(map_to_base.getRotation()).getRPY(roll, pitch, yaw);

                NODELET_DEBUG("*** Check [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                              mMapFrameId.c_str(), mBaseFrameId.c_str(),
                              map_to_base.getOrigin().x(), map_to_base.getOrigin().y(), map_to_base.getOrigin().z(),
                              roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

                NODELET_DEBUG("*******************************");
#endif

                double mean = mElabPeriodMean_sec->addValue(loop_rate.cycleTime().toSec()) ;

                if (!loop_rate.sleep()) {
                    if (mean > loop_rate.expectedCycleTime().toSec()) {
                        NODELET_DEBUG_THROTTLE(
                            1.0,
                            "Working thread is not synchronized with the Camera frame rate");
                        NODELET_DEBUG_STREAM_THROTTLE(
                            1.0, "Expected cycle time: " << loop_rate.expectedCycleTime()
                            << " - Real cycle time: "
                            << mean);
                        NODELET_WARN_THROTTLE(10.0, "Elaboration takes longer than requested "
                                              "by the FPS rate. Please consider to "
                                              "lower the 'frame_rate' setting.");
                    }
                }
            } else {
                NODELET_DEBUG_THROTTLE(5.0, "No topics subscribed by users");

                // Publish odometry tf only if enabled
                if (mPublishTf) {
                    ros::Time t;

                    if (mSvoMode) {
                        t = ros::Time::now();
                    } else {
                        t = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));
                    }

                    publishOdomFrame(mOdom2BaseTransf, mFrameTimestamp); // publish the base Frame in odometry frame

                    if (mPublishMapTf) {
                        publishPoseFrame(mMap2OdomTransf, mFrameTimestamp); // publish the odometry Frame in map frame
                    }
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(10)); // No subscribers, we just wait
                loop_rate.reset();
            }

            mDiagUpdater.update();
        } // while loop

        mStopNode = true; // Stops other threads

        std::lock_guard<std::mutex> lock(mCloseZedMutex);
        NODELET_DEBUG("Closing ZED");
        mZed.close();

        NODELET_DEBUG("ZED pool thread finished");
    }

    void ZEDWrapperNodelet::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) {

        if (mConnStatus == sl::SUCCESS) {
            if (mGrabActive) {
                if (mGrabStatus == sl::SUCCESS || mGrabStatus == sl::ERROR_CODE_NOT_A_NEW_FRAME) {

                    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera grabbing");

                    double freq = 1000000. / mGrabPeriodMean_usec->getMean();
                    double freq_perc = 100.*freq / mCamFrameRate;
                    stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

                    stat.addf("Processing Time", "Mean time: %.3f sec (Exp. %.3f sec)", mElabPeriodMean_sec->getMean(), 1. / mCamFrameRate);

                    if (mComputeDepth) {
                        stat.add("Depth status", "ACTIVE");

                        if (mPcPublishing) {
                            double freq = 1000000. / mPcPeriodMean_usec->getMean();
                            double freq_perc = 100.*freq / mCamFrameRate;
                            stat.addf("Point Cloud", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
                        } else {
                            stat.add("Point Cloud", "Topic not subscribed");
                        }

                        if (mFloorAlignment) {
                            if (mInitOdomWithPose) {
                                stat.add("Floor Detection", "NOT INITIALIZED");
                            } else {
                                stat.add("Floor Detection", "INITIALIZED");
                            }
                        }

                        if (mTrackingActivated) {
                            stat.addf("Tracking status", "%s", sl::toString(mTrackingStatus).c_str());
                        } else {
                            stat.add("Tracking status", "INACTIVE");
                        }
                    } else {
                        stat.add("Depth status", "INACTIVE");
                    }
                } else {
                    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera error: %s", sl::toString(mGrabStatus).c_str());
                }

            } else {
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Waiting for data subscriber");
                stat.add("Capture", "INACTIVE");
            }

            if (mImuPublishing) {
                double freq = 1000000. / mImuPeriodMean_usec->getMean();
                double freq_perc = 100.*freq / mImuPubRate;
                stat.addf("IMU", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
            } else {
                stat.add("IMU", "Topics not subscribed");
            }
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, sl::toString(mConnStatus).c_str());
        }

    }
} // namespace
