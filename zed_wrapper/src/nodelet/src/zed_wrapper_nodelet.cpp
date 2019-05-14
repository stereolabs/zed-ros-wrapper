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


        // Node handlers
        mNh = getMTNodeHandle();
        mNhNs = getMTPrivateNodeHandle();

        mStopNode = false;
        mPcDataReady = false;

#ifndef NDEBUG

        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                           ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

#endif
        std::string ver = sl_tools::getSDKVersion(mVerMajor, mVerMinor, mVerSubMinor);
        NODELET_INFO_STREAM("SDK version : " << ver);

        readParameters();
        initTransforms();

        std::string img_topic = "/image_rect_color";
        std::string img_raw_topic = "/image_raw_color";
        std::string raw_suffix = "_raw";

        // Set the video topic names
        string left_topic = mLeftTopicRoot + img_topic;
        string left_raw_topic = mLeftTopicRoot + raw_suffix + img_raw_topic;
        string right_topic = mRightTopicRoot + img_topic;
        string right_raw_topic = mRightTopicRoot + raw_suffix + img_raw_topic;
        string rgb_topic = mRgbTopicRoot + img_topic;
        string rgb_raw_topic = mRgbTopicRoot + raw_suffix + img_raw_topic;
        string stereo_topic = mStereoTopicRoot + img_topic;
        string stereo_raw_topic = mStereoTopicRoot + raw_suffix + img_raw_topic;

        // Set the depth topic names
        string depth_topic = mDepthTopicRoot;

        if (mOpenniDepthMode) {
            NODELET_INFO_STREAM("Openni depth mode activated");
            depth_topic += "/depth_raw_registered";
        } else {
            depth_topic += "/depth_registered";
        }

        string pointcloud_topic = mPointCloudTopicRoot + "/cloud_registered";
        string pointcloud_fused_topic = mPointCloudTopicRoot + "/fused_cloud_registered";

        string conf_img_topic_name = "confidence_image";
        string conf_map_topic_name = "confidence_map";
        string conf_img_topic = mConfImgRoot + "/" + conf_img_topic_name;
        string conf_map_topic = mConfImgRoot + "/" + conf_map_topic_name;

        // Set the positional tracking topic names
        string pose_cov_topic;
        pose_cov_topic = mPoseTopic + "_with_covariance";

        string odom_path_topic = "path_odom";
        string map_path_topic = "path_map";

        // Create camera info
        mRgbCamInfoMsg.reset(new sensor_msgs::CameraInfo());
        mLeftCamInfoMsg.reset(new sensor_msgs::CameraInfo());
        mRightCamInfoMsg.reset(new sensor_msgs::CameraInfo());
        mRgbCamInfoRawMsg.reset(new sensor_msgs::CameraInfo());
        mLeftCamInfoRawMsg.reset(new sensor_msgs::CameraInfo());
        mRightCamInfoRawMsg.reset(new sensor_msgs::CameraInfo());
        mDepthCamInfoMsg.reset(new sensor_msgs::CameraInfo());

        // Initialization transformation listener
        mTfBuffer.reset(new tf2_ros::Buffer);
        mTfListener.reset(new tf2_ros::TransformListener(*mTfBuffer));

        // Try to initialize the ZED
        if (!mSvoFilepath.empty() || !mRemoteStreamAddr.empty()) {

            if (!mSvoFilepath.empty()) {
                mZedParams.svo_input_filename = mSvoFilepath.c_str();
                mZedParams.svo_real_time_mode = false;
            } else  if (!mRemoteStreamAddr.empty()) {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )
                std::vector<std::string> configStream = sl_tools::split_string(mRemoteStreamAddr, ':');
                sl::String ip = sl::String(configStream.at(0).c_str());

                if (configStream.size() == 2) {
                    mZedParams.input.setFromStream(ip, atoi(configStream.at(1).c_str()));
                } else {
                    mZedParams.input.setFromStream(ip);
                }

#else
                ROS_ERROR_STREAM("Acquiring a remote stream requires the ZED SDK v2.8 or newer");
                return;
#endif
            }

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

#if (ZED_SDK_MAJOR_VERSION<2)
        NODELET_WARN_STREAM("Please consider to upgrade to latest SDK version to "
                            "get better performances");

        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;

        NODELET_INFO_STREAM(" * Camera coordinate system\t-> COORDINATE_SYSTEM_IMAGE");
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

        NODELET_INFO_STREAM(" * Camera coordinate system\t-> COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP");
        mIdxX = 1;
        mIdxY = 0;
        mIdxZ = 2;
        mSignX = 1;
        mSignY = -1;
        mSignZ = 1;
#else
        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;

        NODELET_INFO_STREAM(" * Camera coordinate system\t-> COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD");
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
        mZedParams.depth_minimum_distance = static_cast<float>(mCamMinDepth);
        mZedParams.camera_disable_self_calib = !mCameraSelfCalib;

        if (mVerMajor > 2 || (mVerMajor == 2 && mVerMinor >= 8)) {
            //mZedParams.color_enhancement = mColorEnhancement; TODO uncomment when the paramenter is available
        }

        mDiagUpdater.add("ZED Diagnostic", this, &ZEDWrapperNodelet::updateDiagnostic);
        mDiagUpdater.setHardwareID("ZED camera");

        mConnStatus = sl::ERROR_CODE_CAMERA_NOT_DETECTED;

        while (mConnStatus != sl::SUCCESS) {
            mConnStatus = mZed.open(mZedParams);
            NODELET_INFO_STREAM("ZED connection -> " << sl::toString(mConnStatus));
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

        if (mZedRealCamModel == sl::MODEL_ZED) {
            if (mZedUserCamModel != 0) {
                NODELET_WARN("Camera model does not match user parameter. Please modify "
                             "the value of the parameter 'camera_model' to 0");
            }
        } else if (mZedRealCamModel == sl::MODEL_ZED_M) {
            if (mZedUserCamModel != 1) {
                NODELET_WARN("Camera model does not match user parameter. Please modify "
                             "the value of the parameter 'camera_model' to 1");
            }
        }

        NODELET_INFO_STREAM(" * CAMERA MODEL\t -> " << sl::toString(mZedRealCamModel).c_str());
        mZedSerialNumber = mZed.getCameraInformation().serial_number;
        NODELET_INFO_STREAM(" * Serial Number -> " << mZedSerialNumber);

        if (!mSvoMode) {
            mFwVersion = mZed.getCameraInformation().firmware_version;
            NODELET_INFO_STREAM(" * FW Version\t -> " << mFwVersion);
        } else {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )
            NODELET_INFO_STREAM(" * Input type\t -> " << sl::toString(mZed.getCameraInformation().input_type).c_str());
#else
            NODELET_INFO_STREAM(" * Input type\t -> SVO");
#endif
        }

        // Set the IMU topic names using real camera model
        string imu_topic;
        string imu_topic_raw;

        if (mZedRealCamModel == sl::MODEL_ZED_M) {
            string imu_topic_name = "data";
            string imu_topic_raw_name = "data_raw";
            imu_topic = mImuTopicRoot + "/" + imu_topic_name;
            imu_topic_raw = mImuTopicRoot + "/" + imu_topic_raw_name;
        }

        mDiagUpdater.setHardwareIDf("%s-%d", sl::toString(mZedRealCamModel).c_str(), mZedSerialNumber);

        // Dynamic Reconfigure parameters
        mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<zed_wrapper::ZedConfig>>();
        dynamic_reconfigure::Server<zed_wrapper::ZedConfig>::CallbackType f;
        f = boost::bind(&ZEDWrapperNodelet::dynamicReconfCallback, this, _1, _2);
        mDynRecServer->setCallback(f);

        // Create all the publishers
        // Image publishers
        //        image_transport::ImageTransport it_zed(mNhNs);
        image_transport::ImageTransport it_zed(mNhNs);

        mPubRgb = it_zed.advertiseCamera(rgb_topic, 1); // rgb
        NODELET_INFO_STREAM("Advertised on topic " << mPubRgb.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubRgb.getInfoTopic());
        mPubRawRgb = it_zed.advertiseCamera(rgb_raw_topic, 1); // rgb raw
        NODELET_INFO_STREAM("Advertised on topic " << mPubRawRgb.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubRawRgb.getInfoTopic());
        mPubLeft = it_zed.advertiseCamera(left_topic, 1); // left
        NODELET_INFO_STREAM("Advertised on topic " << mPubLeft.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubLeft.getInfoTopic());
        mPubRawLeft = it_zed.advertiseCamera(left_raw_topic, 1); // left raw
        NODELET_INFO_STREAM("Advertised on topic " << mPubRawLeft.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubRawLeft.getInfoTopic());
        mPubRight = it_zed.advertiseCamera(right_topic, 1); // right
        NODELET_INFO_STREAM("Advertised on topic " << mPubRight.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubRight.getInfoTopic());
        mPubRawRight = it_zed.advertiseCamera(right_raw_topic, 1); // right raw
        NODELET_INFO_STREAM("Advertised on topic " << mPubRawRight.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubRawRight.getInfoTopic());
        mPubDepth = it_zed.advertiseCamera(depth_topic, 1); // depth
        NODELET_INFO_STREAM("Advertised on topic " << mPubDepth.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubDepth.getInfoTopic());
        mPubConfImg = it_zed.advertiseCamera(conf_img_topic, 1); // confidence image
        NODELET_INFO_STREAM("Advertised on topic " << mPubConfImg.getTopic());
        NODELET_INFO_STREAM("Advertised on topic " << mPubConfImg.getInfoTopic());

        mPubStereo = it_zed.advertise(stereo_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubStereo.getTopic());
        mPubRawStereo = it_zed.advertise(stereo_raw_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubRawStereo.getTopic());

        // Confidence Map publisher
        mPubConfMap = mNhNs.advertise<sensor_msgs::Image>(conf_map_topic, 1); // confidence map
        NODELET_INFO_STREAM("Advertised on topic " << mPubConfMap.getTopic());

        // Disparity publisher
        mPubDisparity = mNhNs.advertise<stereo_msgs::DisparityImage>(mDisparityTopic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubDisparity.getTopic());

        // PointCloud publisher
        mPointcloudMsg.reset(new sensor_msgs::PointCloud2);
        mPubCloud = mNhNs.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubCloud.getTopic());

#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )

        if (mMappingEnabled) {
            mPointcloudFusedMsg.reset(new sensor_msgs::PointCloud2);
            mPubFusedCloud = mNhNs.advertise<sensor_msgs::PointCloud2>(pointcloud_fused_topic, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubFusedCloud.getTopic() << " @ " << mFusedPcPubFreq << " Hz");
        }

#endif

        // Odometry and Pose publisher
        mPubPose = mNhNs.advertise<geometry_msgs::PoseStamped>(mPoseTopic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubPose.getTopic());

        if (mPublishPoseCovariance) {
            mPubPoseCov = mNhNs.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_cov_topic, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubPoseCov.getTopic());
        }

        mPubOdom = mNhNs.advertise<nav_msgs::Odometry>(mOdometryTopic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubOdom.getTopic());

        // Camera Path
        if (mPathPubRate > 0) {
            mPubOdomPath = mNhNs.advertise<nav_msgs::Path>(odom_path_topic, 1, true);
            NODELET_INFO_STREAM("Advertised on topic " << mPubOdomPath.getTopic());
            mPubMapPath = mNhNs.advertise<nav_msgs::Path>(map_path_topic, 1, true);
            NODELET_INFO_STREAM("Advertised on topic " << mPubMapPath.getTopic());

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

        if (!mSvoMode) {
            if (mImuPubRate > 0 && mZedRealCamModel == sl::MODEL_ZED_M) {
                mPubImu = mNhNs.advertise<sensor_msgs::Imu>(imu_topic, 500);
                NODELET_INFO_STREAM("Advertised on topic " << mPubImu.getTopic() << " @ "
                                    << mImuPubRate << " Hz");
                mPubImuRaw = mNhNs.advertise<sensor_msgs::Imu>(imu_topic_raw, 500);
                NODELET_INFO_STREAM("Advertised on topic " << mPubImuRaw.getTopic() << " @ "
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
        }

        // Services
        mSrvSetInitPose = mNhNs.advertiseService("set_pose", &ZEDWrapperNodelet::on_set_pose, this);
        mSrvResetOdometry = mNhNs.advertiseService("reset_odometry", &ZEDWrapperNodelet::on_reset_odometry, this);
        mSrvResetTracking = mNhNs.advertiseService("reset_tracking", &ZEDWrapperNodelet::on_reset_tracking, this);
        mSrvSvoStartRecording = mNhNs.advertiseService("start_svo_recording", &ZEDWrapperNodelet::on_start_svo_recording, this);
        mSrvSvoStopRecording = mNhNs.advertiseService("stop_svo_recording", &ZEDWrapperNodelet::on_stop_svo_recording, this);

        if (mVerMajor > 2 || (mVerMajor == 2 && mVerMinor >= 8)) {
            mSrvSetLedStatus = mNhNs.advertiseService("set_led_status", &ZEDWrapperNodelet::on_set_led_status, this);
            mSrvToggleLed = mNhNs.advertiseService("toggle_led", &ZEDWrapperNodelet::on_toggle_led, this);
            mSrvSvoStartStream = mNhNs.advertiseService("start_remote_stream", &ZEDWrapperNodelet::on_start_remote_stream, this);
            mSrvSvoStopStream = mNhNs.advertiseService("stop_remote_stream", &ZEDWrapperNodelet::on_stop_remote_stream, this);
        }

        // Start Pointcloud thread
        mPcThread = std::thread(&ZEDWrapperNodelet::pointcloud_thread_func, this);

        // Start pool thread
        mDevicePollThread = std::thread(&ZEDWrapperNodelet::device_poll_thread_func, this);
    }

    void ZEDWrapperNodelet::readParameters() {

        NODELET_INFO_STREAM("*** PARAMETERS ***");

        // ----> General
        // Get parameters from param files
        mNhNs.getParam("general/resolution", mCamResol);
        NODELET_INFO_STREAM(" * Camera Resolution\t\t-> " << sl::toString(static_cast<sl::RESOLUTION>
                            (mCamResol)).c_str());
        mNhNs.getParam("general/frame_rate", mCamFrameRate);
        checkResolFps();
        NODELET_INFO_STREAM(" * Camera Framerate\t\t-> " << mCamFrameRate);
        mNhNs.getParam("general/gpu_id", mGpuId);
        NODELET_INFO_STREAM(" * Gpu ID\t\t\t-> " << mGpuId);
        mNhNs.getParam("general/zed_id", mZedId);
        NODELET_INFO_STREAM(" * Camera ID\t\t\t-> " << mGpuId);
        mNhNs.getParam("general/verbose", mVerbose);
        NODELET_INFO_STREAM(" * Verbose\t\t\t-> " << (mVerbose ? "ENABLED" : "DISABLED"));
        mNhNs.param<bool>("general/camera_flip", mCameraFlip, false);
        NODELET_INFO_STREAM(" * Camera Flip\t\t\t-> " << (mCameraFlip ? "ENABLED" : "DISABLED"));
        mNhNs.param<bool>("general/self_calib", mCameraSelfCalib, true);
        NODELET_INFO_STREAM(" * Self calibration\t\t-> " << (mCameraSelfCalib ? "ENABLED" : "DISABLED"));

        int tmp_sn = 0;
        mNhNs.getParam("general/serial_number", tmp_sn);

        if (tmp_sn > 0) {
            mZedSerialNumber = static_cast<int>(tmp_sn);
            NODELET_INFO_STREAM(" * Serial number\t\t-> " <<  mZedSerialNumber);
        }


        string camera_model;
        mNhNs.getParam("camera_model", camera_model);

        if (camera_model == "zed") {
            mZedUserCamModel = 0;
            NODELET_INFO_STREAM(" * Camera Model\t\t\t-> " <<  camera_model);
        } else if (camera_model == "zedm") {
            mZedUserCamModel = 1;
            NODELET_INFO_STREAM(" * Camera Model\t\t\t-> " <<  camera_model);
        } else {
            NODELET_ERROR_STREAM("Camera model not valid: " << camera_model);
        }

        // <---- General

        // ----> Video
        mNhNs.getParam("video/color_enhancement", mColorEnhancement); // TODO Future use?

        mNhNs.param<std::string>("video/rgb_topic_root", mRgbTopicRoot, "rgb");
        mNhNs.param<std::string>("video/right_topic_root", mRightTopicRoot, "right");
        mNhNs.param<std::string>("video/left_topic_root", mLeftTopicRoot, "left");
        mNhNs.param<std::string>("video/stereo_topic_root", mStereoTopicRoot, "stereo");
        // <---- Video

        // -----> Depth
        mNhNs.param<std::string>("depth/depth_topic_root", mDepthTopicRoot, "depth");
        mNhNs.param<std::string>("depth/disparity_topic", mDisparityTopic, "disparity/disparity_image");
        mNhNs.param<std::string>("depth/point_cloud_topic_root", mPointCloudTopicRoot, "point_cloud");
        mNhNs.param<std::string>("depth/confidence_root", mConfImgRoot, "confidence");

        mNhNs.getParam("depth/quality", mCamQuality);
        NODELET_INFO_STREAM(" * Depth quality\t\t-> " << sl::toString(static_cast<sl::DEPTH_MODE>(mCamQuality)).c_str());
        mNhNs.getParam("depth/sensing_mode", mCamSensingMode);
        NODELET_INFO_STREAM(" * Depth Sensing mode\t\t-> " << sl::toString(static_cast<sl::SENSING_MODE>
                            (mCamSensingMode)).c_str());
        mNhNs.getParam("depth/openni_depth_mode", mOpenniDepthMode);
        NODELET_INFO_STREAM(" * OpenNI mode\t\t\t-> " << (mOpenniDepthMode ? "ENABLED" : "DISABLED"));
        mNhNs.getParam("depth/depth_stabilization", mDepthStabilization);
        NODELET_INFO_STREAM(" * Depth Stabilization\t\t-> " << (mDepthStabilization ? "ENABLED" : "DISABLED"));
        mNhNs.getParam("depth/min_depth", mCamMinDepth);
        NODELET_INFO_STREAM(" * Minimum depth\t\t-> " <<  mCamMinDepth);
        // <----- Depth

        // ----> Tracking
        mNhNs.param<std::string>("tracking/pose_topic", mPoseTopic, "pose");
        mNhNs.param<std::string>("tracking/odometry_topic", mOdometryTopic, "odom");

        mNhNs.getParam("tracking/path_pub_rate", mPathPubRate);
        NODELET_INFO_STREAM(" * Path rate\t\t\t-> " <<  mPathPubRate << " Hz");
        mNhNs.getParam("tracking/path_max_count", mPathMaxCount);
        NODELET_INFO_STREAM(" * Path history size\t\t-> " << (mPathMaxCount == -1) ? std::string("INFINITE") : std::to_string(
                                mPathMaxCount));

        if (mPathMaxCount < 2 && mPathMaxCount != -1) {
            mPathMaxCount = 2;
        }

        mNhNs.getParam("tracking/initial_base_pose", mInitialBasePose);

        mNhNs.getParam("tracking/odometry_DB", mOdometryDb);
        NODELET_INFO_STREAM(" * Odometry DB path\t\t-> " << mOdometryDb.c_str());
        mNhNs.param<bool>("tracking/spatial_memory", mSpatialMemory, false);
        NODELET_INFO_STREAM(" * Spatial Memory\t\t-> " << (mSpatialMemory ? "ENABLED" : "DISABLED"));
        mNhNs.param<bool>("tracking/imu_fusion", mImuFusion, true);
        NODELET_INFO_STREAM(" * IMU Fusion\t\t\t-> " << (mImuFusion ? "ENABLED" : "DISABLED"));
        mNhNs.param<bool>("tracking/floor_alignment", mFloorAlignment, false);
        NODELET_INFO_STREAM(" * Floor alignment\t\t-> " << (mFloorAlignment ? "ENABLED" : "DISABLED"));
        mNhNs.param<bool>("tracking/init_odom_with_first_valid_pose", mInitOdomWithPose, true);
        NODELET_INFO_STREAM(" * Init Odometry with first valid pose data -> " << (mInitOdomWithPose ? "ENABLED" : "DISABLED"));
        mNhNs.param<bool>("tracking/two_d_mode", mTwoDMode, false);
        NODELET_INFO_STREAM(" * Two D mode\t\t\t-> " << (mTwoDMode ? "ENABLED" : "DISABLED"));
        mNhNs.param<double>("tracking/fixed_z_value", mFixedZValue, 0.0);

        if (mTwoDMode) {
            NODELET_INFO_STREAM(" * Fixed Z value\t\t-> " << mFixedZValue);
        }

        mNhNs.getParam("tracking/publish_pose_covariance", mPublishPoseCovariance);
        NODELET_INFO_STREAM(" * Publish Pose Covariance\t-> " << (mPublishPoseCovariance ? "ENABLED" : "DISABLED"));

        if (mVerMajor > 2 || (mVerMajor == 2 && mVerMinor >= 8)) {
            mNhNs.getParam("tracking/fixed_covariance", mFixedCov);
        } else {
            if (mFixedCov) {
                ROS_WARN("Dynamic covariance is available with SDK v2.8 or newer");
            }

            mFixedCov = true;
        }

        NODELET_INFO_STREAM(" * Fixed covariance\t\t-> " << (mFixedCov ? "ENABLED" : "DISABLED"));

        mNhNs.getParam("tracking/fixed_cov_value", mFixedCovValue);
        NODELET_INFO_STREAM(" * Fixed cov. value\t\t-> " << mFixedCovValue);
        // <---- Tracking

        // ----> Mapping
        mNhNs.param<bool>("mapping/mapping_enabled", mMappingEnabled, false);

        if (mMappingEnabled) {
            if ((mVerMajor == 2 && mVerMinor < 8) || mVerMajor < 2) {
                NODELET_WARN_STREAM("The mapping module is available with SDK v2.8 or newer");
                mMappingEnabled = false;
                NODELET_INFO_STREAM(" * Mapping\t\t\t-> DISABLED");
            } else {
                NODELET_INFO_STREAM(" * Mapping\t\t\t-> ENABLED");

                mNhNs.getParam("mapping/resolution", mMappingRes);
                NODELET_INFO_STREAM(" * Mapping resolution\t\t-> " << sl::toString(
                                        static_cast<sl::SpatialMappingParameters::MAPPING_RESOLUTION>(mMappingRes)));
                mNhNs.getParam("mapping/fused_pointcloud_freq", mFusedPcPubFreq);
                NODELET_INFO_STREAM(" * Fused point cloud freq:\t-> " << mFusedPcPubFreq << " Hz");
            }
        } else {
            NODELET_INFO_STREAM(" * Mapping\t\t\t-> DISABLED");
        }

        // <---- Mapping


        // ----> IMU
        mNhNs.param<std::string>("imu/imu_topic_root", mImuTopicRoot, "imu");
        mNhNs.getParam("imu/imu_timestamp_sync", mImuTimestampSync);
        NODELET_INFO_STREAM(" * IMU timestamp sync\t\t-> " << (mImuTimestampSync ? "ENABLED" : "DISABLED"));
        mNhNs.getParam("imu/imu_pub_rate", mImuPubRate);
        NODELET_INFO_STREAM(" * IMU data freq\t\t-> " << mImuPubRate << " Hz");
        // <---- IMU

        // ----> SVO
        mNhNs.param<std::string>("svo_file", mSvoFilepath, std::string());
        int svo_compr = 0;
        mNhNs.getParam("general/svo_compression", svo_compr);

        if (svo_compr >= sl::SVO_COMPRESSION_MODE_LAST) {
            NODELET_WARN_STREAM("The parameter `general/svo_compression` has an invalid value. Please check it in the configuration file `common.yaml`");

            if ((mVerMajor == 2 && mVerMinor < 7) || mVerMajor < 2) {
                NODELET_WARN_STREAM("Note: AVCHD (H264) and HEVC (H265) compression modes are available with SDK v2.7 or newer");
            }

            svo_compr = 0;
        }

        mSvoComprMode = static_cast<sl::SVO_COMPRESSION_MODE>(svo_compr);

        NODELET_INFO_STREAM(" * SVO REC compression\t\t-> " << sl::toString(mSvoComprMode));
        // <---- SVO

        // Remote Stream
        mNhNs.param<std::string>("stream", mRemoteStreamAddr, std::string());

        // ----> Coordinate frames
        mNhNs.param<std::string>("tracking/world_frame", mWorldFrameId, "map");
        mNhNs.param<std::string>("tracking/map_frame", mMapFrameId, "map");
        mNhNs.param<std::string>("tracking/odometry_frame", mOdometryFrameId, "odom");
        mNhNs.param<std::string>("general/base_frame", mBaseFrameId, "base_link");
        mNhNs.param<std::string>("general/camera_frame", mCameraFrameId, "zed_camera_center");
        mNhNs.param<std::string>("imu/imu_frame", mImuFrameId, "imu_link");
        mNhNs.param<std::string>("general/left_camera_frame", mLeftCamFrameId, "left_camera_frame");
        mNhNs.param<std::string>("general/left_camera_optical_frame", mLeftCamOptFrameId, "left_camera_optical_frame");
        mNhNs.param<std::string>("general/right_camera_frame", mRightCamFrameId, "right_camera_frame");
        mNhNs.param<std::string>("general/right_camera_optical_frame", mRightCamOptFrameId, "right_camera_optical_frame");
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

        // Print TF frames
        NODELET_INFO_STREAM(" * world_frame\t\t\t-> " << mWorldFrameId);
        NODELET_INFO_STREAM(" * map_frame\t\t\t-> " << mMapFrameId);
        NODELET_INFO_STREAM(" * odometry_frame\t\t-> " << mOdometryFrameId);
        NODELET_INFO_STREAM(" * base_frame\t\t\t-> " << mBaseFrameId);
        NODELET_INFO_STREAM(" * camera_frame\t\t\t-> " << mCameraFrameId);
        NODELET_INFO_STREAM(" * imu_link\t\t\t-> " << mImuFrameId);
        NODELET_INFO_STREAM(" * left_camera_frame\t\t-> " << mLeftCamFrameId);
        NODELET_INFO_STREAM(" * left_camera_optical_frame\t-> " << mLeftCamOptFrameId);
        NODELET_INFO_STREAM(" * right_camera_frame\t\t-> " << mRightCamFrameId);
        NODELET_INFO_STREAM(" * right_camera_optical_frame\t-> " << mRightCamOptFrameId);
        NODELET_INFO_STREAM(" * depth_frame\t\t\t-> " << mDepthFrameId);
        NODELET_INFO_STREAM(" * depth_optical_frame\t\t-> " << mDepthOptFrameId);
        NODELET_INFO_STREAM(" * disparity_frame\t\t-> " << mDisparityFrameId);
        NODELET_INFO_STREAM(" * disparity_optical_frame\t-> " << mDisparityOptFrameId);
        NODELET_INFO_STREAM(" * confidence_frame\t\t-> " << mConfidenceFrameId);
        NODELET_INFO_STREAM(" * confidence_optical_frame\t-> " << mConfidenceOptFrameId);
        // <---- Coordinate frames

        // ----> TF broadcasting
        mNhNs.param<bool>("tracking/publish_tf", mPublishTf, true);
        NODELET_INFO_STREAM(" * Broadcast odometry TF\t-> " << (mPublishTf ? "ENABLED" : "DISABLED"));
        mNhNs.param<bool>("tracking/publish_map_tf", mPublishMapTf, true);
        NODELET_INFO_STREAM(" * Broadcast map pose TF\t-> " << (mPublishTf ? (mPublishMapTf ? "ENABLED" : "DISABLED") :
                            "DISABLED"));
        // <---- TF broadcasting

        // ----> Dynamic
        mNhNs.getParam("mat_resize_factor", mCamMatResizeFactor);


        if (mCamMatResizeFactor < 0.1) {
            mCamMatResizeFactor = 0.1;
            NODELET_WARN_STREAM("Minimum allowed values for 'mat_resize_factor' is 0.1");
        }

        if (mCamMatResizeFactor > 1.0) {
            mCamMatResizeFactor = 1.0;
            NODELET_WARN_STREAM("Maximum allowed values for 'mat_resize_factor' is 1.0");
        }

        NODELET_INFO_STREAM(" * [DYN] mat_resize_factor\t-> " << mCamMatResizeFactor);

        mNhNs.getParam("confidence", mCamConfidence);
        NODELET_INFO_STREAM(" * [DYN] confidence\t\t-> " << mCamConfidence);
        mNhNs.getParam("max_depth", mCamMaxDepth);
        NODELET_INFO_STREAM(" * [DYN] max_depth\t\t-> " << mCamMaxDepth);
        mNhNs.getParam("exposure", mCamExposure);
        NODELET_INFO_STREAM(" * [DYN] exposure\t\t-> " << mCamExposure);
        mNhNs.getParam("gain", mCamGain);
        NODELET_INFO_STREAM(" * [DYN] gain\t\t\t-> " << mCamGain);
        mNhNs.getParam("auto_exposure", mCamAutoExposure);
        NODELET_INFO_STREAM(" * [DYN] auto_exposure\t\t-> " << (mCamAutoExposure ? "ENABLED" : "DISABLED"));
        mNhNs.getParam("point_cloud_freq", mPointCloudFreq);
        NODELET_INFO_STREAM(" * [DYN] point_cloud_freq\t-> " << mPointCloudFreq << " Hz");

        if (mCamAutoExposure) {
            mTriggerAutoExposure = true;
        }

        // <---- Dynamic

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
        // According to REP 105 -> http://www.ros.org/reps/rep-0105.html

        // base_link <- odom <- map
        //     ^                 |
        //     |                 |
        //     -------------------

        // ----> Dynamic transforms
        mOdom2BaseTransf.setIdentity();     // broadcasted if `publish_tf` is true
        mMap2OdomTransf.setIdentity();      // broadcasted if `publish_map_tf` is true
        mMap2BaseTransf.setIdentity();      // used internally, but not broadcasted
        mMap2CameraTransf.setIdentity();    // used internally, but not broadcasted
        // <---- Dynamic transforms
    }

    bool ZEDWrapperNodelet::getCamera2BaseTransform() {
        ROS_DEBUG("Getting static TF from '%s' to '%s'", mCameraFrameId.c_str(), mBaseFrameId.c_str());

        mCamera2BaseTransfValid = false;
        static int errCount = 0;

        // ----> Static transforms
        // Sensor to Base link
        try {

            // Save the transformation
            geometry_msgs::TransformStamped c2b =
                mTfBuffer->lookupTransform(mCameraFrameId, mBaseFrameId, ros::Time(0), ros::Duration(2));

            // Get the TF2 transformation
            tf2::fromMsg(c2b.transform, mCamera2BaseTransf);

            double roll, pitch, yaw;
            tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

            ROS_INFO("Static transform Camera Center to Base [%s -> %s]",
                     mCameraFrameId.c_str(), mBaseFrameId.c_str());
            ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                     mCamera2BaseTransf.getOrigin().x(), mCamera2BaseTransf.getOrigin().y(), mCamera2BaseTransf.getOrigin().z());
            ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                     roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

        } catch (tf2::TransformException& ex) {
            if (++errCount % 50 == 0) {
                ROS_WARN("The tf from '%s' to '%s' does not seem to be available, "
                         "will assume it as identity!",
                         mCameraFrameId.c_str(), mBaseFrameId.c_str());
                ROS_WARN("Transform error: %s", ex.what());
            }

            mCamera2BaseTransf.setIdentity();
            return false;
        }

        // <---- Static transforms

        errCount = 0;
        mCamera2BaseTransfValid = true;
        return true;
    }

    bool ZEDWrapperNodelet::getSens2CameraTransform() {
        ROS_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mCameraFrameId.c_str());

        mSensor2CameraTransfValid = false;
        static int errCount = 0;

        // ----> Static transforms
        // Sensor to Camera Center
        try {
            // Save the transformation
            geometry_msgs::TransformStamped s2c =
                mTfBuffer->lookupTransform(mDepthFrameId, mCameraFrameId, ros::Time(0), ros::Duration(2));
            // Get the TF2 transformation
            tf2::fromMsg(s2c.transform, mSensor2CameraTransf);

            double roll, pitch, yaw;
            tf2::Matrix3x3(mSensor2CameraTransf.getRotation()).getRPY(roll, pitch, yaw);

            ROS_INFO("Static transform Sensor to Camera Center [%s -> %s]",
                     mDepthFrameId.c_str(), mCameraFrameId.c_str());
            ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                     mSensor2CameraTransf.getOrigin().x(), mSensor2CameraTransf.getOrigin().y(), mSensor2CameraTransf.getOrigin().z());
            ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                     roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
        } catch (tf2::TransformException& ex) {
            if (++errCount % 50 == 0) {
                ROS_WARN("The tf from '%s' to '%s' does not seem to be available, "
                         "will assume it as identity!",
                         mDepthFrameId.c_str(), mCameraFrameId.c_str());
                ROS_WARN("Transform error: %s", ex.what());
            }

            mSensor2CameraTransf.setIdentity();
            return false;
        }

        // <---- Static transforms

        errCount = 0;
        mSensor2CameraTransfValid = true;
        return true;
    }

    bool ZEDWrapperNodelet::getSens2BaseTransform() {
        ROS_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mBaseFrameId.c_str());

        mSensor2BaseTransfValid = false;
        static int errCount = 0;

        // ----> Static transforms
        // Sensor to Base link
        try {
            // Save the transformation
            geometry_msgs::TransformStamped s2b =
                mTfBuffer->lookupTransform(mDepthFrameId, mBaseFrameId, ros::Time(0), ros::Duration(2));
            // Get the TF2 transformation
            tf2::fromMsg(s2b.transform, mSensor2BaseTransf);

            double roll, pitch, yaw;
            tf2::Matrix3x3(mSensor2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

            ROS_INFO("Static transform Sensor to Base [%s -> %s]",
                     mDepthFrameId.c_str(), mBaseFrameId.c_str());
            ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                     mSensor2BaseTransf.getOrigin().x(), mSensor2BaseTransf.getOrigin().y(), mSensor2BaseTransf.getOrigin().z());
            ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                     roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

        } catch (tf2::TransformException& ex) {
            if (++errCount % 50 == 0) {
                ROS_WARN("The tf from '%s' to '%s' does not seem to be available, "
                         "will assume it as identity!",
                         mDepthFrameId.c_str(), mBaseFrameId.c_str());
                ROS_WARN("Transform error: %s", ex.what());
            }

            mSensor2BaseTransf.setIdentity();
            return false;
        }

        // <---- Static transforms

        errCount = 0;
        mSensor2BaseTransfValid = true;
        return true;
    }

    bool ZEDWrapperNodelet::set_pose(float xt, float yt, float zt, float rr,
                                     float pr, float yr) {
        initTransforms();

        if (!mSensor2BaseTransfValid) {
            getSens2BaseTransform();
        }

        if (!mSensor2CameraTransfValid) {
            getSens2CameraTransform();
        }

        if (!mCamera2BaseTransfValid) {
            getCamera2BaseTransform();
        }

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

        return (mSensor2BaseTransfValid & mSensor2CameraTransfValid & mCamera2BaseTransfValid);

    }

    bool ZEDWrapperNodelet::on_set_pose(
        zed_wrapper::set_pose::Request& req,
        zed_wrapper::set_pose::Response& res) {
        mInitialBasePose.resize(6);
        mInitialBasePose[0] = req.x;
        mInitialBasePose[1] = req.y;
        mInitialBasePose[2] = req.z;
        mInitialBasePose[3] = req.R;
        mInitialBasePose[4] = req.P;
        mInitialBasePose[5] = req.Y;

        if (!set_pose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2],
                      mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5])) {
            res.done = false;
            return false;
        }

        std::lock_guard<std::mutex> lock(mPosTrkMutex);

        // Disable tracking
        mTrackingActivated = false;
        mZed.disableTracking();

        // Restart tracking
        start_tracking();

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

        if (!set_pose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2],
                      mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5])) {
            res.reset_done = false;
            return false;
        }

        std::lock_guard<std::mutex> lock(mPosTrkMutex);

        // Disable tracking
        mTrackingActivated = false;
        mZed.disableTracking();

        // Restart tracking
        start_tracking();

        res.reset_done = true;
        return true;
    }

    bool ZEDWrapperNodelet::on_reset_odometry(
        zed_wrapper::reset_odometry::Request& req,
        zed_wrapper::reset_odometry::Response& res) {
        mResetOdom = true;
        res.reset_done = true;
        return true;
    }

    void ZEDWrapperNodelet::start_mapping() {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )

        if (!mMappingEnabled) {
            NODELET_WARN_STREAM("Cannot enable MAPPING. The parameter `mapping_enable` is set to FALSE");

            return;
        }

        NODELET_INFO_STREAM("*** Starting Spatial Mapping ***");

        sl::SpatialMappingParameters params;
        params.map_type = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE_FUSED_POINT_CLOUD;
        params.use_chunk_only = true;
        params.set(static_cast<sl::SpatialMappingParameters::MAPPING_RESOLUTION>(mMappingRes));

        sl::ERROR_CODE err = mZed.enableSpatialMapping(params);

        if (err == sl::SUCCESS) {
            mMappingActivated = true;

            mFusedPcTimer = mNhNs.createTimer(ros::Duration(1.0 / mFusedPcPubFreq), &ZEDWrapperNodelet::pubFusedPointCloudCallback,
                                              this);

            ROS_INFO_STREAM(" * Resolution: " << params.resolution_meter << " m");
        } else {
            mMappingActivated = false;
            mFusedPcTimer.stop();

            ROS_WARN("Mapping not activated: %s", sl::toString(err).c_str());
        }

#else
        NODELET_WARN("Enabling MAPPING requires the ZED SDK v2.8 or newer");
#endif
    }

    void ZEDWrapperNodelet::start_tracking() {
        NODELET_INFO_STREAM("*** Starting Positional Tracking ***");

        ROS_INFO(" * Waiting for valid static transformations...");

        bool transformOk = false;
        double elapsed = 0.0;

        auto start = std::chrono::high_resolution_clock::now();

        do {
            transformOk = set_pose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2],
                                   mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5]);

            elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() -
                      start).count();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (elapsed > 10000) {
                ROS_WARN(" !!! Failed to get static transforms. Is the 'ROBOT STATE PUBLISHER' node correctly working? ");
                break;
            }

        } while (transformOk == false);

        if (transformOk) {
            ROS_DEBUG("Time required to get valid static transforms: %g sec", elapsed / 1000.);
        }

        ROS_INFO("Initial ZED left camera pose (ZED pos. tracking): ");
        ROS_INFO(" * T: [%g,%g,%g]",
                 mInitialPoseSl.getTranslation().x, mInitialPoseSl.getTranslation().y, mInitialPoseSl.getTranslation().z);
        ROS_INFO(" * Q: [%g,%g,%g,%g]",
                 mInitialPoseSl.getOrientation().ox, mInitialPoseSl.getOrientation().oy,
                 mInitialPoseSl.getOrientation().oz, mInitialPoseSl.getOrientation().ow);

        if (mOdometryDb != "" && !sl_tools::file_exist(mOdometryDb)) {
            mOdometryDb = "";
            NODELET_WARN("odometry_DB path doesn't exist or is unreachable.");
        }

        // Tracking parameters
        sl::TrackingParameters trackParams;

        trackParams.area_file_path = mOdometryDb.c_str();

        mPoseSmoothing = false; // Always false. Pose Smoothing is to be enabled only for VR/AR applications
        trackParams.enable_pose_smoothing = mPoseSmoothing;

        trackParams.enable_spatial_memory = mSpatialMemory;
        trackParams.enable_imu_fusion = mImuFusion;
        trackParams.initial_world_transform = mInitialPoseSl;

#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=6))
        trackParams.set_floor_as_origin = mFloorAlignment;
#else

        if (mFloorAlignment)  {
            NODELET_WARN("Floor Alignment is available with ZED SDK v2.6 or newer");
        }

#endif

        sl::ERROR_CODE err = mZed.enableTracking(trackParams);

        if (err == sl::SUCCESS) {
            mTrackingActivated = true;
        } else {
            mTrackingActivated = false;

            ROS_WARN("Tracking not activated: %s", sl::toString(err).c_str());
        }
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
        if (!mFixedCov && mPublishPoseCovariance) {
            for (size_t i = 0; i < odom.pose.covariance.size(); i++) {
                odom.pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]);

                if (mTwoDMode) {
                    if (i == 14 || i == 21 || i == 28) {
                        odom.pose.covariance[i] = 1e-9;    // Very low covariance if 2D mode
                    } else if ((i >= 2 && i <= 4) ||
                               (i >= 8 && i <= 10) ||
                               (i >= 12 && i <= 13) ||
                               (i >= 15 && i <= 16) ||
                               (i >= 18 && i <= 20) ||
                               (i == 22) ||
                               (i >= 24 && i <= 27)) {
                        odom.pose.covariance[i] = 0.0;
                    }
                }
            }
        } else {
            for (size_t i = 0; i < odom.pose.covariance.size(); i += 7) {
                odom.pose.covariance[i] = mFixedCovValue;
            }
        }

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
        header.frame_id = mWorldFrameId;
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
                if (!mFixedCov) {
                    for (size_t i = 0; i < poseCov.pose.covariance.size(); i++) {
                        poseCov.pose.covariance[i] = static_cast<double>(mLastZedPose.pose_covariance[i]);

                        if (mTwoDMode) {
                            if (i == 14 || i == 21 || i == 28) {
                                poseCov.pose.covariance[i] = 1e-9;    // Very low covariance if 2D mode
                            } else if ((i >= 2 && i <= 4) ||
                                       (i >= 8 && i <= 10) ||
                                       (i >= 12 && i <= 13) ||
                                       (i >= 15 && i <= 16) ||
                                       (i >= 18 && i <= 20) ||
                                       (i == 22) ||
                                       (i >= 24 && i <= 27)) {
                                poseCov.pose.covariance[i] = 0.0;
                            }
                        }
                    }
                } else {
                    for (size_t i = 0; i < poseCov.pose.covariance.size(); i += 7) {
                        poseCov.pose.covariance[i] = mFixedCovValue;
                    }
                }

                // Publish pose with covariance stamped message
                mPubPoseCov.publish(poseCov);
            }
        }
    }

    void ZEDWrapperNodelet::publishOdomFrame(tf2::Transform odomTransf, ros::Time t) {
        if (!mSensor2BaseTransfValid) {
            getSens2BaseTransform();
        }

        if (!mSensor2CameraTransfValid) {
            getSens2CameraTransform();
        }

        if (!mCamera2BaseTransfValid) {
            getCamera2BaseTransform();
        }

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
        if (!mSensor2BaseTransfValid) {
            getSens2BaseTransform();
        }

        if (!mSensor2CameraTransfValid) {
            getSens2CameraTransform();
        }

        if (!mCamera2BaseTransfValid) {
            getCamera2BaseTransform();
        }

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
        if (!mSensor2BaseTransfValid) {
            getSens2BaseTransform();
        }

        if (!mSensor2CameraTransfValid) {
            getSens2CameraTransform();
        }

        if (!mCamera2BaseTransfValid) {
            getCamera2BaseTransform();
        }

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
                                         image_transport::CameraPublisher& pubImg, sensor_msgs::CameraInfoPtr camInfoMsg,
                                         string imgFrameId, ros::Time t) {
        camInfoMsg->header.stamp = t;
        pubImg.publish(sl_tools::imageToROSmsg(img, imgFrameId, t), camInfoMsg);
    }

    void ZEDWrapperNodelet::publishDepth(sl::Mat depth, ros::Time t) {

        mDepthCamInfoMsg->header.stamp = t;

        if (!mOpenniDepthMode) {
            mPubDepth.publish(*sl_tools::imageToROSmsg(depth, mDepthOptFrameId, t), *mDepthCamInfoMsg, t);
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

        mPubDepth.publish(depthMessage, mDepthCamInfoMsg);
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

            // ----> Check publishing frequency
            double pc_period_msec = 1000.0 / mPointCloudFreq;

            static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

            double elapsed_msec = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

            if (elapsed_msec < pc_period_msec) {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long int>(pc_period_msec - elapsed_msec)));
            }

            // <---- Check publishing frequency

            last_time = std::chrono::steady_clock::now();
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

    void ZEDWrapperNodelet::pubFusedPointCloudCallback(const ros::TimerEvent& e) {

#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )
        uint32_t fusedCloudSubnumber = mPubFusedCloud.getNumSubscribers();

        if (fusedCloudSubnumber == 0) {
            return;
        }

        std::lock_guard<std::mutex> lock(mCloseZedMutex);

        if (!mZed.isOpened()) {
            return;
        }

        mPointcloudFusedMsg->header.stamp = mFrameTimestamp;
        mZed.requestSpatialMapAsync();

        while (mZed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE_FAILURE) {
            //Mesh is still generating
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        sl::ERROR_CODE res = mZed.retrieveSpatialMapAsync(mFusedPC);

        if (res != sl::SUCCESS) {
            ROS_WARN_STREAM("Fused point cloud not extracted: " << sl::toString(res).c_str());
            return;
        }

        size_t ptsCount = mFusedPC.getNumberOfPoints();
        bool resized = false;

        if (mPointcloudFusedMsg->width != ptsCount || mPointcloudFusedMsg->height != 1) {
            // Initialize Point Cloud message
            // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
            mPointcloudFusedMsg->header.frame_id = mWorldFrameId; // Set the header values of the ROS message

            mPointcloudFusedMsg->is_bigendian = false;
            mPointcloudFusedMsg->is_dense = false;

            mPointcloudFusedMsg->width = ptsCount;
            mPointcloudFusedMsg->height = 1;

            sensor_msgs::PointCloud2Modifier modifier(*mPointcloudFusedMsg);
            modifier.setPointCloud2Fields(4,
                                          "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "rgb", 1, sensor_msgs::PointField::FLOAT32);

            resized = true;
        }

        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

        //ROS_INFO_STREAM("Chunks: " << mFusedPC.chunks.size());

        int index = 0;
        float* ptCloudPtr = (float*)(&mPointcloudFusedMsg->data[0]);
        int updated = 0;

        for (int c = 0; c < mFusedPC.chunks.size(); c++) {
            if (mFusedPC.chunks[c].has_been_updated || resized) {
                updated++;

                size_t chunkSize = mFusedPC.chunks[c].vertices.size();

                if (chunkSize > 0) {

                    float* cloud_pts = (float*)(mFusedPC.chunks[c].vertices.data());

                    memcpy(ptCloudPtr, cloud_pts, 4 * chunkSize * sizeof(float));

                    ptCloudPtr += 4 * chunkSize;
                }

            } else {
                index += mFusedPC.chunks[c].vertices.size();
            }
        }

        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

        //ROS_INFO_STREAM("Updated: " << updated);


        double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();


        //        ROS_INFO_STREAM("Data copy: " << elapsed_usec << " usec [" << ptsCount << "] - " << (static_cast<double>
        //                        (ptsCount) / elapsed_usec) << " pts/usec");

        // Pointcloud publishing
        mPubFusedCloud.publish(mPointcloudFusedMsg);
#endif
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

        case 1:
            mCamConfidence = config.confidence;
            NODELET_INFO("Reconfigure confidence : %d", mCamConfidence);
            break;

        case 2:
            mCamMaxDepth = config.max_depth;
            NODELET_INFO("Reconfigure max depth : %g", mCamMaxDepth);
            break;

        case 3:
            mPointCloudFreq = config.point_cloud_freq;
            NODELET_INFO("Reconfigure point cloud frequency : %g", mPointCloudFreq);
            break;

        case 4:
            mCamAutoExposure = config.auto_exposure;

            if (mCamAutoExposure) {
                mTriggerAutoExposure = true;
            }

            NODELET_INFO("Reconfigure auto control of exposure and gain : %s",
                         mCamAutoExposure ? "Enable" : "Disable");
            break;

        case 5:
            mCamGain = config.gain;
            NODELET_INFO("Reconfigure gain : %d", mCamGain);
            break;

        case 6:
            mCamExposure = config.exposure;
            NODELET_INFO("Reconfigure exposure : %d", mCamExposure);
            break;
        }
    }

    void ZEDWrapperNodelet::pathPubCallback(const ros::TimerEvent& e) {
        uint32_t mapPathSub = mPubMapPath.getNumSubscribers();
        uint32_t odomPathSub = mPubOdomPath.getNumSubscribers();

        geometry_msgs::PoseStamped odomPose;
        geometry_msgs::PoseStamped mapPose;

        odomPose.header.stamp = mFrameTimestamp;
        odomPose.header.frame_id = mWorldFrameId; // frame
        // conversion from Tranform to message
        geometry_msgs::Transform base2odom = tf2::toMsg(mOdom2BaseTransf);
        // Add all value in Pose message
        odomPose.pose.position.x = base2odom.translation.x;
        odomPose.pose.position.y = base2odom.translation.y;
        odomPose.pose.position.z = base2odom.translation.z;
        odomPose.pose.orientation.x = base2odom.rotation.x;
        odomPose.pose.orientation.y = base2odom.rotation.y;
        odomPose.pose.orientation.z = base2odom.rotation.z;
        odomPose.pose.orientation.w = base2odom.rotation.w;

        mapPose.header.stamp = mFrameTimestamp;
        mapPose.header.frame_id = mWorldFrameId; // frame
        // conversion from Tranform to message
        geometry_msgs::Transform base2map = tf2::toMsg(mMap2BaseTransf);
        // Add all value in Pose message
        mapPose.pose.position.x = base2map.translation.x;
        mapPose.pose.position.y = base2map.translation.y;
        mapPose.pose.position.z = base2map.translation.z;
        mapPose.pose.orientation.x = base2map.rotation.x;
        mapPose.pose.orientation.y = base2map.rotation.y;
        mapPose.pose.orientation.z = base2map.rotation.z;
        mapPose.pose.orientation.w = base2map.rotation.w;

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

        if (mapPathSub > 0) {
            nav_msgs::Path mapPath;
            mapPath.header.frame_id = mWorldFrameId;
            mapPath.header.stamp = mFrameTimestamp;
            mapPath.poses = mMapPath;

            mPubMapPath.publish(mapPath);
        }

        if (odomPathSub > 0) {
            nav_msgs::Path odomPath;
            odomPath.header.frame_id = mWorldFrameId;
            odomPath.header.stamp = mFrameTimestamp;
            odomPath.poses = mOdomPath;

            mPubOdomPath.publish(odomPath);
        }
    }

    void ZEDWrapperNodelet::imuPubCallback(const ros::TimerEvent& e) {

        if (mStreaming) {
            return;
        }

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

        mRecording = false;

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
        mMappingActivated = false;
        mRecording = false;

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

        // the reference camera is the Left one (next to the ZED logo)

        mRgbCamInfoMsg = mDepthCamInfoMsg = mLeftCamInfoMsg;
        mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;

        sl::RuntimeParameters runParams;
        runParams.sensing_mode = static_cast<sl::SENSING_MODE>(mCamSensingMode);
        sl::Mat leftZEDMat, rightZEDMat, depthZEDMat, disparityZEDMat, confImgZEDMat, confMapZEDMat;

        // Main loop
        while (mNhNs.ok()) {
            std::chrono::steady_clock::time_point start_elab = std::chrono::steady_clock::now();

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
            uint32_t fusedCloudSubnumber = mPubFusedCloud.getNumSubscribers();
            uint32_t poseSubnumber = mPubPose.getNumSubscribers();
            uint32_t poseCovSubnumber = mPubPoseCov.getNumSubscribers();
            uint32_t odomSubnumber = mPubOdom.getNumSubscribers();
            uint32_t confImgSubnumber = mPubConfImg.getNumSubscribers();
            uint32_t confMapSubnumber = mPubConfMap.getNumSubscribers();
            //            uint32_t imuSubnumber = mPubImu.getNumSubscribers();
            //            uint32_t imuRawsubnumber = mPubImuRaw.getNumSubscribers();
            uint32_t pathSubNumber = mPubMapPath.getNumSubscribers() + mPubOdomPath.getNumSubscribers();
            uint32_t stereoSubNumber = mPubStereo.getNumSubscribers();
            uint32_t stereoRawSubNumber = mPubRawStereo.getNumSubscribers();

            mGrabActive =  mRecording || mStreaming || mMappingEnabled || mTrackingActivated ||
                           ((rgbSubnumber + rgbRawSubnumber + leftSubnumber +
                             leftRawSubnumber + rightSubnumber + rightRawSubnumber +
                             depthSubnumber + disparitySubnumber + cloudSubnumber +
                             poseSubnumber + poseCovSubnumber + odomSubnumber + confImgSubnumber +
                             confMapSubnumber /*+ imuSubnumber + imuRawsubnumber*/ + pathSubNumber +
                             stereoSubNumber + stereoRawSubNumber) > 0);

            runParams.enable_point_cloud = false;

            // Run the loop only if there is some subscribers or SVO is active
            if (mGrabActive) {
                std::lock_guard<std::mutex> lock(mPosTrkMutex);

                // Note: one tracking is started is never stopped anymore
                bool computeTracking = (mMappingEnabled || (mComputeDepth & mDepthStabilization) || poseSubnumber > 0 ||
                                        poseCovSubnumber > 0 || odomSubnumber > 0 || pathSubNumber > 0);

                // Start the tracking?
                if ((computeTracking) && !mTrackingActivated && (mCamQuality != sl::DEPTH_MODE_NONE)) {
                    start_tracking();
                }

                // Start the mapping?
                if (mMappingEnabled && !mMappingActivated) {
                    start_mapping();
                }

                // Detect if one of the subscriber need to have the depth information
                mComputeDepth = mCamQuality != sl::DEPTH_MODE_NONE &&
                                ((depthSubnumber + disparitySubnumber + cloudSubnumber + fusedCloudSubnumber +
                                  poseSubnumber + poseCovSubnumber + odomSubnumber + confImgSubnumber +
                                  confMapSubnumber) > 0);

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
                    runParams.enable_depth = false; // Ask to not compute the depth
                }

                mGrabStatus = mZed.grab(runParams);

                // cout << toString(grab_status) << endl;
                if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
                    // Detect if a error occurred (for example:
                    // the zed have been disconnected) and
                    // re-initialize the ZED
                    if (mGrabStatus != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                        NODELET_INFO_STREAM_ONCE(toString(mGrabStatus));
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));

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

                        computeTracking = mDepthStabilization || poseSubnumber > 0 || poseCovSubnumber > 0 ||
                                          odomSubnumber > 0;

                        if (computeTracking) {  // Start the tracking
                            start_tracking();
                        }
                    }

                    mDiagUpdater.update();

                    continue;
                }

                // SVO recording
                mRecMutex.lock();

                if (mRecording) {
                    mRecState = mZed.record();

                    if (!mRecState.status) {
                        ROS_ERROR_THROTTLE(1.0, "Error saving frame to SVO");
                    }

                    mDiagUpdater.force_update();
                }

                mRecMutex.unlock();

                // Timestamp
                mPrevFrameTimestamp = mFrameTimestamp;

                // Publish freq calculation
                static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
                std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

                double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
                last_time = now;

                mGrabPeriodMean_usec->addValue(elapsed_usec);

                //ROS_INFO_STREAM("Grab time: " << elapsed_usec / 1000 << " msec");

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
                        publishImage(leftZEDMat, mPubLeft, mLeftCamInfoMsg, mLeftCamOptFrameId, mFrameTimestamp);
                    }

                    if (rgbSubnumber > 0) {
                        publishImage(leftZEDMat, mPubRgb, mRgbCamInfoMsg, mDepthOptFrameId, mFrameTimestamp); // rgb is the left image
                    }
                }

                // Publish the left_raw == rgb_raw image if someone has subscribed to
                if (leftRawSubnumber > 0 || rgbRawSubnumber > 0) {

                    // Retrieve RGBA Left image
                    mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);

                    if (leftRawSubnumber > 0) {
                        publishImage(leftZEDMat, mPubRawLeft, mLeftCamInfoRawMsg, mLeftCamOptFrameId, mFrameTimestamp);
                    }

                    if (rgbRawSubnumber > 0) {
                        publishImage(leftZEDMat, mPubRawRgb, mRgbCamInfoRawMsg, mDepthOptFrameId, mFrameTimestamp);
                    }
                }

                // Publish the right image if someone has subscribed to
                if (rightSubnumber > 0) {

                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT, sl::MEM_CPU, mMatWidth, mMatHeight);

                    publishImage(rightZEDMat, mPubRight, mRightCamInfoMsg, mRightCamOptFrameId, mFrameTimestamp);
                }

                // Publish the right image if someone has subscribed to
                if (rightRawSubnumber > 0) {

                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);

                    publishImage(rightZEDMat, mPubRawRight, mRightCamInfoRawMsg, mRightCamOptFrameId, mFrameTimestamp);
                }

                // Stereo couple side-by-side
                if (stereoSubNumber > 0) {

                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT, sl::MEM_CPU, mMatWidth, mMatHeight);
                    mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT, sl::MEM_CPU, mMatWidth, mMatHeight);

                    mPubStereo.publish(sl_tools::imagesToROSmsg(leftZEDMat, rightZEDMat, mCameraFrameId, mFrameTimestamp));
                }

                // Stereo RAW couple side-by-side
                if (stereoRawSubNumber > 0) {

                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);
                    mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED, sl::MEM_CPU, mMatWidth, mMatHeight);

                    mPubRawStereo.publish(sl_tools::imagesToROSmsg(leftZEDMat, rightZEDMat, mCameraFrameId, mFrameTimestamp));
                }

                // Publish the depth image if someone has subscribed to
                if (depthSubnumber > 0 || disparitySubnumber > 0) {

                    mZed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH, sl::MEM_CPU, mMatWidth, mMatHeight);
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
                    publishImage(confImgZEDMat, mPubConfImg, mDepthCamInfoMsg, mConfidenceOptFrameId, mFrameTimestamp);
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
                if (computeTracking) {

                    if (!mSensor2BaseTransfValid) {
                        getSens2BaseTransform();
                    }

                    if (!mSensor2CameraTransfValid) {
                        getSens2CameraTransform();
                    }

                    if (!mCamera2BaseTransfValid) {
                        getCamera2BaseTransform();
                    }

                    if (!mInitOdomWithPose) {
                        sl::Pose deltaOdom;
                        mTrackingStatus = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME_CAMERA);

                        sl::Translation translation = deltaOdom.getTranslation();
                        sl::Orientation quat = deltaOdom.getOrientation();

#if 0
                        NODELET_DEBUG("delta ODOM [%s] - %.2f,%.2f,%.2f %.2f,%.2f,%.2f,%.2f",
                                      sl::toString(mTrackingStatus).c_str(),
                                      translation(mIdxX), translation(mIdxY), translation(mIdxZ),
                                      quat(mIdxX), quat(mIdxY), quat(mIdxZ), quat(3));

                        NODELET_DEBUG_STREAM("ODOM -> Tracking Status: " << sl::toString(mTrackingStatus));
#endif

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

                            if (mTwoDMode) {
                                tf2::Vector3 tr_2d = mOdom2BaseTransf.getOrigin();
                                tr_2d.setZ(mFixedZValue);
                                mOdom2BaseTransf.setOrigin(tr_2d);

                                double roll, pitch, yaw;
                                tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                                tf2::Quaternion quat_2d;
                                quat_2d.setRPY(0.0, 0.0, yaw);

                                mOdom2BaseTransf.setRotation(quat_2d);
                            }

#if 0 //#ifndef NDEBUG // Enable for TF checking
                            double roll, pitch, yaw;
                            tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                            NODELET_DEBUG("+++ Odometry [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                          mOdometryFrameId.c_str(), mBaseFrameId.c_str(),
                                          mOdom2BaseTransf.getOrigin().x(), mOdom2BaseTransf.getOrigin().y(), mOdom2BaseTransf.getOrigin().z(),
                                          roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

                            // Publish odometry message
                            if (odomSubnumber > 0) {
                                publishOdom(mOdom2BaseTransf, deltaOdom, mFrameTimestamp);
                            }

                            mTrackingReady = true;
                        }
                    } else if (mFloorAlignment) {
                        NODELET_WARN_THROTTLE(5.0, "Odometry will be published as soon as the floor as been detected for the first time");
                    }

                }

                // Publish the zed camera pose if someone has subscribed to
                if (computeTracking) {
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

                    NODELET_DEBUG_STREAM("MAP -> Tracking Status: " << sl::toString(mTrackingStatus));

#endif

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

                        if (mTwoDMode) {
                            tf2::Vector3 tr_2d = mMap2BaseTransf.getOrigin();
                            tr_2d.setZ(mFixedZValue);
                            mMap2BaseTransf.setOrigin(tr_2d);

                            double roll, pitch, yaw;
                            tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                            tf2::Quaternion quat_2d;
                            quat_2d.setRPY(0.0, 0.0, yaw);

                            mMap2BaseTransf.setRotation(quat_2d);
                        }

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
                        if ((poseSubnumber + poseCovSubnumber) > 0) {
                            publishPose(mFrameTimestamp);
                        }

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
                std::chrono::steady_clock::time_point end_elab = std::chrono::steady_clock::now();

                double elab_usec = std::chrono::duration_cast<std::chrono::microseconds>(end_elab - start_elab).count();

                double mean_elab_sec = mElabPeriodMean_sec->addValue(elab_usec / 1000000.);

                static int count_warn = 0;

                if (!loop_rate.sleep()) {
                    if (mean_elab_sec > (1. / mCamFrameRate)) {
                        if (++count_warn > 10) {
                            NODELET_DEBUG_THROTTLE(
                                1.0,
                                "Working thread is not synchronized with the Camera frame rate");
                            NODELET_DEBUG_STREAM_THROTTLE(
                                1.0, "Expected cycle time: " << loop_rate.expectedCycleTime()
                                << " - Real cycle time: "
                                << mean_elab_sec);
                            NODELET_WARN_STREAM_THROTTLE(10.0, "Elaboration takes longer (" << mean_elab_sec << " sec) than requested "
                                                         "by the FPS rate (" << loop_rate.expectedCycleTime() << " sec). Please consider to "
                                                         "lower the 'frame_rate' setting or to reduce the power requirements reducing the resolutions.");
                        }

                        loop_rate.reset();
                    } else {
                        count_warn = 0;
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

                    stat.addf("Processing Time", "Mean time: %.3f sec (Max. %.3f sec)", mElabPeriodMean_sec->getMean(), 1. / mCamFrameRate);

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

            if (mRecording) {
                if (!mRecState.status) {
                    if (mGrabActive) {
                        stat.add("SVO Recording", "ERROR");
                        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                                     "Error adding frames to SVO file while recording. Check free disk space");
                    } else {
                        stat.add("SVO Recording", "WAITING");
                    }
                } else {
                    stat.add("SVO Recording", "ACTIVE");
                    stat.addf("SVO compression time", "%g msec", mRecState.average_compression_time);
                    stat.addf("SVO compression ratio", "%.1f%%", mRecState.average_compression_ratio);
                }
            } else {
                stat.add("SVO Recording", "NOT ACTIVE");
            }
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, sl::toString(mConnStatus).c_str());
        }
    }

    bool ZEDWrapperNodelet::on_start_svo_recording(zed_wrapper::start_svo_recording::Request& req,
            zed_wrapper::start_svo_recording::Response& res) {
        std::lock_guard<std::mutex> lock(mRecMutex);

        if (mRecording) {
            res.result = false;
            res.info = "Recording was just active";
            return false;
        }

        // Check filename
        if (req.svo_filename.empty()) {
            req.svo_filename = "zed.svo";
        }

        sl::ERROR_CODE err;
        sl::SVO_COMPRESSION_MODE compression = mSvoComprMode;

        err = mZed.enableRecording(req.svo_filename.c_str(), mSvoComprMode);

        if (err == sl::ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION) {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=7))
            compression = mSvoComprMode == sl::SVO_COMPRESSION_MODE_HEVC ? sl::SVO_COMPRESSION_MODE_AVCHD :
                          sl::SVO_COMPRESSION_MODE_HEVC;

            ROS_WARN_STREAM("The chosen " << sl::toString(mSvoComprMode).c_str() << "mode is not available. Trying " <<
                            sl::toString(compression).c_str());

            err = mZed.enableRecording(req.svo_filename.c_str(), compression);

            if (err == sl::ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION) {
                ROS_WARN_STREAM(sl::toString(compression).c_str() << "not available. Trying " << sl::toString(
                                    sl::SVO_COMPRESSION_MODE_LOSSY).c_str());
                compression = sl::SVO_COMPRESSION_MODE_LOSSY;
                err = mZed.enableRecording(req.svo_filename.c_str(), compression);  // JPEG Compression?


#else
            compression = mSvoComprMode == sl::SVO_COMPRESSION_MODE_LOSSY ? sl::SVO_COMPRESSION_MODE_LOSSLESS :
                          sl::SVO_COMPRESSION_MODE_LOSSY;

            ROS_WARN_STREAM("The chosen " << sl::toString(mSvoComprMode).c_str() << "mode is not available. Trying " <<
                            sl::toString(compression).c_str());


            err = mZed.enableRecording(req.svo_filename.c_str(), compression);
#endif

                if (err == sl::ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION) {
                    compression = sl::SVO_COMPRESSION_MODE_RAW;
                    err = mZed.enableRecording(req.svo_filename.c_str(), compression);
                }
            }
        }

        if (err != sl::SUCCESS) {
            res.result = false;
            res.info = sl::toString(err).c_str();
            mRecording = false;
            return false;
        }

        mSvoComprMode = compression;
        mRecording = true;
        res.info = "Recording started (";
        res.info += sl::toString(compression).c_str();
        res.info += ")";
        res.result = true;

        ROS_INFO_STREAM("SVO recording STARTED: " << req.svo_filename << " (" << sl::toString(compression).c_str() << ")");

        return true;
    }

    bool ZEDWrapperNodelet::on_stop_svo_recording(zed_wrapper::stop_svo_recording::Request& req,
            zed_wrapper::stop_svo_recording::Response& res) {
        std::lock_guard<std::mutex> lock(mRecMutex);

        if (!mRecording) {
            res.done = false;
            res.info = "Recording was not active";
            return false;
        }

        mZed.disableRecording();
        mRecording = false;
        res.info = "Recording stopped";
        res.done = true;

        ROS_INFO_STREAM("SVO recording STOPPED");

        return true;
    }

    bool ZEDWrapperNodelet::on_start_remote_stream(zed_wrapper::start_remote_stream::Request& req,
            zed_wrapper::start_remote_stream::Response& res) {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )

        if (mStreaming) {
            res.result = false;
            res.info = "SVO remote streaming was just active";
            return false;
        }

        sl::StreamingParameters params;
        params.codec = static_cast<sl::STREAMING_CODEC>(req.codec);
        params.port = req.port;
        params.bitrate = req.bitrate;
        params.gop_size = req.gop_size;
        params.adaptative_bitrate = req.adaptative_bitrate;

        if (params.gop_size < -1 || params.gop_size > 256) {

            mStreaming = false;

            res.result = false;
            res.info = "`gop_size` wrong (";
            res.info += params.gop_size;
            res.info += "). Remote streaming not started";

            ROS_ERROR_STREAM(res.info);
            return false;
        }

        if (params.port % 2 != 0) {
            mStreaming = false;

            res.result = false;
            res.info = "`port` must be an even number. Remote streaming not started";

            ROS_ERROR_STREAM(res.info);
            return false;
        }

        sl::ERROR_CODE err = mZed.enableStreaming(params);

        if (err != sl::SUCCESS) {
            mStreaming = false;

            res.result = false;
            res.info = sl::toString(err).c_str();

            ROS_ERROR_STREAM("Remote streaming not started (" << res.info << ")");

            return false;
        }

        mStreaming = true;

        ROS_INFO_STREAM("Remote streaming STARTED");

        res.result = true;
        res.info = "Remote streaming STARTED";
        return true;
#else
        ROS_WARN("Remote streaming requires the ZED SDK v2.8 or newer");

        res.result = false;
        res.info = "Remote streaming requires the ZED SDK v2.8 or newer";
        return true;
#endif
    }

    bool ZEDWrapperNodelet::on_stop_remote_stream(zed_wrapper::stop_remote_stream::Request& req,
            zed_wrapper::stop_remote_stream::Response& res) {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )

        if (mStreaming) {
            mZed.disableStreaming();
        }

        mStreaming = false;

        ROS_INFO_STREAM("SVO remote streaming STOPPED");
#endif
        res.done = true;

        return true;
    }

    bool ZEDWrapperNodelet::on_set_led_status(zed_wrapper::set_led_status::Request& req,
            zed_wrapper::set_led_status::Response& res) {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )

        if (mFwVersion < 1523) {
            ROS_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
            return false;
        }

        mZed.setCameraSettings(sl::CAMERA_SETTINGS_LED_STATUS, req.led_enabled ? 1 : 0);

        return true;
#else
        ROS_WARN("LED control requires the ZED SDK v2.8 or newer");
        return false;
#endif
    }

    bool ZEDWrapperNodelet::on_toggle_led(zed_wrapper::toggle_led::Request& req,
                                          zed_wrapper::toggle_led::Response& res) {
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )

        if (mFwVersion < 1523) {
            ROS_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
            return false;
        }

        int status = mZed.getCameraSettings(sl::CAMERA_SETTINGS_LED_STATUS);
        int new_status = status == 0 ? 1 : 0;
        mZed.setCameraSettings(sl::CAMERA_SETTINGS_LED_STATUS, new_status);

        return (new_status == 1);
#else
        ROS_WARN("LED control requires the ZED SDK v2.8 or newer");
        return false;
#endif
    }
} // namespace
