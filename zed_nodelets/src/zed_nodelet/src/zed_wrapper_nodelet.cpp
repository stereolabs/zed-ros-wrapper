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

#include <chrono>
#include <csignal>
#include <sstream>

#include "zed_wrapper_nodelet.hpp"

#include "zed_wrapper_nodelet.hpp"

#ifndef NDEBUG
#include <ros/console.h>
#endif

#include "zed_interfaces/Object.h"
#include "zed_interfaces/ObjectsStamped.h"
#include <zed_interfaces/PlaneStamped.h>

//#define DEBUG_SENS_TS 1

namespace zed_nodelets {
#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

#define MAG_FREQ 50.
#define BARO_FREQ 25.

ZEDWrapperNodelet::ZEDWrapperNodelet()
    : Nodelet()
{
}

ZEDWrapperNodelet::~ZEDWrapperNodelet()
{
    if (mDevicePollThread.joinable()) {
        mDevicePollThread.join();
    }

    if (mPcThread.joinable()) {
        mPcThread.join();
    }

    if (mSensThread.joinable()) {
        mSensThread.join();
    }

    std::cerr << "ZED Nodelet destroyed" << std::endl;
}

void ZEDWrapperNodelet::onInit()
{
    // Node handlers
    mNh = getMTNodeHandle();
    mNhNs = getMTPrivateNodeHandle();

    mStopNode = false;
    mPcDataReady = false;
    mRgbDepthDataRetrieved = true;

#ifndef NDEBUG

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

#endif

    NODELET_INFO("********** Starting nodelet '%s' **********", getName().c_str());

    std::string ver = sl_tools::getSDKVersion(mVerMajor, mVerMinor, mVerSubMinor);
    NODELET_INFO_STREAM("SDK version : " << ver);

    if (mVerMajor < 3) {
        NODELET_ERROR("This version of the ZED ROS Wrapper is designed to be used with ZED SDK v3.x");
        ros::shutdown();
        raise(SIGINT);
        raise(SIGABRT);
        exit(-1);
    }

    readParameters();

    initTransforms();

    // Set the video topic names
    std::string rgbTopicRoot = "rgb";
    std::string rightTopicRoot = "right";
    std::string leftTopicRoot = "left";
    std::string stereoTopicRoot = "stereo";
    std::string img_topic = "/image_rect_color";
    std::string img_raw_topic = "/image_raw_color";
    std::string img_gray_topic = "/image_rect_gray";
    std::string img_raw_gray_topic_ = "/image_raw_gray";
    std::string raw_suffix = "_raw";
    std::string left_topic = leftTopicRoot + img_topic;
    std::string left_raw_topic = leftTopicRoot + raw_suffix + img_raw_topic;
    std::string right_topic = rightTopicRoot + img_topic;
    std::string right_raw_topic = rightTopicRoot + raw_suffix + img_raw_topic;
    std::string rgb_topic = rgbTopicRoot + img_topic;
    std::string rgb_raw_topic = rgbTopicRoot + raw_suffix + img_raw_topic;
    std::string stereo_topic = stereoTopicRoot + img_topic;
    std::string stereo_raw_topic = stereoTopicRoot + raw_suffix + img_raw_topic;
    std::string left_gray_topic = leftTopicRoot + img_gray_topic;
    std::string left_raw_gray_topic = leftTopicRoot + raw_suffix + img_raw_gray_topic_;
    std::string right_gray_topic = rightTopicRoot + img_gray_topic;
    std::string right_raw_gray_topic = rightTopicRoot + raw_suffix + img_raw_gray_topic_;
    std::string rgb_gray_topic = rgbTopicRoot + img_gray_topic;
    std::string rgb_raw_gray_topic = rgbTopicRoot + raw_suffix + img_raw_gray_topic_;

    // Set the disparity topic name
    std::string disparityTopic = "disparity/disparity_image";

    // Set the depth topic names
    std::string depth_topic_root = "depth";

    if (mOpenniDepthMode) {
        NODELET_INFO_STREAM("Openni depth mode activated -> Units: mm, Encoding: TYPE_16UC1");
    }
    depth_topic_root += "/depth_registered";

    std::string pointcloud_topic = "point_cloud/cloud_registered";

    std::string pointcloud_fused_topic = "mapping/fused_cloud";

    std::string object_det_topic_root = "obj_det";
    std::string object_det_topic = object_det_topic_root + "/objects";

    std::string confImgRoot = "confidence";
    std::string conf_map_topic_name = "confidence_map";
    std::string conf_map_topic = confImgRoot + "/" + conf_map_topic_name;

    // Set the positional tracking topic names
    std::string poseTopic = "pose";
    std::string pose_cov_topic;
    pose_cov_topic = poseTopic + "_with_covariance";

    std::string odometryTopic = "odom";
    std::string odom_path_topic = "path_odom";
    std::string map_path_topic = "path_map";

    // Extracted plane topics
    std::string marker_topic = "plane_marker";
    std::string plane_topic = "plane";

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
            mZedParams.input.setFromSVOFile(mSvoFilepath.c_str());
            mZedParams.svo_real_time_mode = true;

            // TODO ADD PARAMETER FOR SVO REAL TIME
        } else if (!mRemoteStreamAddr.empty()) {
            std::vector<std::string> configStream = sl_tools::split_string(mRemoteStreamAddr, ':');
            sl::String ip = sl::String(configStream.at(0).c_str());

            if (configStream.size() == 2) {
                mZedParams.input.setFromStream(ip, atoi(configStream.at(1).c_str()));
            } else {
                mZedParams.input.setFromStream(ip);
            }
        }

        mSvoMode = true;
    } else {
        mZedParams.camera_fps = mCamFrameRate;
        mZedParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);

        if (mZedSerialNumber == 0) {
            mZedParams.input.setFromCameraID(mZedId);
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

                if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::NOT_AVAILABLE) {
                    std::string msg = "ZED SN" + std::to_string(mZedSerialNumber) + " not detected ! Please connect this ZED";
                    NODELET_INFO_STREAM(msg.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                } else {
                    waiting_for_camera = false;
                    mZedParams.input.setFromCameraID(prop.id);
                }
            }
        }
    }

    mZedParams.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    NODELET_INFO_STREAM(" * Camera coordinate system\t-> " << sl::toString(mZedParams.coordinate_system));

    mZedParams.coordinate_units = sl::UNIT::METER;
    mZedParams.depth_mode = static_cast<sl::DEPTH_MODE>(mDepthMode);
    mZedParams.sdk_verbose = mVerbose;
    mZedParams.sdk_gpu_id = mGpuId;
    mZedParams.depth_stabilization = mDepthStabilization;
    mZedParams.camera_image_flip = mCameraFlip;
    mZedParams.depth_minimum_distance = static_cast<float>(mCamMinDepth);
    mZedParams.depth_maximum_distance = static_cast<float>(mCamMaxDepth);
    mZedParams.camera_disable_self_calib = !mCameraSelfCalib;

    mZedParams.enable_image_enhancement = true; // Always active

    mDiagUpdater.add("ZED Diagnostic", this, &ZEDWrapperNodelet::callback_updateDiagnostic);
    mDiagUpdater.setHardwareID("ZED camera");

    mConnStatus = sl::ERROR_CODE::CAMERA_NOT_DETECTED;

    NODELET_INFO_STREAM(" *** Opening " << sl::toString(mZedUserCamModel) << "...");
    while (mConnStatus != sl::ERROR_CODE::SUCCESS) {
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
    NODELET_INFO_STREAM(" ...  " << sl::toString(mZedRealCamModel) << " ready");

    // CUdevice devid;
    cuCtxGetDevice(&mGpuId);

    NODELET_INFO_STREAM("ZED SDK running on GPU #" << mGpuId);

    // Disable AEC_AGC and Auto Whitebalance to trigger it if use set to automatic
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);

    mZedRealCamModel = mZed.getCameraInformation().camera_model;

    if (mZedRealCamModel == sl::MODEL::ZED) {
        if (mZedUserCamModel != mZedRealCamModel) {
            NODELET_WARN("Camera model does not match user parameter. Please modify "
                         "the value of the parameter 'camera_model' to 'zed'");
        }
    } else if (mZedRealCamModel == sl::MODEL::ZED_M) {
        if (mZedUserCamModel != mZedRealCamModel) {
            NODELET_WARN("Camera model does not match user parameter. Please modify "
                         "the value of the parameter 'camera_model' to 'zedm'");
        }
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
        mSlCamImuTransf = mZed.getCameraInformation().camera_imu_transform;
#else
        mSlCamImuTransf = mZed.getCameraInformation().sensors_configuration.camera_imu_transform;
#endif

        NODELET_INFO("Camera-IMU Transform: \n %s", mSlCamImuTransf.getInfos().c_str());
    } else if (mZedRealCamModel == sl::MODEL::ZED2) {
        if (mZedUserCamModel != mZedRealCamModel) {
            NODELET_WARN("Camera model does not match user parameter. Please modify "
                         "the value of the parameter 'camera_model' to 'zed2'");
        }

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
        mSlCamImuTransf = mZed.getCameraInformation().camera_imu_transform;
#else
        mSlCamImuTransf = mZed.getCameraInformation().sensors_configuration.camera_imu_transform;
#endif

        NODELET_INFO("Camera-IMU Transform: \n %s", mSlCamImuTransf.getInfos().c_str());
    } else if (mZedRealCamModel == sl::MODEL::ZED2i) {
        if (mZedUserCamModel != mZedRealCamModel) {
            NODELET_WARN("Camera model does not match user parameter. Please modify "
                         "the value of the parameter 'camera_model' to 'zed2i'");
        }

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
        mSlCamImuTransf = mZed.getCameraInformation().camera_imu_transform;
#else
        mSlCamImuTransf = mZed.getCameraInformation().sensors_configuration.camera_imu_transform;
#endif

        NODELET_INFO("Camera-IMU Transform: \n %s", mSlCamImuTransf.getInfos().c_str());
    }

    NODELET_INFO_STREAM(" * CAMERA MODEL\t -> " << sl::toString(mZedRealCamModel).c_str());
    mZedSerialNumber = mZed.getCameraInformation().serial_number;
    NODELET_INFO_STREAM(" * Serial Number -> " << mZedSerialNumber);

    if (!mSvoMode) {
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
        mCamFwVersion = mZed.getCameraInformation().camera_firmware_version;
#else
        mCamFwVersion = mZed.getCameraInformation().camera_configuration.firmware_version;
#endif

        NODELET_INFO_STREAM(" * Camera FW Version -> " << mCamFwVersion);
        if (mZedRealCamModel != sl::MODEL::ZED) {
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
            mSensFwVersion = mZed.getCameraInformation().sensors_firmware_version;
#else
            mSensFwVersion = mZed.getCameraInformation().sensors_configuration.firmware_version;
#endif
            NODELET_INFO_STREAM(" * Sensors FW Version -> " << mSensFwVersion);
        }
    } else {
        NODELET_INFO_STREAM(" * Input type\t -> " << sl::toString(mZed.getCameraInformation().input_type).c_str());
    }

    // Set the IMU topic names using real camera model
    std::string imu_topic;
    std::string imu_topic_raw;
    std::string imu_temp_topic;
    std::string imu_mag_topic;
    // std::string imu_mag_topic_raw;
    std::string pressure_topic;
    std::string temp_topic_root = "temperature";
    std::string temp_topic_left = temp_topic_root + "/left";
    std::string temp_topic_right = temp_topic_root + "/right";

    if (mZedRealCamModel != sl::MODEL::ZED) {
        std::string imuTopicRoot = "imu";
        std::string imu_topic_name = "data";
        std::string imu_topic_raw_name = "data_raw";
        std::string imu_topic_mag_name = "mag";
        // std::string imu_topic_mag_raw_name = "mag_raw";
        std::string pressure_topic_name = "atm_press";
        imu_topic = imuTopicRoot + "/" + imu_topic_name;
        imu_topic_raw = imuTopicRoot + "/" + imu_topic_raw_name;
        imu_temp_topic = temp_topic_root + "/" + imuTopicRoot;
        imu_mag_topic = imuTopicRoot + "/" + imu_topic_mag_name;
        // imu_mag_topic_raw = imuTopicRoot + "/" + imu_topic_mag_raw_name;
        pressure_topic = /*imuTopicRoot + "/" +*/ pressure_topic_name;
    }

    mDiagUpdater.setHardwareIDf("%s - s/n: %d [GPU #%d]", sl::toString(mZedRealCamModel).c_str(), mZedSerialNumber,
        mGpuId);

    // ----> Dynamic Reconfigure parameters
    mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<zed_nodelets::ZedConfig>>(mDynServerMutex);
    dynamic_reconfigure::Server<zed_nodelets::ZedConfig>::CallbackType f;
    f = boost::bind(&ZEDWrapperNodelet::callback_dynamicReconf, this, _1, _2);
    mDynRecServer->setCallback(f);
    // Update parameters
    zed_nodelets::ZedConfig config;
    mDynRecServer->getConfigDefault(config);
    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();

    updateDynamicReconfigure();
    // <---- Dynamic Reconfigure parameters

    // Create all the publishers
    // Image publishers
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

    mPubRgbGray = it_zed.advertiseCamera(rgb_gray_topic, 1); // rgb
    NODELET_INFO_STREAM("Advertised on topic " << mPubRgbGray.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubRgbGray.getInfoTopic());
    mPubRawRgbGray = it_zed.advertiseCamera(rgb_raw_gray_topic, 1); // rgb raw
    NODELET_INFO_STREAM("Advertised on topic " << mPubRawRgbGray.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubRawRgbGray.getInfoTopic());
    mPubLeftGray = it_zed.advertiseCamera(left_gray_topic, 1); // left
    NODELET_INFO_STREAM("Advertised on topic " << mPubLeftGray.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubLeftGray.getInfoTopic());
    mPubRawLeftGray = it_zed.advertiseCamera(left_raw_gray_topic, 1); // left raw
    NODELET_INFO_STREAM("Advertised on topic " << mPubRawLeftGray.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubRawLeftGray.getInfoTopic());
    mPubRightGray = it_zed.advertiseCamera(right_gray_topic, 1); // right
    NODELET_INFO_STREAM("Advertised on topic " << mPubRightGray.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubRightGray.getInfoTopic());
    mPubRawRightGray = it_zed.advertiseCamera(right_raw_gray_topic, 1); // right raw
    NODELET_INFO_STREAM("Advertised on topic " << mPubRawRightGray.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubRawRightGray.getInfoTopic());

    mPubDepth = it_zed.advertiseCamera(depth_topic_root, 1); // depth
    NODELET_INFO_STREAM("Advertised on topic " << mPubDepth.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubDepth.getInfoTopic());

    mPubStereo = it_zed.advertise(stereo_topic, 1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubStereo.getTopic());
    mPubRawStereo = it_zed.advertise(stereo_raw_topic, 1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubRawStereo.getTopic());

    // Confidence Map publisher
    mPubConfMap = mNhNs.advertise<sensor_msgs::Image>(conf_map_topic, 1); // confidence map
    NODELET_INFO_STREAM("Advertised on topic " << mPubConfMap.getTopic());

    // Disparity publisher
    mPubDisparity = mNhNs.advertise<stereo_msgs::DisparityImage>(disparityTopic, static_cast<int>(mVideoDepthFreq));
    NODELET_INFO_STREAM("Advertised on topic " << mPubDisparity.getTopic());

    // PointCloud publishers
    mPubCloud = mNhNs.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubCloud.getTopic());

    if (mMappingEnabled) {
        mPubFusedCloud = mNhNs.advertise<sensor_msgs::PointCloud2>(pointcloud_fused_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubFusedCloud.getTopic() << " @ " << mFusedPcPubFreq << " Hz");
    }

    // Object detection publishers
    if (mObjDetEnabled) {
        mPubObjDet = mNhNs.advertise<zed_interfaces::ObjectsStamped>(object_det_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubObjDet.getTopic());
    }

    // Odometry and Pose publisher
    mPubPose = mNhNs.advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubPose.getTopic());

    mPubPoseCov = mNhNs.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_cov_topic, 1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubPoseCov.getTopic());

    mPubOdom = mNhNs.advertise<nav_msgs::Odometry>(odometryTopic, 1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubOdom.getTopic());

    // Rviz markers publisher
    mPubMarker = mNhNs.advertise<visualization_msgs::Marker>(marker_topic, 10, true);

    // Detected planes publisher
    mPubPlane = mNhNs.advertise<zed_interfaces::PlaneStamped>(plane_topic, 1);

    // Camera Path
    if (mPathPubRate > 0) {
        mPubOdomPath = mNhNs.advertise<nav_msgs::Path>(odom_path_topic, 1, true);
        NODELET_INFO_STREAM("Advertised on topic " << mPubOdomPath.getTopic());
        mPubMapPath = mNhNs.advertise<nav_msgs::Path>(map_path_topic, 1, true);
        NODELET_INFO_STREAM("Advertised on topic " << mPubMapPath.getTopic());

        mPathTimer = mNhNs.createTimer(ros::Duration(1.0 / mPathPubRate), &ZEDWrapperNodelet::callback_pubPath, this);

        if (mPathMaxCount != -1) {
            NODELET_DEBUG_STREAM("Path vectors reserved " << mPathMaxCount << " poses.");
            mOdomPath.reserve(mPathMaxCount);
            mMapPath.reserve(mPathMaxCount);

            NODELET_DEBUG_STREAM("Path vector sizes: " << mOdomPath.size() << " " << mMapPath.size());
        }
    } else {
        NODELET_INFO_STREAM("Path topics not published -> mPathPubRate: " << mPathPubRate);
    }

    // Sensor publishers
    if (mZedRealCamModel != sl::MODEL::ZED) {
        // IMU Publishers
        mPubImu = mNhNs.advertise<sensor_msgs::Imu>(imu_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubImu.getTopic());
        mPubImuRaw = mNhNs.advertise<sensor_msgs::Imu>(imu_topic_raw, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubImuRaw.getTopic());
        mPubImuMag = mNhNs.advertise<sensor_msgs::MagneticField>(imu_mag_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubImuMag.getTopic());

        if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i) {
            // IMU temperature sensor
            mPubImuTemp = mNhNs.advertise<sensor_msgs::Temperature>(imu_temp_topic, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubImuTemp.getTopic());

            // Atmospheric pressure
            mPubPressure = mNhNs.advertise<sensor_msgs::FluidPressure>(pressure_topic, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubPressure.getTopic());

            // CMOS sensor temperatures
            mPubTempL = mNhNs.advertise<sensor_msgs::Temperature>(temp_topic_left, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubTempL.getTopic());
            mPubTempR = mNhNs.advertise<sensor_msgs::Temperature>(temp_topic_right, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubTempR.getTopic());
        }

        // Publish camera imu transform in a latched topic
        if (mZedRealCamModel != sl::MODEL::ZED) {
            std::string cam_imu_tr_topic = "left_cam_imu_transform";
            mPubCamImuTransf = mNhNs.advertise<geometry_msgs::Transform>(cam_imu_tr_topic, 1, true);

            sl::Orientation sl_rot = mSlCamImuTransf.getOrientation();
            sl::Translation sl_tr = mSlCamImuTransf.getTranslation();

            mCameraImuTransfMgs = boost::make_shared<geometry_msgs::Transform>();

            mCameraImuTransfMgs->rotation.x = sl_rot.ox;
            mCameraImuTransfMgs->rotation.y = sl_rot.oy;
            mCameraImuTransfMgs->rotation.z = sl_rot.oz;
            mCameraImuTransfMgs->rotation.w = sl_rot.ow;

            mCameraImuTransfMgs->translation.x = sl_tr.x;
            mCameraImuTransfMgs->translation.y = sl_tr.y;
            mCameraImuTransfMgs->translation.z = sl_tr.z;

            NODELET_DEBUG("Camera-IMU Rotation: \n %s", sl_rot.getRotationMatrix().getInfos().c_str());
            NODELET_DEBUG("Camera-IMU Translation: \n %g %g %g", sl_tr.x, sl_tr.y, sl_tr.z);

            mPubCamImuTransf.publish(mCameraImuTransfMgs);

            NODELET_INFO_STREAM("Advertised on topic " << mPubCamImuTransf.getTopic() << " [LATCHED]");
        }

        if (!mSvoMode && !mSensTimestampSync) {
            mFrameTimestamp = ros::Time::now();
            mSensPeriodMean_usec.reset(new sl_tools::CSmartMean(mSensPubRate / 2));
        } else {
            mSensPeriodMean_usec.reset(new sl_tools::CSmartMean(mCamFrameRate / 2));
        }
    }

    // Subscribers
    mClickedPtSub = mNhNs.subscribe(mClickedPtTopic, 10, &ZEDWrapperNodelet::clickedPtCallback, this);

    NODELET_INFO_STREAM("Subscribed to topic " << mClickedPtTopic.c_str());

    // Services
    mSrvSetInitPose = mNhNs.advertiseService("set_pose", &ZEDWrapperNodelet::on_set_pose, this);
    mSrvResetOdometry = mNhNs.advertiseService("reset_odometry", &ZEDWrapperNodelet::on_reset_odometry, this);
    mSrvResetTracking = mNhNs.advertiseService("reset_tracking", &ZEDWrapperNodelet::on_reset_tracking, this);
    mSrvSvoStartRecording = mNhNs.advertiseService("start_svo_recording", &ZEDWrapperNodelet::on_start_svo_recording, this);
    mSrvSvoStopRecording = mNhNs.advertiseService("stop_svo_recording", &ZEDWrapperNodelet::on_stop_svo_recording, this);

    mSrvSetLedStatus = mNhNs.advertiseService("set_led_status", &ZEDWrapperNodelet::on_set_led_status, this);
    mSrvToggleLed = mNhNs.advertiseService("toggle_led", &ZEDWrapperNodelet::on_toggle_led, this);
    mSrvSvoStartStream = mNhNs.advertiseService("start_remote_stream", &ZEDWrapperNodelet::on_start_remote_stream, this);
    mSrvSvoStopStream = mNhNs.advertiseService("stop_remote_stream", &ZEDWrapperNodelet::on_stop_remote_stream, this);

    mSrvStartMapping = mNhNs.advertiseService("start_3d_mapping", &ZEDWrapperNodelet::on_start_3d_mapping, this);
    mSrvStopMapping = mNhNs.advertiseService("stop_3d_mapping", &ZEDWrapperNodelet::on_stop_3d_mapping, this);
    mSrvSave3dMap = mNhNs.advertiseService("save_3d_map", &ZEDWrapperNodelet::on_save_3d_map, this);

    mSrvStartObjDet = mNhNs.advertiseService("start_object_detection", &ZEDWrapperNodelet::on_start_object_detection, this);
    mSrvStopObjDet = mNhNs.advertiseService("stop_object_detection", &ZEDWrapperNodelet::on_stop_object_detection, this);

    mSrvSaveAreaMemory = mNhNs.advertiseService("save_area_memory", &ZEDWrapperNodelet::on_save_area_memory, this);

    // Start Pointcloud thread
    mPcThread = std::thread(&ZEDWrapperNodelet::pointcloud_thread_func, this);

    // Start pool thread
    mDevicePollThread = std::thread(&ZEDWrapperNodelet::device_poll_thread_func, this);

    // Start data publishing timer
    mVideoDepthTimer = mNhNs.createTimer(ros::Duration(1.0 / mVideoDepthFreq), &ZEDWrapperNodelet::callback_pubVideoDepth, this);

    // Start Sensors thread
    mSensThread = std::thread(&ZEDWrapperNodelet::sensors_thread_func, this);
}

void ZEDWrapperNodelet::readParameters()
{
    NODELET_INFO_STREAM("*** GENERAL PARAMETERS ***");

    // ----> General
    // Get parameters from param files
    mNhNs.getParam("general/camera_name", mCameraName);
    NODELET_INFO_STREAM(" * Camera Name\t\t\t-> " << mCameraName.c_str());
    int resol;
    mNhNs.getParam("general/resolution", resol);
    mCamResol = static_cast<sl::RESOLUTION>(resol);
    NODELET_INFO_STREAM(" * Camera Resolution\t\t-> " << sl::toString(mCamResol).c_str());
    mNhNs.getParam("general/grab_frame_rate", mCamFrameRate);
    checkResolFps();
    NODELET_INFO_STREAM(" * Camera Grab Framerate\t-> " << mCamFrameRate);

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
        NODELET_INFO_STREAM(" * Serial number\t\t-> " << mZedSerialNumber);
    }

    std::string camera_model;
    mNhNs.getParam("general/camera_model", camera_model);

    if (camera_model == "zed") {
        mZedUserCamModel = sl::MODEL::ZED;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
    } else if (camera_model == "zedm") {
        mZedUserCamModel = sl::MODEL::ZED_M;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
    } else if (camera_model == "zed2") {
        mZedUserCamModel = sl::MODEL::ZED2;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
    } else if (camera_model == "zed2i") {
        mZedUserCamModel = sl::MODEL::ZED2i;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
    } else {
        NODELET_ERROR_STREAM("Camera model not valid: " << camera_model);
    }
    // <---- General

    NODELET_INFO_STREAM("*** VIDEO PARAMETERS ***");

    // ----> Video
    mNhNs.getParam("video/img_downsample_factor", mCamImageResizeFactor);
    NODELET_INFO_STREAM(" * Image resample factor\t-> " << mCamImageResizeFactor);

    mNhNs.getParam("video/extrinsic_in_camera_frame", mUseOldExtrinsic);
    NODELET_INFO_STREAM(" * Extrinsic param. frame\t-> "
        << (mUseOldExtrinsic ? "X RIGHT - Y DOWN - Z FWD" : "X FWD - Y LEFT - Z UP"));
    // <---- Video

    NODELET_INFO_STREAM("*** DEPTH PARAMETERS ***");

    // -----> Depth
    int depth_mode;
    mNhNs.getParam("depth/quality", depth_mode);
    mDepthMode = static_cast<sl::DEPTH_MODE>(depth_mode);
    NODELET_INFO_STREAM(" * Depth quality\t\t-> " << sl::toString(mDepthMode).c_str());
    int sensing_mode;
    mNhNs.getParam("depth/sensing_mode", sensing_mode);
    mCamSensingMode = static_cast<sl::SENSING_MODE>(sensing_mode);
    NODELET_INFO_STREAM(" * Depth Sensing mode\t\t-> " << sl::toString(mCamSensingMode).c_str());
    mNhNs.getParam("depth/openni_depth_mode", mOpenniDepthMode);
    NODELET_INFO_STREAM(" * OpenNI mode\t\t\t-> " << (mOpenniDepthMode ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("depth/depth_stabilization", mDepthStabilization);
    NODELET_INFO_STREAM(" * Depth Stabilization\t\t-> " << (mDepthStabilization ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("depth/min_depth", mCamMinDepth);
    NODELET_INFO_STREAM(" * Minimum depth\t\t-> " << mCamMinDepth << " m");
    mNhNs.getParam("depth/max_depth", mCamMaxDepth);
    NODELET_INFO_STREAM(" * Maximum depth\t\t-> " << mCamMaxDepth << " m");
    mNhNs.getParam("depth/depth_downsample_factor", mCamDepthResizeFactor);
    NODELET_INFO_STREAM(" * Depth resample factor\t-> " << mCamDepthResizeFactor);
    // <----- Depth

    NODELET_INFO_STREAM("*** POSITIONAL TRACKING PARAMETERS ***");

    // ----> Tracking
    mNhNs.param<bool>("pos_tracking/pos_tracking_enabled", mPosTrackingEnabled, true);
    NODELET_INFO_STREAM(" * Positional tracking\t\t-> " << (mPosTrackingEnabled ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("pos_tracking/path_pub_rate", mPathPubRate);
    NODELET_INFO_STREAM(" * Path rate\t\t\t-> " << mPathPubRate << " Hz");
    mNhNs.getParam("pos_tracking/path_max_count", mPathMaxCount);
    NODELET_INFO_STREAM(" * Path history size\t\t-> " << (mPathMaxCount == -1) ? std::string("INFINITE") : std::to_string(mPathMaxCount));

    if (mPathMaxCount < 2 && mPathMaxCount != -1) {
        mPathMaxCount = 2;
    }

    mNhNs.getParam("pos_tracking/initial_base_pose", mInitialBasePose);

    mNhNs.getParam("pos_tracking/area_memory_db_path", mAreaMemDbPath);
    mAreaMemDbPath = sl_tools::resolveFilePath(mAreaMemDbPath);
    NODELET_INFO_STREAM(" * Odometry DB path\t\t-> " << mAreaMemDbPath.c_str());

    mNhNs.param<bool>("pos_tracking/save_area_memory_db_on_exit", mSaveAreaMapOnClosing, false);
    NODELET_INFO_STREAM(" * Save Area Memory on closing\t-> " << (mSaveAreaMapOnClosing ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/area_memory", mAreaMemory, false);
    NODELET_INFO_STREAM(" * Area Memory\t\t\t-> " << (mAreaMemory ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/imu_fusion", mImuFusion, true);
    NODELET_INFO_STREAM(" * IMU Fusion\t\t\t-> " << (mImuFusion ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/floor_alignment", mFloorAlignment, false);
    NODELET_INFO_STREAM(" * Floor alignment\t\t-> " << (mFloorAlignment ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/init_odom_with_first_valid_pose", mInitOdomWithPose, true);
    NODELET_INFO_STREAM(" * Init Odometry with first valid pose data -> "
        << (mInitOdomWithPose ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/two_d_mode", mTwoDMode, false);
    NODELET_INFO_STREAM(" * Two D mode\t\t\t-> " << (mTwoDMode ? "ENABLED" : "DISABLED"));
    mNhNs.param<double>("pos_tracking/fixed_z_value", mFixedZValue, 0.0);

    if (mTwoDMode) {
        NODELET_INFO_STREAM(" * Fixed Z value\t\t-> " << mFixedZValue);
    }
    // <---- Tracking

    NODELET_INFO_STREAM("*** MAPPING PARAMETERS ***");

    // ----> Mapping
    mNhNs.param<bool>("mapping/mapping_enabled", mMappingEnabled, false);

    if (mMappingEnabled) {
        NODELET_INFO_STREAM(" * Mapping\t\t\t-> ENABLED");

        mNhNs.getParam("mapping/resolution", mMappingRes);
        NODELET_INFO_STREAM(" * Mapping resolution\t\t-> " << mMappingRes << " m");

        mNhNs.getParam("mapping/max_mapping_range", mMaxMappingRange);
        NODELET_INFO_STREAM(" * Mapping max range\t\t-> " << mMaxMappingRange << " m"
                                                          << ((mMaxMappingRange < 0.0) ? " [AUTO]" : ""));

        mNhNs.getParam("mapping/fused_pointcloud_freq", mFusedPcPubFreq);
        NODELET_INFO_STREAM(" * Fused point cloud freq:\t-> " << mFusedPcPubFreq << " Hz");
    } else {
        NODELET_INFO_STREAM(" * Mapping\t\t\t-> DISABLED");
    }

    mNhNs.param<std::string>("mapping/clicked_point_topic", mClickedPtTopic, std::string("/clicked_point"));
    NODELET_INFO_STREAM(" * Clicked point topic\t\t-> " << mClickedPtTopic.c_str());
    // <---- Mapping

    NODELET_INFO_STREAM("*** OBJECT DETECTION PARAMETERS ***");

    // ----> Object Detection
    mNhNs.param<bool>("object_detection/od_enabled", mObjDetEnabled, false);

    if (mObjDetEnabled) {
        NODELET_INFO_STREAM(" * Object Detection\t\t-> ENABLED");
        mNhNs.getParam("object_detection/confidence_threshold", mObjDetConfidence);
        NODELET_INFO_STREAM(" * Object confidence\t\t-> " << mObjDetConfidence);
        mNhNs.getParam("object_detection/object_tracking_enabled", mObjDetTracking);
        NODELET_INFO_STREAM(" * Object tracking\t\t-> " << (mObjDetTracking ? "ENABLED" : "DISABLED"));
        mNhNs.getParam("object_detection/max_range", mObjDetMaxRange);
        if (mObjDetMaxRange > mCamMaxDepth) {
            NODELET_WARN("Detection max range cannot be major than depth max range. Automatically fixed.");
            mObjDetMaxRange = mCamMaxDepth;
        }
        NODELET_INFO_STREAM(" * Detection max range\t\t-> " << mObjDetMaxRange);

        int model;
        mNhNs.getParam("object_detection/model", model);
        if (model < 0 || model >= static_cast<int>(sl::DETECTION_MODEL::LAST)) {
            NODELET_WARN("Detection model not valid. Forced to the default value");
            model = static_cast<int>(mObjDetModel);
        }
        mObjDetModel = static_cast<sl::DETECTION_MODEL>(model);

        NODELET_INFO_STREAM(" * Detection model\t\t-> " << sl::toString(mObjDetModel));

        if (mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE || mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_MEDIUM || mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_FAST) {
            mNhNs.getParam("object_detection/body_fitting", mObjDetBodyFitting);
            NODELET_INFO_STREAM(" * Body fitting\t\t\t-> " << (mObjDetBodyFitting ? "ENABLED" : "DISABLED"));
        } else {
            mNhNs.getParam("object_detection/mc_people", mObjDetPeopleEnable);
            NODELET_INFO_STREAM(" * Detect people\t\t-> " << (mObjDetPeopleEnable ? "ENABLED" : "DISABLED"));
            mNhNs.getParam("object_detection/mc_vehicle", mObjDetVehiclesEnable);
            NODELET_INFO_STREAM(" * Detect vehicles\t\t-> " << (mObjDetVehiclesEnable ? "ENABLED" : "DISABLED"));
            mNhNs.getParam("object_detection/mc_bag", mObjDetBagsEnable);
            NODELET_INFO_STREAM(" * Detect bags\t\t\t-> " << (mObjDetBagsEnable ? "ENABLED" : "DISABLED"));
            mNhNs.getParam("object_detection/mc_animal", mObjDetAnimalsEnable);
            NODELET_INFO_STREAM(" * Detect animals\t\t-> " << (mObjDetAnimalsEnable ? "ENABLED" : "DISABLED"));
            mNhNs.getParam("object_detection/mc_electronics", mObjDetElectronicsEnable);
            NODELET_INFO_STREAM(" * Detect electronics\t\t-> " << (mObjDetElectronicsEnable ? "ENABLED" : "DISABLED"));
            mNhNs.getParam("object_detection/mc_fruit_vegetable", mObjDetFruitsEnable);
            NODELET_INFO_STREAM(" * Detect fruit and vegetables\t-> " << (mObjDetFruitsEnable ? "ENABLED" : "DISABLED"));
            mNhNs.getParam("object_detection/mc_sport", mObjDetSportsEnable);
            NODELET_INFO_STREAM(" * Detect sport-related objects\t-> " << (mObjDetSportsEnable ? "ENABLED" : "DISABLED"));
        }
    } else if (mObjDetModel != sl::DETECTION_MODEL::PERSON_HEAD_BOX) {
        NODELET_INFO_STREAM(" * Object Detection\t\t-> DISABLED");
    }
    // <---- Object Detection

    NODELET_INFO_STREAM("*** SENSORS PARAMETERS ***");

    // ----> Sensors
    if (camera_model != "zed") {
        mNhNs.getParam("sensors/sensors_timestamp_sync", mSensTimestampSync);
        NODELET_INFO_STREAM(" * Sensors timestamp sync\t-> " << (mSensTimestampSync ? "ENABLED" : "DISABLED"));
        mNhNs.getParam("sensors/max_pub_rate", mSensPubRate);
        if (camera_model == "zedm") {
            if (mSensPubRate > 800.)
                mSensPubRate = 800.;
        } else {
            if (mSensPubRate > 400.)
                mSensPubRate = 400.;
        }

        NODELET_INFO_STREAM(" * Max sensors rate\t\t-> " << mSensPubRate);
    } else {
        NODELET_INFO_STREAM(" * The ZED camera has no available inertial/environmental sensors.");
    }
    // <---- Sensors

    NODELET_INFO_STREAM("*** SVO PARAMETERS ***");

    // ----> SVO
    mNhNs.param<std::string>("svo_file", mSvoFilepath, std::string());
    mSvoFilepath = sl_tools::resolveFilePath(mSvoFilepath);
    NODELET_INFO_STREAM(" * SVO input file: \t\t-> " << mSvoFilepath.c_str());

    int svo_compr = 0;
    mNhNs.getParam("general/svo_compression", svo_compr);

    if (svo_compr >= static_cast<int>(sl::SVO_COMPRESSION_MODE::LAST)) {
        NODELET_WARN_STREAM("The parameter `general/svo_compression` has an invalid value. Please check it in the "
                            "configuration file `common.yaml`");

        svo_compr = 0;
    }

    mSvoComprMode = static_cast<sl::SVO_COMPRESSION_MODE>(svo_compr);

    NODELET_INFO_STREAM(" * SVO REC compression\t\t-> " << sl::toString(mSvoComprMode));
    // <---- SVO

    // Remote Stream
    mNhNs.param<std::string>("stream", mRemoteStreamAddr, std::string());

    NODELET_INFO_STREAM("*** COORDINATE FRAMES ***");

    // ----> Coordinate frames
    mNhNs.param<std::string>("pos_tracking/map_frame", mMapFrameId, "map");
    mNhNs.param<std::string>("pos_tracking/odometry_frame", mOdometryFrameId, "odom");
    mNhNs.param<std::string>("general/base_frame", mBaseFrameId, "base_link");

    mCameraFrameId = mCameraName + "_camera_center";
    mImuFrameId = mCameraName + "_imu_link";
    mLeftCamFrameId = mCameraName + "_left_camera_frame";
    mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
    mRightCamFrameId = mCameraName + "_right_camera_frame";
    mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";

    mBaroFrameId = mCameraName + "_baro_link";
    mMagFrameId = mCameraName + "_mag_link";
    mTempLeftFrameId = mCameraName + "_temp_left_link";
    mTempRightFrameId = mCameraName + "_temp_right_link";

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
    mNhNs.param<bool>("pos_tracking/publish_tf", mPublishTf, true);
    NODELET_INFO_STREAM(" * Broadcast odometry TF\t-> " << (mPublishTf ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/publish_map_tf", mPublishMapTf, true);
    NODELET_INFO_STREAM(" * Broadcast map pose TF\t-> "
        << (mPublishTf ? (mPublishMapTf ? "ENABLED" : "DISABLED") : "DISABLED"));
    mNhNs.param<bool>("sensors/publish_imu_tf", mPublishImuTf, true);
    NODELET_INFO_STREAM(" * Broadcast IMU pose TF\t-> " << (mPublishImuTf ? "ENABLED" : "DISABLED"));
    // <---- TF broadcasting

    NODELET_INFO_STREAM("*** DYNAMIC PARAMETERS (Init. values) ***");

    // ----> Dynamic
    mNhNs.getParam("depth_confidence", mCamDepthConfidence);
    NODELET_INFO_STREAM(" * [DYN] Depth confidence\t-> " << mCamDepthConfidence);
    mNhNs.getParam("depth_texture_conf", mCamDepthTextureConf);
    NODELET_INFO_STREAM(" * [DYN] Depth texture conf.\t-> " << mCamDepthTextureConf);

    mNhNs.getParam("pub_frame_rate", mVideoDepthFreq);
    NODELET_INFO_STREAM(" * [DYN] pub_frame_rate\t\t-> " << mVideoDepthFreq << " Hz");
    mNhNs.getParam("point_cloud_freq", mPointCloudFreq);
    NODELET_INFO_STREAM(" * [DYN] point_cloud_freq\t-> " << mPointCloudFreq << " Hz");
    mNhNs.getParam("brightness", mCamBrightness);
    NODELET_INFO_STREAM(" * [DYN] brightness\t\t-> " << mCamBrightness);
    mNhNs.getParam("contrast", mCamContrast);
    NODELET_INFO_STREAM(" * [DYN] contrast\t\t-> " << mCamContrast);
    mNhNs.getParam("hue", mCamHue);
    NODELET_INFO_STREAM(" * [DYN] hue\t\t\t-> " << mCamHue);
    mNhNs.getParam("saturation", mCamSaturation);
    NODELET_INFO_STREAM(" * [DYN] saturation\t\t-> " << mCamSaturation);
    mNhNs.getParam("sharpness", mCamSharpness);
    NODELET_INFO_STREAM(" * [DYN] sharpness\t\t-> " << mCamSharpness);
#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 1)
    mNhNs.getParam("gamma", mCamGamma);
    NODELET_INFO_STREAM(" * [DYN] gamma\t\t\t-> " << mCamGamma);
#endif
    mNhNs.getParam("auto_exposure_gain", mCamAutoExposure);
    NODELET_INFO_STREAM(" * [DYN] auto_exposure_gain\t-> " << (mCamAutoExposure ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("gain", mCamGain);
    mNhNs.getParam("exposure", mCamExposure);
    if (!mCamAutoExposure) {
        NODELET_INFO_STREAM("  * [DYN] gain\t\t-> " << mCamGain);
        NODELET_INFO_STREAM("  * [DYN] exposure\t\t-> " << mCamExposure);
    }
    mNhNs.getParam("auto_whitebalance", mCamAutoWB);
    NODELET_INFO_STREAM(" * [DYN] auto_whitebalance\t-> " << (mCamAutoWB ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("whitebalance_temperature", mCamWB);
    if (!mCamAutoWB) {
        NODELET_INFO_STREAM("  * [DYN] whitebalance_temperature\t\t-> " << mCamWB);
    }

    if (mCamAutoExposure) {
        mTriggerAutoExposure = true;
    }
    if (mCamAutoWB) {
        mTriggerAutoWB = true;
    }
    // <---- Dynamic
}

void ZEDWrapperNodelet::checkResolFps()
{
    switch (mCamResol) {
    case sl::RESOLUTION::HD2K:
        if (mCamFrameRate != 15) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD2K. Set to 15 FPS.");
            mCamFrameRate = 15;
        }

        break;

    case sl::RESOLUTION::HD1080:
        if (mCamFrameRate == 15 || mCamFrameRate == 30) {
            break;
        }

        if (mCamFrameRate > 15 && mCamFrameRate < 30) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD1080. Set to 15 FPS.");
            mCamFrameRate = 15;
        } else if (mCamFrameRate > 30) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD1080. Set to 30 FPS.");
            mCamFrameRate = 30;
        } else {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD1080. Set to 15 FPS.");
            mCamFrameRate = 15;
        }

        break;

    case sl::RESOLUTION::HD720:
        if (mCamFrameRate == 15 || mCamFrameRate == 30 || mCamFrameRate == 60) {
            break;
        }

        if (mCamFrameRate > 15 && mCamFrameRate < 30) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 15 FPS.");
            mCamFrameRate = 15;
        } else if (mCamFrameRate > 30 && mCamFrameRate < 60) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 30 FPS.");
            mCamFrameRate = 30;
        } else if (mCamFrameRate > 60) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 60 FPS.");
            mCamFrameRate = 60;
        } else {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 15 FPS.");
            mCamFrameRate = 15;
        }

        break;

    case sl::RESOLUTION::VGA:
        if (mCamFrameRate == 15 || mCamFrameRate == 30 || mCamFrameRate == 60 || mCamFrameRate == 100) {
            break;
        }

        if (mCamFrameRate > 15 && mCamFrameRate < 30) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 15 FPS.");
            mCamFrameRate = 15;
        } else if (mCamFrameRate > 30 && mCamFrameRate < 60) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 30 FPS.");
            mCamFrameRate = 30;
        } else if (mCamFrameRate > 60 && mCamFrameRate < 100) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 60 FPS.");
            mCamFrameRate = 60;
        } else if (mCamFrameRate > 100) {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 100 FPS.");
            mCamFrameRate = 100;
        } else {
            NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 15 FPS.");
            mCamFrameRate = 15;
        }

        break;

    default:
        NODELET_WARN_STREAM("Invalid resolution. Set to HD720 @ 30 FPS");
        mCamResol = sl::RESOLUTION::HD720;
        mCamFrameRate = 30;
    }
}

void ZEDWrapperNodelet::initTransforms()
{
    // According to REP 105 -> http://www.ros.org/reps/rep-0105.html

    // base_link <- odom <- map
    //     ^                 |
    //     |                 |
    //     -------------------

    // ----> Dynamic transforms
    mOdom2BaseTransf.setIdentity(); // broadcasted if `publish_tf` is true
    mMap2OdomTransf.setIdentity(); // broadcasted if `publish_map_tf` is true
    mMap2BaseTransf.setIdentity(); // used internally, but not broadcasted
    mMap2CameraTransf.setIdentity(); // used internally, but not broadcasted
        // <---- Dynamic transforms
}

bool ZEDWrapperNodelet::getCamera2BaseTransform()
{
    NODELET_DEBUG("Getting static TF from '%s' to '%s'", mCameraFrameId.c_str(), mBaseFrameId.c_str());

    mCamera2BaseTransfValid = false;
    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Base link
    try {
        // Save the transformation
        geometry_msgs::TransformStamped c2b = mTfBuffer->lookupTransform(mCameraFrameId, mBaseFrameId, ros::Time(0), ros::Duration(0.1));

        // Get the TF2 transformation
        tf2::fromMsg(c2b.transform, mCamera2BaseTransf);

        double roll, pitch, yaw;
        tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        NODELET_INFO("Static transform Camera Center to Base [%s -> %s]", mCameraFrameId.c_str(), mBaseFrameId.c_str());
        NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}", mCamera2BaseTransf.getOrigin().x(),
            mCamera2BaseTransf.getOrigin().y(), mCamera2BaseTransf.getOrigin().z());
        NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    } catch (tf2::TransformException& ex) {
        if (!first_error) {
            NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
            NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.", mCameraFrameId.c_str(),
                mBaseFrameId.c_str());
            NODELET_WARN_THROTTLE(1.0,
                "Note: one of the possible cause of the problem is the absense of an instance "
                "of the `robot_state_publisher` node publishing the correct static TF transformations "
                "or a modified URDF not correctly reproducing the ZED "
                "TF chain '%s' -> '%s' -> '%s'",
                mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
            first_error = false;
        }

        mCamera2BaseTransf.setIdentity();
        return false;
    }

    // <---- Static transforms
    mCamera2BaseTransfValid = true;
    return true;
}

bool ZEDWrapperNodelet::getSens2CameraTransform()
{
    NODELET_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mCameraFrameId.c_str());

    mSensor2CameraTransfValid = false;

    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Camera Center
    try {
        // Save the transformation
        geometry_msgs::TransformStamped s2c = mTfBuffer->lookupTransform(mDepthFrameId, mCameraFrameId, ros::Time(0), ros::Duration(0.1));
        // Get the TF2 transformation
        tf2::fromMsg(s2c.transform, mSensor2CameraTransf);

        double roll, pitch, yaw;
        tf2::Matrix3x3(mSensor2CameraTransf.getRotation()).getRPY(roll, pitch, yaw);

        NODELET_INFO("Static transform Sensor to Camera Center [%s -> %s]", mDepthFrameId.c_str(), mCameraFrameId.c_str());
        NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}", mSensor2CameraTransf.getOrigin().x(),
            mSensor2CameraTransf.getOrigin().y(), mSensor2CameraTransf.getOrigin().z());
        NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    } catch (tf2::TransformException& ex) {
        if (!first_error) {
            NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
            NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.", mDepthFrameId.c_str(),
                mCameraFrameId.c_str());
            NODELET_WARN_THROTTLE(1.0,
                "Note: one of the possible cause of the problem is the absense of an instance "
                "of the `robot_state_publisher` node publishing the correct static TF transformations "
                "or a modified URDF not correctly reproducing the ZED "
                "TF chain '%s' -> '%s' -> '%s'",
                mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
            first_error = false;
        }

        mSensor2CameraTransf.setIdentity();
        return false;
    }

    // <---- Static transforms

    mSensor2CameraTransfValid = true;
    return true;
}

bool ZEDWrapperNodelet::getSens2BaseTransform()
{
    NODELET_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mBaseFrameId.c_str());

    mSensor2BaseTransfValid = false;
    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Base link
    try {
        // Save the transformation
        geometry_msgs::TransformStamped s2b = mTfBuffer->lookupTransform(mDepthFrameId, mBaseFrameId, ros::Time(0), ros::Duration(0.1));
        // Get the TF2 transformation
        tf2::fromMsg(s2b.transform, mSensor2BaseTransf);

        double roll, pitch, yaw;
        tf2::Matrix3x3(mSensor2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        NODELET_INFO("Static transform Sensor to Base [%s -> %s]", mDepthFrameId.c_str(), mBaseFrameId.c_str());
        NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}", mSensor2BaseTransf.getOrigin().x(),
            mSensor2BaseTransf.getOrigin().y(), mSensor2BaseTransf.getOrigin().z());
        NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    } catch (tf2::TransformException& ex) {
        if (!first_error) {
            NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
            NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.", mDepthFrameId.c_str(),
                mBaseFrameId.c_str());
            NODELET_WARN_THROTTLE(1.0,
                "Note: one of the possible cause of the problem is the absense of an instance "
                "of the `robot_state_publisher` node publishing the correct static TF transformations "
                "or a modified URDF not correctly reproducing the ZED "
                "TF chain '%s' -> '%s' -> '%s'",
                mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
            first_error = false;
        }

        mSensor2BaseTransf.setIdentity();
        return false;
    }

    // <---- Static transforms

    mSensor2BaseTransfValid = true;
    return true;
}

bool ZEDWrapperNodelet::set_pose(float xt, float yt, float zt, float rr, float pr, float yr)
{
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

bool ZEDWrapperNodelet::on_set_pose(zed_interfaces::set_pose::Request& req, zed_interfaces::set_pose::Response& res)
{
    mInitialBasePose.resize(6);
    mInitialBasePose[0] = req.x;
    mInitialBasePose[1] = req.y;
    mInitialBasePose[2] = req.z;
    mInitialBasePose[3] = req.R;
    mInitialBasePose[4] = req.P;
    mInitialBasePose[5] = req.Y;

    std::lock_guard<std::mutex> lock(mPosTrkMutex);

    // Restart tracking
    start_pos_tracking();

    res.done = true;
    return true;
}

bool ZEDWrapperNodelet::on_reset_tracking(zed_interfaces::reset_tracking::Request& req,
    zed_interfaces::reset_tracking::Response& res)
{
    if (!mPosTrackingActivated) {
        res.reset_done = false;
        return false;
    }

    std::lock_guard<std::mutex> lock(mPosTrkMutex);

    // Restart tracking
    start_pos_tracking();

    res.reset_done = true;
    return true;
}

bool ZEDWrapperNodelet::on_reset_odometry(zed_interfaces::reset_odometry::Request& req,
    zed_interfaces::reset_odometry::Response& res)
{
    mResetOdom = true;
    res.reset_done = true;
    return true;
}

bool ZEDWrapperNodelet::start_3d_mapping()
{
    if (!mMappingEnabled) {
        return false;
    }

    NODELET_INFO_STREAM("*** Starting Spatial Mapping ***");

    sl::SpatialMappingParameters params;
    params.map_type = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
    params.use_chunk_only = true;

    sl::SpatialMappingParameters spMapPar;

    float lRes = spMapPar.allowed_resolution.first;
    float hRes = spMapPar.allowed_resolution.second;

    if (mMappingRes < lRes) {
        NODELET_WARN_STREAM("'mapping/resolution' value ("
            << mMappingRes << " m) is lower than the allowed resolution values. Fixed automatically");
        mMappingRes = lRes;
    }
    if (mMappingRes > hRes) {
        NODELET_WARN_STREAM("'mapping/resolution' value ("
            << mMappingRes << " m) is higher than the allowed resolution values. Fixed automatically");
        mMappingRes = hRes;
    }

    params.resolution_meter = mMappingRes;

    float lRng = spMapPar.allowed_range.first;
    float hRng = spMapPar.allowed_range.second;

    if (mMaxMappingRange < 0) {
        mMaxMappingRange = sl::SpatialMappingParameters::getRecommendedRange(mMappingRes, mZed);
        NODELET_INFO_STREAM("Mapping: max range set to " << mMaxMappingRange << " m for a resolution of " << mMappingRes
                                                         << " m");
    } else if (mMaxMappingRange < lRng) {
        NODELET_WARN_STREAM("'mapping/max_mapping_range_m' value ("
            << mMaxMappingRange << " m) is lower than the allowed resolution values. Fixed automatically");
        mMaxMappingRange = lRng;
    } else if (mMaxMappingRange > hRng) {
        NODELET_WARN_STREAM("'mapping/max_mapping_range_m' value ("
            << mMaxMappingRange << " m) is higher than the allowed resolution values. Fixed automatically");
        mMaxMappingRange = hRng;
    }

    params.range_meter = mMaxMappingRange;

    sl::ERROR_CODE err = mZed.enableSpatialMapping(params);

    if (err == sl::ERROR_CODE::SUCCESS) {
        if (mPubFusedCloud.getTopic().empty()) {
            std::string pointcloud_fused_topic = "mapping/fused_cloud";
            mPubFusedCloud = mNhNs.advertise<sensor_msgs::PointCloud2>(pointcloud_fused_topic, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubFusedCloud.getTopic() << " @ " << mFusedPcPubFreq << " Hz");
        }

        mMappingRunning = true;

        mFusedPcTimer = mNhNs.createTimer(ros::Duration(1.0 / mFusedPcPubFreq), &ZEDWrapperNodelet::callback_pubFusedPointCloud, this);

        NODELET_INFO_STREAM(" * Resolution: " << params.resolution_meter << " m");
        NODELET_INFO_STREAM(" * Max Mapping Range: " << params.range_meter << " m");
        NODELET_INFO_STREAM(" * Map point cloud publishing rate: " << mFusedPcPubFreq << " Hz");

        return true;
    } else {
        mMappingRunning = false;
        mFusedPcTimer.stop();

        NODELET_WARN("Mapping not activated: %s", sl::toString(err).c_str());

        return false;
    }
}

void ZEDWrapperNodelet::stop_3d_mapping()
{
    mFusedPcTimer.stop();
    mMappingRunning = false;
    mMappingEnabled = false;
    mZed.disableSpatialMapping();

    NODELET_INFO("*** Spatial Mapping stopped ***");
}

bool ZEDWrapperNodelet::start_obj_detect()
{
    if (mZedRealCamModel == sl::MODEL::ZED) {
        NODELET_ERROR_STREAM("Object detection not started. OD is not available for ZED camera model");
        return false;
    }

    if (!mObjDetEnabled) {
        return false;
    }

    if (!mCamera2BaseTransfValid || !mSensor2CameraTransfValid || !mSensor2BaseTransfValid) {
        NODELET_DEBUG("Tracking transforms not yet ready, OD starting postponed");
        return false;
    }

    NODELET_INFO_STREAM("*** Starting Object Detection ***");

    sl::ObjectDetectionParameters od_p;
    od_p.enable_mask_output = false;
    od_p.enable_tracking = mObjDetTracking;
    od_p.image_sync = false; // Asynchronous object detection
    od_p.detection_model = mObjDetModel;
    od_p.enable_body_fitting = mObjDetBodyFitting;
    od_p.max_range = mObjDetMaxRange;

    sl::ERROR_CODE objDetError = mZed.enableObjectDetection(od_p);

    if (objDetError != sl::ERROR_CODE::SUCCESS) {
        NODELET_ERROR_STREAM("Object detection error: " << sl::toString(objDetError));

        mObjDetRunning = false;
        return false;
    }

    if (mPubObjDet.getTopic().empty()) {
        std::string object_det_topic_root = "obj_det";
        std::string object_det_topic = object_det_topic_root + "/objects";

        mPubObjDet = mNhNs.advertise<zed_interfaces::ObjectsStamped>(object_det_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubObjDet.getTopic());
    }

    mObjDetFilter.clear();

    if (mObjDetModel == sl::DETECTION_MODEL::MULTI_CLASS_BOX || mObjDetModel == sl::DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM || mObjDetModel == sl::DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE) {
        if (mObjDetPeopleEnable) {
            mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
        }
        if (mObjDetVehiclesEnable) {
            mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
        }
        if (mObjDetBagsEnable) {
            mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
        }
        if (mObjDetAnimalsEnable) {
            mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
        }
        if (mObjDetElectronicsEnable) {
            mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
        }
        if (mObjDetFruitsEnable) {
            mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
        }
        if (mObjDetSportsEnable) {
            mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
        }
    }

    mObjDetRunning = true;
    return false;
}

void ZEDWrapperNodelet::stop_obj_detect()
{
    if (mObjDetRunning) {
        NODELET_INFO_STREAM("*** Stopping Object Detection ***");
        mObjDetRunning = false;
        mObjDetEnabled = false;
        mZed.disableObjectDetection();
    }
}

void ZEDWrapperNodelet::start_pos_tracking()
{
    NODELET_INFO_STREAM("*** Starting Positional Tracking ***");

    NODELET_INFO(" * Waiting for valid static transformations...");

    mPosTrackingReady = false; // Useful to not publish wrong TF with IMU frame broadcasting
    bool transformOk = false;
    double elapsed = 0.0;

    auto start = std::chrono::high_resolution_clock::now();

    do {
        transformOk = set_pose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2], mInitialBasePose[3],
            mInitialBasePose[4], mInitialBasePose[5]);

        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start)
                      .count();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (elapsed > 10000) {
            NODELET_WARN(" !!! Failed to get static transforms. Is the 'ROBOT STATE PUBLISHER' node correctly working? ");
            break;
        }

    } while (transformOk == false);

    if (transformOk) {
        NODELET_DEBUG("Time required to get valid static transforms: %g sec", elapsed / 1000.);
    }

    NODELET_INFO("Initial ZED left camera pose (ZED pos. tracking): ");
    NODELET_INFO(" * T: [%g,%g,%g]", mInitialPoseSl.getTranslation().x, mInitialPoseSl.getTranslation().y,
        mInitialPoseSl.getTranslation().z);
    NODELET_INFO(" * Q: [%g,%g,%g,%g]", mInitialPoseSl.getOrientation().ox, mInitialPoseSl.getOrientation().oy,
        mInitialPoseSl.getOrientation().oz, mInitialPoseSl.getOrientation().ow);

    // Positional Tracking parameters
    sl::PositionalTrackingParameters posTrackParams;

    posTrackParams.initial_world_transform = mInitialPoseSl;
    posTrackParams.enable_area_memory = mAreaMemory;

    mPoseSmoothing = false; // Always false. Pose Smoothing is to be enabled only for VR/AR applications
    posTrackParams.enable_pose_smoothing = mPoseSmoothing;

    posTrackParams.set_floor_as_origin = mFloorAlignment;

    if (mAreaMemDbPath != "" && !sl_tools::file_exist(mAreaMemDbPath)) {
        posTrackParams.area_file_path = "";
        NODELET_WARN_STREAM("area_memory_db_path [" << mAreaMemDbPath << "] doesn't exist or is unreachable. ");
        if (mSaveAreaMapOnClosing) {
            NODELET_INFO_STREAM("The file will be automatically created when closing the node or calling the "
                                "'save_area_map' service if a valid Area Memory is available.");
        }
    } else {
        posTrackParams.area_file_path = mAreaMemDbPath.c_str();
    }

    posTrackParams.enable_imu_fusion = mImuFusion;

    posTrackParams.set_as_static = false;

    sl::ERROR_CODE err = mZed.enablePositionalTracking(posTrackParams);

    if (err == sl::ERROR_CODE::SUCCESS) {
        mPosTrackingActivated = true;
    } else {
        mPosTrackingActivated = false;

        NODELET_WARN("Tracking not activated: %s", sl::toString(err).c_str());
    }
}

bool ZEDWrapperNodelet::on_save_area_memory(zed_interfaces::save_area_memory::Request& req,
    zed_interfaces::save_area_memory::Response& res)
{
    std::string file_path = sl_tools::resolveFilePath(req.area_memory_filename);

    bool ret = saveAreaMap(file_path, &res.info);

    return ret;
}

bool ZEDWrapperNodelet::saveAreaMap(std::string file_path, std::string* out_msg)
{
    std::ostringstream os;

    bool node_running = mNhNs.ok();
    if (!mZed.isOpened()) {
        os << "Cannot save Area Memory. The camera is closed.";

        if (node_running)
            NODELET_WARN_STREAM(os.str().c_str());
        else
            std::cerr << os.str() << std::endl;

        if (out_msg)
            *out_msg = os.str();

        return false;
    }

    if (mPosTrackingActivated && mAreaMemory) {
        sl::ERROR_CODE err = mZed.saveAreaMap(sl::String(file_path.c_str()));
        if (err != sl::ERROR_CODE::SUCCESS) {
            os << "Error saving positional tracking area memory: " << sl::toString(err).c_str();

            if (node_running)
                NODELET_WARN_STREAM(os.str().c_str());
            else
                std::cerr << os.str() << std::endl;

            if (out_msg)
                *out_msg = os.str();

            return false;
        }

        if (node_running)
            NODELET_INFO_STREAM("Saving Area Memory file: " << file_path);
        else
            std::cerr << "Saving Area Memory file: " << file_path << " ";

        sl::AREA_EXPORTING_STATE state;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            state = mZed.getAreaExportState();
            if (node_running)
                NODELET_INFO_STREAM(".");
            else
                std::cerr << ".";
        } while (state == sl::AREA_EXPORTING_STATE::RUNNING);
        if (!node_running)
            std::cerr << std::endl;

        if (state == sl::AREA_EXPORTING_STATE::SUCCESS) {
            os << "Area Memory file saved correctly.";

            if (node_running)
                NODELET_INFO_STREAM(os.str().c_str());
            else
                std::cerr << os.str() << std::endl;

            if (out_msg)
                *out_msg = os.str();
            return true;
        }

        os << "Error saving Area Memory file: " << sl::toString(state).c_str();

        if (node_running)
            NODELET_WARN_STREAM(os.str().c_str());
        else
            std::cerr << os.str() << std::endl;

        if (out_msg)
            *out_msg = os.str();

        return false;
    }
    return false;
}

void ZEDWrapperNodelet::publishOdom(tf2::Transform odom2baseTransf, sl::Pose& slPose, ros::Time t)
{
    nav_msgs::OdometryPtr odomMsg = boost::make_shared<nav_msgs::Odometry>();

    odomMsg->header.stamp = t;
    odomMsg->header.frame_id = mOdometryFrameId; // frame
    odomMsg->child_frame_id = mBaseFrameId; // camera_frame
    // conversion from Tranform to message
    geometry_msgs::Transform base2odom = tf2::toMsg(odom2baseTransf);
    // Add all value in odometry message
    odomMsg->pose.pose.position.x = base2odom.translation.x;
    odomMsg->pose.pose.position.y = base2odom.translation.y;
    odomMsg->pose.pose.position.z = base2odom.translation.z;
    odomMsg->pose.pose.orientation.x = base2odom.rotation.x;
    odomMsg->pose.pose.orientation.y = base2odom.rotation.y;
    odomMsg->pose.pose.orientation.z = base2odom.rotation.z;
    odomMsg->pose.pose.orientation.w = base2odom.rotation.w;

    // Odometry pose covariance

    for (size_t i = 0; i < odomMsg->pose.covariance.size(); i++) {
        odomMsg->pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]);

        if (mTwoDMode) {
            if (i == 14 || i == 21 || i == 28) {
                odomMsg->pose.covariance[i] = 1e-9; // Very low covariance if 2D mode
            } else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) || (i >= 12 && i <= 13) || (i >= 15 && i <= 16) || (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27)) {
                odomMsg->pose.covariance[i] = 0.0;
            }
        }
    }

    // Publish odometry message
    mPubOdom.publish(odomMsg);
}

void ZEDWrapperNodelet::publishPose(ros::Time t)
{
    tf2::Transform base_pose;
    base_pose.setIdentity();

    if (mPublishMapTf) {
        base_pose = mMap2BaseTransf;
    } else if (mPublishTf) {
        base_pose = mOdom2BaseTransf;
    }

    std_msgs::Header header;
    header.stamp = t;
    header.frame_id = mMapFrameId;
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

    if (mPubPoseCov.getNumSubscribers() > 0) {
        geometry_msgs::PoseWithCovarianceStampedPtr poseCovMsg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

        poseCovMsg->header = header;
        poseCovMsg->pose.pose = pose;

        // Odometry pose covariance if available
        for (size_t i = 0; i < poseCovMsg->pose.covariance.size(); i++) {
            poseCovMsg->pose.covariance[i] = static_cast<double>(mLastZedPose.pose_covariance[i]);

            if (mTwoDMode) {
                if (i == 14 || i == 21 || i == 28) {
                    poseCovMsg->pose.covariance[i] = 1e-9; // Very low covariance if 2D mode
                } else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) || (i >= 12 && i <= 13) || (i >= 15 && i <= 16) || (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27)) {
                    poseCovMsg->pose.covariance[i] = 0.0;
                }
            }
        }

        // Publish pose with covariance stamped message
        mPubPoseCov.publish(poseCovMsg);
    }
}

void ZEDWrapperNodelet::publishStaticImuFrame()
{
    // Publish IMU TF as static TF
    if (!mPublishImuTf) {
        return;
    }

    if (mStaticImuFramePublished) {
        return;
    }

    mStaticImuTransformStamped.header.stamp = ros::Time::now();
    mStaticImuTransformStamped.header.frame_id = mLeftCamFrameId;
    mStaticImuTransformStamped.child_frame_id = mImuFrameId;
    sl::Translation sl_tr = mSlCamImuTransf.getTranslation();
    mStaticImuTransformStamped.transform.translation.x = sl_tr.x;
    mStaticImuTransformStamped.transform.translation.y = sl_tr.y;
    mStaticImuTransformStamped.transform.translation.z = sl_tr.z;
    sl::Orientation sl_or = mSlCamImuTransf.getOrientation();
    mStaticImuTransformStamped.transform.rotation.x = sl_or.ox;
    mStaticImuTransformStamped.transform.rotation.y = sl_or.oy;
    mStaticImuTransformStamped.transform.rotation.z = sl_or.oz;
    mStaticImuTransformStamped.transform.rotation.w = sl_or.ow;

    // Publish transformation
    mStaticTransformImuBroadcaster.sendTransform(mStaticImuTransformStamped);

    NODELET_INFO_STREAM("Published static transform '" << mImuFrameId << "' -> '" << mLeftCamFrameId << "'");

    mStaticImuFramePublished = true;
}

void ZEDWrapperNodelet::publishOdomFrame(tf2::Transform odomTransf, ros::Time t)
{
    // ----> Avoid duplicated TF publishing
    static ros::Time last_stamp;

    if (t == last_stamp) {
        return;
    }
    last_stamp = t;
    // <---- Avoid duplicated TF publishing

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

    // NODELET_INFO_STREAM( "Published ODOM TF with TS: " << t );
}

void ZEDWrapperNodelet::publishPoseFrame(tf2::Transform baseTransform, ros::Time t)
{
    // ----> Avoid duplicated TF publishing
    static ros::Time last_stamp;

    if (t == last_stamp) {
        return;
    }
    last_stamp = t;
    // <---- Avoid duplicated TF publishing

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

    // NODELET_INFO_STREAM( "Published POSE TF with TS: " << t );
}

void ZEDWrapperNodelet::publishImage(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img,
    image_transport::CameraPublisher& pubImg, sensor_msgs::CameraInfoPtr camInfoMsg,
    std::string imgFrameId, ros::Time t)
{
    camInfoMsg->header.stamp = t;
    sl_tools::imageToROSmsg(imgMsgPtr, img, imgFrameId, t);
    pubImg.publish(imgMsgPtr, camInfoMsg);
}

void ZEDWrapperNodelet::publishDepth(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat depth, ros::Time t)
{
    mDepthCamInfoMsg->header.stamp = t;

    // NODELET_DEBUG_STREAM("mOpenniDepthMode: " << mOpenniDepthMode);

    if (!mOpenniDepthMode) {
        // NODELET_INFO("Using float32");
        sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
        mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);

        return;
    }

#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 4)
    // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
    if (!imgMsgPtr) {
        imgMsgPtr = boost::make_shared<sensor_msgs::Image>();
    }

    imgMsgPtr->header.stamp = t;
    imgMsgPtr->header.frame_id = mDepthOptFrameId;
    imgMsgPtr->height = depth.getHeight();
    imgMsgPtr->width = depth.getWidth();

    int num = 1; // for endianness detection
    imgMsgPtr->is_bigendian = !(*(char*)&num == 1);

    imgMsgPtr->step = imgMsgPtr->width * sizeof(uint16_t);
    imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_16UC1;

    size_t size = imgMsgPtr->step * imgMsgPtr->height;
    imgMsgPtr->data.resize(size);

    uint16_t* data = (uint16_t*)(&imgMsgPtr->data[0]);

    int dataSize = imgMsgPtr->width * imgMsgPtr->height;
    sl::float1* depthDataPtr = depth.getPtr<sl::float1>();

    for (int i = 0; i < dataSize; i++) {
        *(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++) * 1000)); // in mm, rounded
    }
    mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
#else
    // NODELET_INFO("Using depth16");
    sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
    mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
#endif
}

void ZEDWrapperNodelet::publishDisparity(sl::Mat disparity, ros::Time t)
{
    sl::CameraInformation zedParam = mZed.getCameraInformation(mMatResolDepth);

    sensor_msgs::ImagePtr disparityImgMsg = boost::make_shared<sensor_msgs::Image>();
    stereo_msgs::DisparityImagePtr disparityMsg = boost::make_shared<stereo_msgs::DisparityImage>();

    sl_tools::imageToROSmsg(disparityImgMsg, disparity, mDisparityFrameId, t);

    disparityMsg->image = *disparityImgMsg;
    disparityMsg->header = disparityMsg->image.header;

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    disparityMsg->f = zedParam.calibration_parameters.left_cam.fx;
    disparityMsg->T = zedParam.calibration_parameters.T.x;
#else
    disparityMsg->f = zedParam.camera_configuration.calibration_parameters.left_cam.fx;
    disparityMsg->T = zedParam.camera_configuration.calibration_parameters.getCameraBaseline();
#endif

    if (disparityMsg->T > 0) {
        disparityMsg->T *= -1.0f;
    }

    disparityMsg->min_disparity = disparityMsg->f * disparityMsg->T / mZed.getInitParameters().depth_minimum_distance;
    disparityMsg->max_disparity = disparityMsg->f * disparityMsg->T / mZed.getInitParameters().depth_maximum_distance;

    mPubDisparity.publish(disparityMsg);
}

void ZEDWrapperNodelet::pointcloud_thread_func()
{
    std::unique_lock<std::mutex> lock(mPcMutex);

    while (!mStopNode) {
        while (!mPcDataReady) { // loop to avoid spurious wakeups
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

void ZEDWrapperNodelet::publishPointCloud()
{
    sensor_msgs::PointCloud2Ptr pointcloudMsg = boost::make_shared<sensor_msgs::PointCloud2>();

    // Publish freq calculation
    static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
    last_time = now;

    mPcPeriodMean_usec->addValue(elapsed_usec);

    // Initialize Point Cloud message
    // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

    int ptsCount = mMatResolDepth.width * mMatResolDepth.height;

    pointcloudMsg->header.stamp = mPointCloudTime;

    if (pointcloudMsg->width != mMatResolDepth.width || pointcloudMsg->height != mMatResolDepth.height) {
        pointcloudMsg->header.frame_id = mPointCloudFrameId; // Set the header values of the ROS message

        pointcloudMsg->is_bigendian = false;
        pointcloudMsg->is_dense = false;

        pointcloudMsg->width = mMatResolDepth.width;
        pointcloudMsg->height = mMatResolDepth.height;

        sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1, sensor_msgs::PointField::FLOAT32);
    }

    // Data copy
    sl::Vector4<float>* cpu_cloud = mCloud.getPtr<sl::float4>();
    float* ptCloudPtr = (float*)(&pointcloudMsg->data[0]);

    // We can do a direct memcpy since data organization is the same
    memcpy(ptCloudPtr, (float*)cpu_cloud, 4 * ptsCount * sizeof(float));

    // Pointcloud publishing
    mPubCloud.publish(pointcloudMsg);
}

void ZEDWrapperNodelet::callback_pubFusedPointCloud(const ros::TimerEvent& e)
{
    sensor_msgs::PointCloud2Ptr pointcloudFusedMsg = boost::make_shared<sensor_msgs::PointCloud2>();

    uint32_t fusedCloudSubnumber = mPubFusedCloud.getNumSubscribers();

    if (fusedCloudSubnumber == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(mCloseZedMutex);

    if (!mZed.isOpened()) {
        return;
    }

    // pointcloudFusedMsg->header.stamp = t;
    mZed.requestSpatialMapAsync();

    while (mZed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::FAILURE) {
        // Mesh is still generating
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sl::ERROR_CODE res = mZed.retrieveSpatialMapAsync(mFusedPC);

    if (res != sl::ERROR_CODE::SUCCESS) {
        NODELET_WARN_STREAM("Fused point cloud not extracted: " << sl::toString(res).c_str());
        return;
    }

    size_t ptsCount = mFusedPC.getNumberOfPoints();
    bool resized = false;

    if (pointcloudFusedMsg->width != ptsCount || pointcloudFusedMsg->height != 1) {
        // Initialize Point Cloud message
        // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
        pointcloudFusedMsg->header.frame_id = mMapFrameId; // Set the header values of the ROS message
        pointcloudFusedMsg->is_bigendian = false;
        pointcloudFusedMsg->is_dense = false;
        pointcloudFusedMsg->width = ptsCount;
        pointcloudFusedMsg->height = 1;

        sensor_msgs::PointCloud2Modifier modifier(*pointcloudFusedMsg);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1, sensor_msgs::PointField::FLOAT32);

        resized = true;
    }

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    // NODELET_INFO_STREAM("Chunks: " << mFusedPC.chunks.size());

    int index = 0;
    float* ptCloudPtr = (float*)(&pointcloudFusedMsg->data[0]);
    int updated = 0;

    for (int c = 0; c < mFusedPC.chunks.size(); c++) {
        if (mFusedPC.chunks[c].has_been_updated || resized) {
            updated++;

            size_t chunkSize = mFusedPC.chunks[c].vertices.size();

            if (chunkSize > 0) {
                float* cloud_pts = (float*)(mFusedPC.chunks[c].vertices.data());

                memcpy(ptCloudPtr, cloud_pts, 4 * chunkSize * sizeof(float));

                ptCloudPtr += 4 * chunkSize;

                pointcloudFusedMsg->header.stamp = sl_tools::slTime2Ros(mFusedPC.chunks[c].timestamp);
            }
        } else {
            index += mFusedPC.chunks[c].vertices.size();
        }
    }

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

    // NODELET_INFO_STREAM("Updated: " << updated);

    // double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    //        NODELET_INFO_STREAM("Data copy: " << elapsed_usec << " usec [" << ptsCount << "] - " << (static_cast<double>
    //                        (ptsCount) / elapsed_usec) << " pts/usec");

    // Pointcloud publishing
    mPubFusedCloud.publish(pointcloudFusedMsg);
}

void ZEDWrapperNodelet::publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t)
{
    static int seq = 0;
    camInfoMsg->header.stamp = t;
    camInfoMsg->header.seq = seq;
    pubCamInfo.publish(camInfoMsg);
    seq++;
}

void ZEDWrapperNodelet::fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr leftCamInfoMsg,
    sensor_msgs::CameraInfoPtr rightCamInfoMsg, std::string leftFrameId, std::string rightFrameId,
    bool rawParam /*= false*/)
{
    sl::CalibrationParameters zedParam;

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    if (rawParam) {
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters_raw; // ok
    } else {
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters; // ok
    }
#else
    if (rawParam) {
        zedParam = zed.getCameraInformation(mMatResolVideo).camera_configuration.calibration_parameters_raw;
    } else {
        zedParam = zed.getCameraInformation(mMatResolVideo).camera_configuration.calibration_parameters;
    }
#endif

    float baseline = zedParam.getCameraBaseline();
    leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    leftCamInfoMsg->D.resize(5);
    rightCamInfoMsg->D.resize(5);
    leftCamInfoMsg->D[0] = zedParam.left_cam.disto[0]; // k1
    leftCamInfoMsg->D[1] = zedParam.left_cam.disto[1]; // k2
    leftCamInfoMsg->D[2] = zedParam.left_cam.disto[4]; // k3
    leftCamInfoMsg->D[3] = zedParam.left_cam.disto[2]; // p1
    leftCamInfoMsg->D[4] = zedParam.left_cam.disto[3]; // p2
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

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    if (rawParam) {
        std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
        float* p = R_.data();

        for (int i = 0; i < 9; i++) {
            rightCamInfoMsg->R[i] = p[i];
        }
    }
#else
    if (rawParam) {
        if (mUseOldExtrinsic) { // Camera frame (Z forward, Y down, X right)

            std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
            float* p = R_.data();

            for (int i = 0; i < 9; i++) {
                rightCamInfoMsg->R[i] = p[i];
            }
        } else { // ROS frame (X forward, Z up, Y left)
            for (int i = 0; i < 9; i++) {
                rightCamInfoMsg->R[i] = zedParam.stereo_transform.getRotationMatrix().r[i];
            }
        }
    }
#endif

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
    leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatResolVideo.width);
    leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatResolVideo.height);
    leftCamInfoMsg->header.frame_id = leftFrameId;
    rightCamInfoMsg->header.frame_id = rightFrameId;
}

void ZEDWrapperNodelet::fillCamDepthInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr depth_info_msg, std::string frame_id)
{
    sl::CalibrationParameters zedParam;

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    zedParam = zed.getCameraInformation(mMatResolDepth).calibration_parameters;
#else
    zedParam = zed.getCameraInformation(mMatResolDepth).camera_configuration.calibration_parameters;
#endif

    float baseline = zedParam.getCameraBaseline();
    depth_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    depth_info_msg->D.resize(5);
    depth_info_msg->D[0] = zedParam.left_cam.disto[0]; // k1
    depth_info_msg->D[1] = zedParam.left_cam.disto[1]; // k2
    depth_info_msg->D[2] = zedParam.left_cam.disto[4]; // k3
    depth_info_msg->D[3] = zedParam.left_cam.disto[2]; // p1
    depth_info_msg->D[4] = zedParam.left_cam.disto[3]; // p2
    depth_info_msg->K.fill(0.0);
    depth_info_msg->K[0] = static_cast<double>(zedParam.left_cam.fx);
    depth_info_msg->K[2] = static_cast<double>(zedParam.left_cam.cx);
    depth_info_msg->K[4] = static_cast<double>(zedParam.left_cam.fy);
    depth_info_msg->K[5] = static_cast<double>(zedParam.left_cam.cy);
    depth_info_msg->K[8] = 1.0;
    depth_info_msg->R.fill(0.0);

    for (size_t i = 0; i < 3; i++) {
        // identity
        depth_info_msg->R[i + i * 3] = 1;
    }

    depth_info_msg->P.fill(0.0);
    depth_info_msg->P[0] = static_cast<double>(zedParam.left_cam.fx);
    depth_info_msg->P[2] = static_cast<double>(zedParam.left_cam.cx);
    depth_info_msg->P[5] = static_cast<double>(zedParam.left_cam.fy);
    depth_info_msg->P[6] = static_cast<double>(zedParam.left_cam.cy);
    depth_info_msg->P[10] = 1.0;
    // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    depth_info_msg->width = static_cast<uint32_t>(mMatResolDepth.width);
    depth_info_msg->height = static_cast<uint32_t>(mMatResolDepth.height);
    depth_info_msg->header.frame_id = frame_id;
}

void ZEDWrapperNodelet::updateDynamicReconfigure()
{
    // NODELET_DEBUG_STREAM( "updateDynamicReconfigure MUTEX LOCK");
    mDynParMutex.lock();
    zed_nodelets::ZedConfig config;

    config.auto_exposure_gain = mCamAutoExposure;
    config.auto_whitebalance = mCamAutoWB;
    config.brightness = mCamBrightness;
    config.depth_confidence = mCamDepthConfidence;
    config.depth_texture_conf = mCamDepthTextureConf;
    config.contrast = mCamContrast;
    config.exposure = mCamExposure;
    config.gain = mCamGain;
    config.hue = mCamHue;
    config.saturation = mCamSaturation;
    config.sharpness = mCamSharpness;
    config.gamma = mCamGamma;
    config.whitebalance_temperature = mCamWB / 100;
    config.point_cloud_freq = mPointCloudFreq;
    config.pub_frame_rate = mVideoDepthFreq;
    mDynParMutex.unlock();

    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();

    mUpdateDynParams = false;

    // NODELET_DEBUG_STREAM( "updateDynamicReconfigure MUTEX UNLOCK");
}

void ZEDWrapperNodelet::callback_dynamicReconf(zed_nodelets::ZedConfig& config, uint32_t level)
{
    // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX LOCK");
    mDynParMutex.lock();
    DynParams param = static_cast<DynParams>(level);

    switch (param) {
    case DATAPUB_FREQ:
        if (config.pub_frame_rate > mCamFrameRate) {
            mVideoDepthFreq = mCamFrameRate;
            NODELET_WARN_STREAM("'pub_frame_rate' cannot be major than camera grabbing framerate. Set to "
                << mVideoDepthFreq);

            mUpdateDynParams = true;
        } else {
            mVideoDepthFreq = config.pub_frame_rate;
            NODELET_INFO("Reconfigure Video and Depth pub. frequency: %g", mVideoDepthFreq);
        }

        mVideoDepthTimer.setPeriod(ros::Duration(1.0 / mVideoDepthFreq));

        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case CONFIDENCE:
        mCamDepthConfidence = config.depth_confidence;
        NODELET_INFO("Reconfigure confidence threshold: %d", mCamDepthConfidence);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case TEXTURE_CONF:
        mCamDepthTextureConf = config.depth_texture_conf;
        NODELET_INFO("Reconfigure texture confidence threshold: %d", mCamDepthTextureConf);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case POINTCLOUD_FREQ:
        if (config.point_cloud_freq > mCamFrameRate) {
            mPointCloudFreq = mCamFrameRate;
            NODELET_WARN_STREAM("'point_cloud_freq' cannot be major than camera grabbing framerate. Set to "
                << mPointCloudFreq);

            mUpdateDynParams = true;
        } else {
            mPointCloudFreq = config.point_cloud_freq;
            NODELET_INFO("Reconfigure point cloud pub. frequency: %g", mPointCloudFreq);
        }

        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case BRIGHTNESS:
        mCamBrightness = config.brightness;
        NODELET_INFO("Reconfigure image brightness: %d", mCamBrightness);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case CONTRAST:
        mCamContrast = config.contrast;
        NODELET_INFO("Reconfigure image contrast: %d", mCamContrast);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case HUE:
        mCamHue = config.hue;
        NODELET_INFO("Reconfigure image hue: %d", mCamHue);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case SATURATION:
        mCamSaturation = config.saturation;
        NODELET_INFO("Reconfigure image saturation: %d", mCamSaturation);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case SHARPNESS:
        mCamSharpness = config.sharpness;
        NODELET_INFO("Reconfigure image sharpness: %d", mCamSharpness);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case GAMMA:
#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 1)
        mCamGamma = config.gamma;
        NODELET_INFO("Reconfigure image gamma: %d", mCamGamma);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
#else
        NODELET_DEBUG_STREAM("Gamma Control is not available for SDK older that v3.1");
        mDynParMutex.unlock();
#endif
        break;

    case AUTO_EXP_GAIN:
        mCamAutoExposure = config.auto_exposure_gain;
        NODELET_INFO_STREAM("Reconfigure auto exposure/gain: " << mCamAutoExposure ? "ENABLED" : "DISABLED");
        if (!mCamAutoExposure) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
            mTriggerAutoExposure = false;
        } else {
            mTriggerAutoExposure = true;
        }
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case GAIN:
        mCamGain = config.gain;
        if (mCamAutoExposure) {
            NODELET_WARN("Reconfigure gain has no effect if 'auto_exposure_gain' is enabled");
        } else {
            NODELET_INFO("Reconfigure gain: %d", mCamGain);
        }
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case EXPOSURE:
        mCamExposure = config.exposure;
        if (mCamAutoExposure) {
            NODELET_WARN("Reconfigure exposure has no effect if 'auto_exposure_gain' is enabled");
        } else {
            NODELET_INFO("Reconfigure exposure: %d", mCamExposure);
        }
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case AUTO_WB:
        mCamAutoWB = config.auto_whitebalance;
        NODELET_INFO_STREAM("Reconfigure auto white balance: " << mCamAutoWB ? "ENABLED" : "DISABLED");
        if (!mCamAutoWB) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
            mTriggerAutoWB = false;
        } else {
            mTriggerAutoWB = true;
        }
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case WB_TEMP:
        mCamWB = config.whitebalance_temperature * 100;
        if (mCamAutoWB) {
            NODELET_WARN("Reconfigure white balance temperature has no effect if 'auto_whitebalance' is enabled");
        } else {
            NODELET_INFO("Reconfigure white balance temperature: %d", mCamWB);
        }
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    default:
        NODELET_DEBUG_STREAM("dynamicReconfCallback Unknown param: " << level);
        mDynParMutex.unlock();
        // NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
    }
}

void ZEDWrapperNodelet::callback_pubVideoDepth(const ros::TimerEvent& e)
{
    static sl::Timestamp lastZedTs = 0; // Used to calculate stable publish frequency

    uint32_t rgbSubnumber = mPubRgb.getNumSubscribers();
    uint32_t rgbRawSubnumber = mPubRawRgb.getNumSubscribers();
    uint32_t leftSubnumber = mPubLeft.getNumSubscribers();
    uint32_t leftRawSubnumber = mPubRawLeft.getNumSubscribers();
    uint32_t rightSubnumber = mPubRight.getNumSubscribers();
    uint32_t rightRawSubnumber = mPubRawRight.getNumSubscribers();
    uint32_t rgbGraySubnumber = mPubRgbGray.getNumSubscribers();
    uint32_t rgbGrayRawSubnumber = mPubRawRgbGray.getNumSubscribers();
    uint32_t leftGraySubnumber = mPubLeftGray.getNumSubscribers();
    uint32_t leftGrayRawSubnumber = mPubRawLeftGray.getNumSubscribers();
    uint32_t rightGraySubnumber = mPubRightGray.getNumSubscribers();
    uint32_t rightGrayRawSubnumber = mPubRawRightGray.getNumSubscribers();
    uint32_t depthSubnumber = mPubDepth.getNumSubscribers();
    uint32_t disparitySubnumber = mPubDisparity.getNumSubscribers();
    uint32_t confMapSubnumber = mPubConfMap.getNumSubscribers();
    uint32_t stereoSubNumber = mPubStereo.getNumSubscribers();
    uint32_t stereoRawSubNumber = mPubRawStereo.getNumSubscribers();

    uint32_t tot_sub = rgbSubnumber + rgbRawSubnumber + leftSubnumber + leftRawSubnumber + rightSubnumber + rightRawSubnumber + rgbGraySubnumber + rgbGrayRawSubnumber + leftGraySubnumber + leftGrayRawSubnumber + rightGraySubnumber + rightGrayRawSubnumber + depthSubnumber + disparitySubnumber + confMapSubnumber + stereoSubNumber + stereoRawSubNumber;

    bool retrieved = false;

    sl::Mat mat_left, mat_left_raw;
    sl::Mat mat_right, mat_right_raw;
    sl::Mat mat_left_gray, mat_left_raw_gray;
    sl::Mat mat_right_gray, mat_right_raw_gray;
    sl::Mat mat_depth, mat_disp, mat_conf;

    sl::Timestamp ts_rgb = 0; // used to check RGB/Depth sync
    sl::Timestamp ts_depth = 0; // used to check RGB/Depth sync
    sl::Timestamp grab_ts = 0;

    mCamDataMutex.lock();

    // ----> Retrieve all required image data
    if (rgbSubnumber + leftSubnumber + stereoSubNumber > 0) {
        mZed.retrieveImage(mat_left, sl::VIEW::LEFT, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        ts_rgb = mat_left.timestamp;
        grab_ts = mat_left.timestamp;
    }
    if (rgbRawSubnumber + leftRawSubnumber + stereoRawSubNumber > 0) {
        mZed.retrieveImage(mat_left_raw, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        grab_ts = mat_left_raw.timestamp;
    }
    if (rightSubnumber + stereoSubNumber > 0) {
        mZed.retrieveImage(mat_right, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        grab_ts = mat_right.timestamp;
    }
    if (rightRawSubnumber + stereoRawSubNumber > 0) {
        mZed.retrieveImage(mat_right_raw, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        grab_ts = mat_right_raw.timestamp;
    }
    if (rgbGraySubnumber + leftGraySubnumber > 0) {
        mZed.retrieveImage(mat_left_gray, sl::VIEW::LEFT_GRAY, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        grab_ts = mat_left_gray.timestamp;
    }
    if (rgbGrayRawSubnumber + leftGrayRawSubnumber > 0) {
        mZed.retrieveImage(mat_left_raw_gray, sl::VIEW::LEFT_UNRECTIFIED_GRAY, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        grab_ts = mat_left_raw_gray.timestamp;
    }
    if (rightGraySubnumber > 0) {
        mZed.retrieveImage(mat_right_gray, sl::VIEW::RIGHT_GRAY, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        grab_ts = mat_right_gray.timestamp;
    }
    if (rightGrayRawSubnumber > 0) {
        mZed.retrieveImage(mat_right_raw_gray, sl::VIEW::RIGHT_UNRECTIFIED_GRAY, sl::MEM::CPU, mMatResolVideo);
        retrieved = true;
        grab_ts = mat_right_raw_gray.timestamp;
    }
    if (depthSubnumber > 0) {
#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 4)
        mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResolDepth);
#else
        if (!mOpenniDepthMode) {
            mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResolDepth);
        } else {
            mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU, mMatResolDepth);
        }
#endif
        retrieved = true;
        grab_ts = mat_depth.timestamp;

        ts_depth = mat_depth.timestamp;

        if (ts_rgb.data_ns != 0 && (ts_depth.data_ns != ts_rgb.data_ns)) {
            NODELET_WARN_STREAM("!!!!! DEPTH/RGB ASYNC !!!!! - Delta: " << 1e-9 * static_cast<double>(ts_depth - ts_rgb)
                                                                        << " sec");
        }
    }
    if (disparitySubnumber > 0) {
        mZed.retrieveMeasure(mat_disp, sl::MEASURE::DISPARITY, sl::MEM::CPU, mMatResolDepth);
        retrieved = true;
        grab_ts = mat_disp.timestamp;
    }
    if (confMapSubnumber > 0) {
        mZed.retrieveMeasure(mat_conf, sl::MEASURE::CONFIDENCE, sl::MEM::CPU, mMatResolDepth);
        retrieved = true;
        grab_ts = mat_conf.timestamp;
    }
    // <---- Retrieve all required image data

    // ----> Data ROS timestamp
    ros::Time stamp = sl_tools::slTime2Ros(grab_ts);
    if (mSvoMode) {
        stamp = ros::Time::now();
    }
    // <---- Data ROS timestamp

    // ----> Publish sensor data if sync is required by user or SVO
    if (mZedRealCamModel != sl::MODEL::ZED) {
        if (mSensTimestampSync) {
            // NODELET_INFO_STREAM("tot_sub: " << tot_sub << " - retrieved: " << retrieved << " -
            // grab_ts.data_ns!=lastZedTs.data_ns: " << (grab_ts.data_ns!=lastZedTs.data_ns));
            if (tot_sub > 0 && retrieved && (grab_ts.data_ns != lastZedTs.data_ns)) {
                // NODELET_INFO("CALLBACK");
                publishSensData(stamp);
            }
        } else if (mSvoMode) {
            publishSensData(stamp);
        }
    }
    // <---- Publish sensor data if sync is required by user or SVO

    mCamDataMutex.unlock();

    // ----> Notify grab thread that all data are synchronized and a new grab can be done
    // mRgbDepthDataRetrievedCondVar.notify_one();
    // mRgbDepthDataRetrieved = true;
    // <---- Notify grab thread that all data are synchronized and a new grab can be done

    if (!retrieved) {
        mPublishingData = false;
        lastZedTs = 0;
        return;
    }
    mPublishingData = true;

    // ----> Check if a grab has been done before publishing the same images
    if (grab_ts.data_ns == lastZedTs.data_ns) {
        // Data not updated by a grab calling in the grab thread
        return;
    }
    if (lastZedTs.data_ns != 0) {
        double period_sec = static_cast<double>(grab_ts.data_ns - lastZedTs.data_ns) / 1e9;
        // NODELET_DEBUG_STREAM( "PUBLISHING PERIOD: " << period_sec << " sec @" << 1./period_sec << " Hz") ;

        mVideoDepthPeriodMean_sec->addValue(period_sec);
        // NODELET_DEBUG_STREAM( "MEAN PUBLISHING PERIOD: " << mVideoDepthPeriodMean_sec->getMean() << " sec @"
        // << 1./mVideoDepthPeriodMean_sec->getMean() << " Hz") ;
    }
    lastZedTs = grab_ts;
    // <---- Check if a grab has been done before publishing the same images

    // Publish the left = rgb image if someone has subscribed to
    if (leftSubnumber > 0) {
        sensor_msgs::ImagePtr leftImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(leftImgMsg, mat_left, mPubLeft, mLeftCamInfoMsg, mLeftCamOptFrameId, stamp);
    }
    if (rgbSubnumber > 0) {
        sensor_msgs::ImagePtr rgbImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rgbImgMsg, mat_left, mPubRgb, mRgbCamInfoMsg, mDepthOptFrameId, stamp);
    }

    // Publish the left = rgb GRAY image if someone has subscribed to
    if (leftGraySubnumber > 0) {
        sensor_msgs::ImagePtr leftGrayImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(leftGrayImgMsg, mat_left_gray, mPubLeftGray, mLeftCamInfoMsg, mLeftCamOptFrameId, stamp);
    }
    if (rgbGraySubnumber > 0) {
        sensor_msgs::ImagePtr rgbGrayImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rgbGrayImgMsg, mat_left_gray, mPubRgbGray, mRgbCamInfoMsg, mDepthOptFrameId, stamp);
    }

    // Publish the left_raw = rgb_raw image if someone has subscribed to
    if (leftRawSubnumber > 0) {
        sensor_msgs::ImagePtr rawLeftImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rawLeftImgMsg, mat_left_raw, mPubRawLeft, mLeftCamInfoRawMsg, mLeftCamOptFrameId, stamp);
    }
    if (rgbRawSubnumber > 0) {
        sensor_msgs::ImagePtr rawRgbImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rawRgbImgMsg, mat_left_raw, mPubRawRgb, mRgbCamInfoRawMsg, mDepthOptFrameId, stamp);
    }

    // Publish the left_raw == rgb_raw GRAY image if someone has subscribed to
    if (leftGrayRawSubnumber > 0) {
        sensor_msgs::ImagePtr rawLeftGrayImgMsg = boost::make_shared<sensor_msgs::Image>();

        publishImage(rawLeftGrayImgMsg, mat_left_raw_gray, mPubRawLeftGray, mLeftCamInfoRawMsg, mLeftCamOptFrameId, stamp);
    }
    if (rgbGrayRawSubnumber > 0) {
        sensor_msgs::ImagePtr rawRgbGrayImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rawRgbGrayImgMsg, mat_left_raw_gray, mPubRawRgbGray, mRgbCamInfoRawMsg, mDepthOptFrameId, stamp);
    }

    // Publish the right image if someone has subscribed to
    if (rightSubnumber > 0) {
        sensor_msgs::ImagePtr rightImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rightImgMsg, mat_right, mPubRight, mRightCamInfoMsg, mRightCamOptFrameId, stamp);
    }

    // Publish the right image GRAY if someone has subscribed to
    if (rightGraySubnumber > 0) {
        sensor_msgs::ImagePtr rightGrayImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rightGrayImgMsg, mat_right_gray, mPubRightGray, mRightCamInfoMsg, mRightCamOptFrameId, stamp);
    }

    // Publish the right raw image if someone has subscribed to
    if (rightRawSubnumber > 0) {
        sensor_msgs::ImagePtr rawRightImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rawRightImgMsg, mat_right_raw, mPubRawRight, mRightCamInfoRawMsg, mRightCamOptFrameId, stamp);
    }

    // Publish the right raw image GRAY if someone has subscribed to
    if (rightGrayRawSubnumber > 0) {
        sensor_msgs::ImagePtr rawRightGrayImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishImage(rawRightGrayImgMsg, mat_right_raw_gray, mPubRawRightGray, mRightCamInfoRawMsg, mRightCamOptFrameId,
            stamp);
    }

    // Stereo couple side-by-side
    if (stereoSubNumber > 0) {
        sensor_msgs::ImagePtr stereoImgMsg = boost::make_shared<sensor_msgs::Image>();
        sl_tools::imagesToROSmsg(stereoImgMsg, mat_left, mat_right, mCameraFrameId, stamp);
        mPubStereo.publish(stereoImgMsg);
    }

    // Stereo RAW couple side-by-side
    if (stereoRawSubNumber > 0) {
        sensor_msgs::ImagePtr rawStereoImgMsg = boost::make_shared<sensor_msgs::Image>();
        sl_tools::imagesToROSmsg(rawStereoImgMsg, mat_left_raw, mat_right_raw, mCameraFrameId, stamp);
        mPubRawStereo.publish(rawStereoImgMsg);
    }

    // Publish the depth image if someone has subscribed to
    if (depthSubnumber > 0) {
        sensor_msgs::ImagePtr depthImgMsg = boost::make_shared<sensor_msgs::Image>();
        publishDepth(depthImgMsg, mat_depth, stamp);
    }

    // Publish the disparity image if someone has subscribed to
    if (disparitySubnumber > 0) {
        publishDisparity(mat_disp, stamp);
    }

    // Publish the confidence map if someone has subscribed to
    if (confMapSubnumber > 0) {
        sensor_msgs::ImagePtr confMapMsg = boost::make_shared<sensor_msgs::Image>();
        sl_tools::imageToROSmsg(confMapMsg, mat_conf, mConfidenceOptFrameId, stamp);
        mPubConfMap.publish(confMapMsg);
    }
}

void ZEDWrapperNodelet::callback_pubPath(const ros::TimerEvent& e)
{
    uint32_t mapPathSub = mPubMapPath.getNumSubscribers();
    uint32_t odomPathSub = mPubOdomPath.getNumSubscribers();

    geometry_msgs::PoseStamped odomPose;
    geometry_msgs::PoseStamped mapPose;

    odomPose.header.stamp = mFrameTimestamp;
    odomPose.header.frame_id = mMapFrameId; // frame
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
    mapPose.header.frame_id = mMapFrameId; // frame
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
            // NODELET_DEBUG_STREAM("Path vectors adding last available poses");
            mMapPath.push_back(mapPose);
            mOdomPath.push_back(odomPose);
        }
    } else {
        // NODELET_DEBUG_STREAM("No limit path vectors, adding last available poses");
        mMapPath.push_back(mapPose);
        mOdomPath.push_back(odomPose);
    }

    if (mapPathSub > 0) {
        nav_msgs::PathPtr mapPath = boost::make_shared<nav_msgs::Path>();
        mapPath->header.frame_id = mMapFrameId;
        mapPath->header.stamp = mFrameTimestamp;
        mapPath->poses = mMapPath;

        mPubMapPath.publish(mapPath);
    }

    if (odomPathSub > 0) {
        nav_msgs::PathPtr odomPath = boost::make_shared<nav_msgs::Path>();
        odomPath->header.frame_id = mMapFrameId;
        odomPath->header.stamp = mFrameTimestamp;
        odomPath->poses = mOdomPath;

        mPubOdomPath.publish(odomPath);
    }
}

void ZEDWrapperNodelet::sensors_thread_func()
{
    ros::Rate loop_rate(mSensPubRate);

    std::chrono::steady_clock::time_point prev_usec = std::chrono::steady_clock::now();

    int count_warn = 0;

    while (!mStopNode) {
        std::chrono::steady_clock::time_point start_elab = std::chrono::steady_clock::now();

        mCloseZedMutex.lock();
        if (!mZed.isOpened()) {
            mCloseZedMutex.unlock();
            loop_rate.sleep();
            continue;
        }

        publishSensData();
        mCloseZedMutex.unlock();

        if (!loop_rate.sleep()) {
            if (++count_warn > 10) {
                NODELET_INFO_THROTTLE(1.0, "Sensors thread is not synchronized with the Sensors rate");
                NODELET_INFO_STREAM_THROTTLE(1.0, "Expected cycle time: " << loop_rate.expectedCycleTime() << " - Real cycle time: " << loop_rate.cycleTime());
                NODELET_WARN_STREAM_THROTTLE(10.0, "Sensors data publishing takes longer (" << loop_rate.cycleTime() << " sec) than requested  by the Sensors rate (" << loop_rate.expectedCycleTime() << " sec). Please consider to "
                                                                                                                                                                                                          "lower the 'max_pub_rate' setting or to "
                                                                                                                                                                                                          "reduce the power requirements reducing "
                                                                                                                                                                                                          "the resolutions.");
            }

            loop_rate.reset();
        } else {
            count_warn = 0;
        }
    }

    NODELET_DEBUG("Sensors thread finished");
}

void ZEDWrapperNodelet::publishSensData(ros::Time t)
{
    // NODELET_INFO("publishSensData");

    uint32_t imu_SubNumber = mPubImu.getNumSubscribers();
    uint32_t imu_RawSubNumber = mPubImuRaw.getNumSubscribers();
    uint32_t imu_TempSubNumber = 0;
    uint32_t imu_MagSubNumber = 0;
    uint32_t pressSubNumber = 0;
    uint32_t tempLeftSubNumber = 0;
    uint32_t tempRightSubNumber = 0;

    if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i) {
        imu_TempSubNumber = mPubImuTemp.getNumSubscribers();
        imu_MagSubNumber = mPubImuMag.getNumSubscribers();
        pressSubNumber = mPubPressure.getNumSubscribers();
        tempLeftSubNumber = mPubTempL.getNumSubscribers();
        tempRightSubNumber = mPubTempR.getNumSubscribers();
    }

    uint32_t tot_sub = imu_SubNumber + imu_RawSubNumber + imu_TempSubNumber + imu_MagSubNumber + pressSubNumber + tempLeftSubNumber + tempRightSubNumber;

    if (tot_sub > 0) {
        mSensPublishing = true;
    } else {
        mSensPublishing = false;
    }

    bool sensors_data_published = false;

    ros::Time ts_imu;
    ros::Time ts_baro;
    ros::Time ts_mag;

    static ros::Time lastTs_imu = ros::Time();
    static ros::Time lastTs_baro = ros::Time();
    static ros::Time lastT_mag = ros::Time();

    sl::SensorsData sens_data;

    if (mSvoMode || mSensTimestampSync) {
        if (mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS) {
            NODELET_DEBUG("Not retrieved sensors data in IMAGE REFERENCE TIME");
            return;
        }
    } else {
        if (mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS) {
            NODELET_DEBUG("Not retrieved sensors data in CURRENT REFERENCE TIME");
            return;
        }
    }

    if (t != ros::Time(0)) {
        ts_imu = t;
        ts_baro = t;
        ts_mag = t;
    } else {
        ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
        ts_baro = sl_tools::slTime2Ros(sens_data.barometer.timestamp);
        ts_mag = sl_tools::slTime2Ros(sens_data.magnetometer.timestamp);
    }

    bool new_imu_data = ts_imu != lastTs_imu;
    bool new_baro_data = ts_baro != lastTs_baro;
    bool new_mag_data = ts_mag != lastT_mag;

    if (!new_imu_data && !new_baro_data && !new_mag_data) {
        NODELET_DEBUG("No updated sensors data");
        return;
    }

    // ----> Publish odometry tf only if enabled
    if (mPublishTf && mPosTrackingReady && new_imu_data) {
        //NODELET_DEBUG("Publishing TF");

        publishOdomFrame(mOdom2BaseTransf, ts_imu); // publish the base Frame in odometry frame

        if (mPublishMapTf) {
            publishPoseFrame(mMap2OdomTransf, ts_imu); // publish the odometry Frame in map frame
        }
    }
    // <---- Publish odometry tf only if enabled

    if (mPublishImuTf && !mStaticImuFramePublished) {
        NODELET_DEBUG("Publishing static IMU TF");
        publishStaticImuFrame();
    }

    if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i) {
        // Update temperatures for Diagnostic
        sens_data.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT, mTempLeft);
        sens_data.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT, mTempRight);
    }

    if (imu_TempSubNumber > 0 && new_imu_data) {
        lastTs_imu = ts_imu;

        sensor_msgs::TemperaturePtr imuTempMsg = boost::make_shared<sensor_msgs::Temperature>();

        imuTempMsg->header.stamp = ts_imu;

#ifdef DEBUG_SENS_TS
        static ros::Time old_ts;
        if (old_ts == imuTempMsg->header.stamp) {
            NODELET_WARN_STREAM("Publishing IMU data with old timestamp " << old_ts);
        }
        old_ts = imuTempMsg->header.stamp;
#endif

        imuTempMsg->header.frame_id = mImuFrameId;
        float imu_temp;
        sens_data.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, imu_temp);
        imuTempMsg->temperature = static_cast<double>(imu_temp);
        imuTempMsg->variance = 0.0;

        sensors_data_published = true;
        mPubImuTemp.publish(imuTempMsg);
    } /*else {
        NODELET_DEBUG("No new IMU temp.");
    }*/

    if (sens_data.barometer.is_available && new_baro_data) {
        lastTs_baro = ts_baro;

        if (pressSubNumber > 0) {
            sensor_msgs::FluidPressurePtr pressMsg = boost::make_shared<sensor_msgs::FluidPressure>();

            pressMsg->header.stamp = ts_baro;

#ifdef DEBUG_SENS_TS
            static ros::Time old_ts;
            if (old_ts == pressMsg->header.stamp) {
                NODELET_WARN_STREAM("Publishing BARO data with old timestamp " << old_ts);
            }
            old_ts = pressMsg->header.stamp;
#endif
            pressMsg->header.frame_id = mBaroFrameId;
            pressMsg->fluid_pressure = sens_data.barometer.pressure * 1e2; // Pascal
            pressMsg->variance = 1.0585e-2;

            sensors_data_published = true;
            mPubPressure.publish(pressMsg);
        }

        if (tempLeftSubNumber > 0) {
            sensor_msgs::TemperaturePtr tempLeftMsg = boost::make_shared<sensor_msgs::Temperature>();

            tempLeftMsg->header.stamp = ts_baro;

#ifdef DEBUG_SENS_TS
            static ros::Time old_ts;
            if (old_ts == tempLeftMsg->header.stamp) {
                NODELET_WARN_STREAM("Publishing BARO data with old timestamp " << old_ts);
            }
            old_ts = tempLeftMsg->header.stamp;
#endif

            tempLeftMsg->header.frame_id = mTempLeftFrameId;
            tempLeftMsg->temperature = static_cast<double>(mTempLeft);
            tempLeftMsg->variance = 0.0;

            sensors_data_published = true;
            mPubTempL.publish(tempLeftMsg);
        }

        if (tempRightSubNumber > 0) {
            sensor_msgs::TemperaturePtr tempRightMsg = boost::make_shared<sensor_msgs::Temperature>();

            tempRightMsg->header.stamp = ts_baro;

#ifdef DEBUG_SENS_TS
            static ros::Time old_ts;
            if (old_ts == tempRightMsg->header.stamp) {
                NODELET_WARN_STREAM("Publishing BARO data with old timestamp " << old_ts);
            }
            old_ts = tempRightMsg->header.stamp;
#endif

            tempRightMsg->header.frame_id = mTempRightFrameId;
            tempRightMsg->temperature = static_cast<double>(mTempRight);
            tempRightMsg->variance = 0.0;

            sensors_data_published = true;
            mPubTempR.publish(tempRightMsg);
        }
    } /*else {
        NODELET_DEBUG("No new BAROM. DATA");
    }*/

    if (imu_MagSubNumber > 0) {
        if (sens_data.magnetometer.is_available && new_mag_data) {
            lastT_mag = ts_mag;

            sensor_msgs::MagneticFieldPtr magMsg = boost::make_shared<sensor_msgs::MagneticField>();

            magMsg->header.stamp = ts_mag;

#ifdef DEBUG_SENS_TS
            static ros::Time old_ts;
            if (old_ts == magMsg->header.stamp) {
                NODELET_WARN_STREAM("Publishing MAG data with old timestamp " << old_ts);
            }
            old_ts = magMsg->header.stamp;
#endif

            magMsg->header.frame_id = mMagFrameId;
            magMsg->magnetic_field.x = sens_data.magnetometer.magnetic_field_calibrated.x * 1e-6; // Tesla
            magMsg->magnetic_field.y = sens_data.magnetometer.magnetic_field_calibrated.y * 1e-6; // Tesla
            magMsg->magnetic_field.z = sens_data.magnetometer.magnetic_field_calibrated.z * 1e-6; // Tesla
            magMsg->magnetic_field_covariance[0] = 0.039e-6;
            magMsg->magnetic_field_covariance[1] = 0.0f;
            magMsg->magnetic_field_covariance[2] = 0.0f;
            magMsg->magnetic_field_covariance[3] = 0.0f;
            magMsg->magnetic_field_covariance[4] = 0.037e-6;
            magMsg->magnetic_field_covariance[5] = 0.0f;
            magMsg->magnetic_field_covariance[6] = 0.0f;
            magMsg->magnetic_field_covariance[7] = 0.0f;
            magMsg->magnetic_field_covariance[8] = 0.047e-6;

            sensors_data_published = true;
            mPubImuMag.publish(magMsg);
        }
    } /*else {
        NODELET_DEBUG("No new MAG. DATA");
    }*/

    if (imu_SubNumber > 0 && new_imu_data) {
        lastTs_imu = ts_imu;

        sensor_msgs::ImuPtr imuMsg = boost::make_shared<sensor_msgs::Imu>();

        imuMsg->header.stamp = ts_imu;

#ifdef DEBUG_SENS_TS
        static ros::Time old_ts;
        if (old_ts == imuMsg->header.stamp) {
            NODELET_WARN_STREAM("Publishing IMU data with old timestamp " << old_ts);
        } else {
            NODELET_INFO_STREAM("Publishing IMU data with new timestamp. Freq: " << 1. / (ts_imu.toSec() - old_ts.toSec()));
            old_ts = imuMsg->header.stamp;
        }
#endif

        imuMsg->header.frame_id = mImuFrameId;

        imuMsg->orientation.x = sens_data.imu.pose.getOrientation()[0];
        imuMsg->orientation.y = sens_data.imu.pose.getOrientation()[1];
        imuMsg->orientation.z = sens_data.imu.pose.getOrientation()[2];
        imuMsg->orientation.w = sens_data.imu.pose.getOrientation()[3];

        imuMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
        imuMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
        imuMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

        imuMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
        imuMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
        imuMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

        for (int i = 0; i < 3; ++i) {
            int r = 0;

            if (i == 0) {
                r = 0;
            } else if (i == 1) {
                r = 1;
            } else {
                r = 2;
            }

            imuMsg->orientation_covariance[i * 3 + 0] = sens_data.imu.pose_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
            imuMsg->orientation_covariance[i * 3 + 1] = sens_data.imu.pose_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
            imuMsg->orientation_covariance[i * 3 + 2] = sens_data.imu.pose_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;

            imuMsg->linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
            imuMsg->linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
            imuMsg->linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

            imuMsg->angular_velocity_covariance[i * 3 + 0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
            imuMsg->angular_velocity_covariance[i * 3 + 1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
            imuMsg->angular_velocity_covariance[i * 3 + 2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
        }

        sensors_data_published = true;
        mPubImu.publish(imuMsg);
    } /*else {
        NODELET_DEBUG("No new IMU DATA");
    }*/

    if (imu_RawSubNumber > 0 && new_imu_data) {
        lastTs_imu = ts_imu;

        sensor_msgs::ImuPtr imuRawMsg = boost::make_shared<sensor_msgs::Imu>();

        imuRawMsg->header.stamp = ts_imu;
        imuRawMsg->header.frame_id = mImuFrameId;
        imuRawMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
        imuRawMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
        imuRawMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;
        imuRawMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
        imuRawMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
        imuRawMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

        for (int i = 0; i < 3; ++i) {
            int r = 0;

            if (i == 0) {
                r = 0;
            } else if (i == 1) {
                r = 1;
            } else {
                r = 2;
            }

            imuRawMsg->linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
            imuRawMsg->linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
            imuRawMsg->linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];
            imuRawMsg->angular_velocity_covariance[i * 3 + 0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
            imuRawMsg->angular_velocity_covariance[i * 3 + 1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
            imuRawMsg->angular_velocity_covariance[i * 3 + 2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
        }

        // Orientation data is not available in "data_raw" -> See ROS REP145
        // http://www.ros.org/reps/rep-0145.html#topics
        imuRawMsg->orientation_covariance[0] = -1;
        sensors_data_published = true;
        mPubImuRaw.publish(imuRawMsg);
    }

    // ----> Update Diagnostic
    if (sensors_data_published) {
        // Publish freq calculation
        static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

        double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        last_time = now;

        mSensPeriodMean_usec->addValue(elapsed_usec);
    }
    // <---- Update Diagnostic
}

void ZEDWrapperNodelet::device_poll_thread_func()
{
    ros::Rate loop_rate(mCamFrameRate);

    mRecording = false;

    mElabPeriodMean_sec.reset(new sl_tools::CSmartMean(mCamFrameRate));
    mGrabPeriodMean_usec.reset(new sl_tools::CSmartMean(mCamFrameRate));
    mVideoDepthPeriodMean_sec.reset(new sl_tools::CSmartMean(mCamFrameRate));
    mPcPeriodMean_usec.reset(new sl_tools::CSmartMean(mCamFrameRate));
    mObjDetPeriodMean_msec.reset(new sl_tools::CSmartMean(mCamFrameRate));

    // Timestamp initialization
    if (mSvoMode) {
        mFrameTimestamp = ros::Time::now();
    } else {
        mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
    }

    mPrevFrameTimestamp = mFrameTimestamp;

    mPosTrackingActivated = false;
    mMappingRunning = false;
    mRecording = false;

    // Get the parameters of the ZED images
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    mCamWidth = mZed.getCameraInformation().camera_resolution.width;
    mCamHeight = mZed.getCameraInformation().camera_resolution.height;
#else
    mCamWidth = mZed.getCameraInformation().camera_configuration.resolution.width;
    mCamHeight = mZed.getCameraInformation().camera_configuration.resolution.height;
#endif
    NODELET_DEBUG_STREAM("Camera Frame size : " << mCamWidth << "x" << mCamHeight);
    int v_w = static_cast<int>(mCamWidth * mCamImageResizeFactor);
    int v_h = static_cast<int>(mCamHeight * mCamImageResizeFactor);
    mMatResolVideo = sl::Resolution(v_w, v_h);
    NODELET_DEBUG_STREAM("Image Mat size : " << mMatResolVideo.width << "x" << mMatResolVideo.height);
    int d_w = static_cast<int>(mCamWidth * mCamDepthResizeFactor);
    int d_h = static_cast<int>(mCamHeight * mCamDepthResizeFactor);
    mMatResolDepth = sl::Resolution(d_w, d_h);
    NODELET_DEBUG_STREAM("Depth Mat size : " << mMatResolDepth.width << "x" << mMatResolDepth.height);

    // Create and fill the camera information messages
    fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
    fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
    fillCamDepthInfo(mZed, mDepthCamInfoMsg, mLeftCamOptFrameId);

    // the reference camera is the Left one (next to the ZED logo)

    mRgbCamInfoMsg = mLeftCamInfoMsg;
    mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;

    sl::RuntimeParameters runParams;
    runParams.sensing_mode = static_cast<sl::SENSING_MODE>(mCamSensingMode);

    // Main loop
    while (mNhNs.ok()) {
        // Check for subscribers
        uint32_t rgbSubnumber = mPubRgb.getNumSubscribers();
        uint32_t rgbRawSubnumber = mPubRawRgb.getNumSubscribers();
        uint32_t leftSubnumber = mPubLeft.getNumSubscribers();
        uint32_t leftRawSubnumber = mPubRawLeft.getNumSubscribers();
        uint32_t rightSubnumber = mPubRight.getNumSubscribers();
        uint32_t rightRawSubnumber = mPubRawRight.getNumSubscribers();
        uint32_t rgbGraySubnumber = mPubRgbGray.getNumSubscribers();
        uint32_t rgbGrayRawSubnumber = mPubRawRgbGray.getNumSubscribers();
        uint32_t leftGraySubnumber = mPubLeftGray.getNumSubscribers();
        uint32_t leftGrayRawSubnumber = mPubRawLeftGray.getNumSubscribers();
        uint32_t rightGraySubnumber = mPubRightGray.getNumSubscribers();
        uint32_t rightGrayRawSubnumber = mPubRawRightGray.getNumSubscribers();
        uint32_t depthSubnumber = mPubDepth.getNumSubscribers();
        uint32_t disparitySubnumber = mPubDisparity.getNumSubscribers();
        uint32_t cloudSubnumber = mPubCloud.getNumSubscribers();
        uint32_t fusedCloudSubnumber = mPubFusedCloud.getNumSubscribers();
        uint32_t poseSubnumber = mPubPose.getNumSubscribers();
        uint32_t poseCovSubnumber = mPubPoseCov.getNumSubscribers();
        uint32_t odomSubnumber = mPubOdom.getNumSubscribers();
        uint32_t confMapSubnumber = mPubConfMap.getNumSubscribers();
        uint32_t pathSubNumber = mPubMapPath.getNumSubscribers() + mPubOdomPath.getNumSubscribers();
        uint32_t stereoSubNumber = mPubStereo.getNumSubscribers();
        uint32_t stereoRawSubNumber = mPubRawStereo.getNumSubscribers();

        uint32_t objDetSubnumber = 0;
        if (mObjDetEnabled && mObjDetRunning) {
            objDetSubnumber = mPubObjDet.getNumSubscribers();
        }

        mGrabActive = mRecording || mStreaming || mMappingEnabled || mObjDetEnabled || mPosTrackingEnabled || mPosTrackingActivated || ((rgbSubnumber + rgbRawSubnumber + leftSubnumber + leftRawSubnumber + rightSubnumber + rightRawSubnumber + rgbGraySubnumber + rgbGrayRawSubnumber + leftGraySubnumber + leftGrayRawSubnumber + rightGraySubnumber + rightGrayRawSubnumber + depthSubnumber + disparitySubnumber + cloudSubnumber + poseSubnumber + poseCovSubnumber + odomSubnumber + confMapSubnumber /*+ imuSubnumber + imuRawsubnumber*/ + pathSubNumber + stereoSubNumber + stereoRawSubNumber + objDetSubnumber) > 0);

        // Run the loop only if there is some subscribers or SVO is active
        if (mGrabActive) {
            std::lock_guard<std::mutex> lock(mPosTrkMutex);

            // Note: ones tracking is started it is never stopped anymore to not lose tracking information
            bool computeTracking = (mPosTrackingEnabled || mPosTrackingActivated || mMappingEnabled || mObjDetEnabled || (mComputeDepth & mDepthStabilization) || poseSubnumber > 0 || poseCovSubnumber > 0 || odomSubnumber > 0 || pathSubNumber > 0);

            // Start the tracking?
            if ((computeTracking) && !mPosTrackingActivated && (mDepthMode != sl::DEPTH_MODE::NONE)) {
                start_pos_tracking();
            }

            // Start the mapping?
            mMappingMutex.lock();
            if (mMappingEnabled && !mMappingRunning) {
                start_3d_mapping();
            }
            mMappingMutex.unlock();

            // Start the object detection?
            mObjDetMutex.lock();
            if (mObjDetEnabled && !mObjDetRunning) {
                start_obj_detect();
            }
            mObjDetMutex.unlock();

            // Detect if one of the subscriber need to have the depth information
            mComputeDepth = mDepthMode != sl::DEPTH_MODE::NONE && (computeTracking || ((depthSubnumber + disparitySubnumber + cloudSubnumber + fusedCloudSubnumber + poseSubnumber + poseCovSubnumber + odomSubnumber + confMapSubnumber + objDetSubnumber) > 0));

            if (mComputeDepth) {
                runParams.confidence_threshold = mCamDepthConfidence;
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 2
                runParams.textureness_confidence_threshold = mCamDepthTextureConf;
#else
                runParams.texture_confidence_threshold = mCamDepthTextureConf;
#endif
                runParams.enable_depth = true; // Ask to compute the depth
            } else {
                runParams.enable_depth = false; // Ask to not compute the depth
            }

            std::chrono::steady_clock::time_point start_elab = std::chrono::steady_clock::now();

            mCamDataMutex.lock();
            mRgbDepthDataRetrieved = false;
            mGrabStatus = mZed.grab(runParams);
            mCamDataMutex.unlock();

            // cout << toString(grab_status) << endl;
            if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
                // Detect if a error occurred (for example:
                // the zed have been disconnected) and
                // re-initialize the ZED

                NODELET_INFO_STREAM_ONCE(toString(mGrabStatus));

                std::this_thread::sleep_for(std::chrono::milliseconds(1));

                if ((ros::Time::now() - mPrevFrameTimestamp).toSec() > 5 && !mSvoMode) {
                    mCloseZedMutex.lock();
                    mZed.close();
                    mCloseZedMutex.unlock();

                    mConnStatus = sl::ERROR_CODE::CAMERA_NOT_DETECTED;

                    while (mConnStatus != sl::ERROR_CODE::SUCCESS) {
                        if (!mNhNs.ok()) {
                            mStopNode = true;

                            std::lock_guard<std::mutex> lock(mCloseZedMutex);
                            NODELET_DEBUG("Closing ZED");
                            if (mRecording) {
                                mRecording = false;
                                mZed.disableRecording();
                            }
                            mZed.close();

                            NODELET_DEBUG("ZED pool thread finished");
                            return;
                        }

                        int id = sl_tools::checkCameraReady(mZedSerialNumber);

                        if (id >= 0) {
                            mZedParams.input.setFromCameraID(id);
                            mConnStatus = mZed.open(mZedParams); // Try to initialize the ZED
                            NODELET_INFO_STREAM(toString(mConnStatus));
                        } else {
                            NODELET_INFO_STREAM("Waiting for the ZED (S/N " << mZedSerialNumber << ") to be re-connected");
                        }

                        mDiagUpdater.force_update();
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    }

                    mPosTrackingActivated = false;

                    computeTracking = mPosTrackingEnabled || mDepthStabilization || poseSubnumber > 0 || poseCovSubnumber > 0 || odomSubnumber > 0;

                    if (computeTracking) { // Start the tracking
                        start_pos_tracking();
                    }
                }

                mDiagUpdater.update();

                continue;
            }

            mFrameCount++;

            // SVO recording
            mRecMutex.lock();

            if (mRecording) {
                mRecStatus = mZed.getRecordingStatus();

                if (!mRecStatus.status) {
                    NODELET_ERROR_THROTTLE(1.0, "Error saving frame to SVO");
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

            // NODELET_INFO_STREAM("Grab time: " << elapsed_usec / 1000 << " msec");

            // Timestamp
            if (mSvoMode) {
                mFrameTimestamp = ros::Time::now();
            } else {
                mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
            }

            ros::Time stamp = mFrameTimestamp; // Timestamp

            // ----> Camera Settings
            if (!mSvoMode && mFrameCount % 5 == 0) {
                // NODELET_DEBUG_STREAM( "[" << mFrameCount << "] device_poll_thread_func MUTEX LOCK");
                mDynParMutex.lock();

                int brightness = mZed.getCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS);
                if (brightness != mCamBrightness) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, mCamBrightness);
                    NODELET_DEBUG_STREAM("mCamBrightness changed: " << mCamBrightness << " <- " << brightness);
                    mUpdateDynParams = true;
                }

                int contrast = mZed.getCameraSettings(sl::VIDEO_SETTINGS::CONTRAST);
                if (contrast != mCamContrast) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, mCamContrast);
                    NODELET_DEBUG_STREAM("mCamContrast changed: " << mCamContrast << " <- " << contrast);
                    mUpdateDynParams = true;
                }

                int hue = mZed.getCameraSettings(sl::VIDEO_SETTINGS::HUE);
                if (hue != mCamHue) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, mCamHue);
                    NODELET_DEBUG_STREAM("mCamHue changed: " << mCamHue << " <- " << hue);
                    mUpdateDynParams = true;
                }

                int saturation = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SATURATION);
                if (saturation != mCamSaturation) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, mCamSaturation);
                    NODELET_DEBUG_STREAM("mCamSaturation changed: " << mCamSaturation << " <- " << saturation);
                    mUpdateDynParams = true;
                }

                int sharpness = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS);
                if (sharpness != mCamSharpness) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, mCamSharpness);
                    NODELET_DEBUG_STREAM("mCamSharpness changed: " << mCamSharpness << " <- " << sharpness);
                    mUpdateDynParams = true;
                }

#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 1)
                int gamma = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAMMA);
                if (gamma != mCamGamma) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, mCamGamma);
                    NODELET_DEBUG_STREAM("mCamGamma changed: " << mCamGamma << " <- " << gamma);
                    mUpdateDynParams = true;
                }
#endif

                if (mCamAutoExposure) {
                    if (mTriggerAutoExposure) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 1);
                        mTriggerAutoExposure = false;
                    }
                } else {
                    int exposure = mZed.getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE);
                    if (exposure != mCamExposure) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, mCamExposure);
                        NODELET_DEBUG_STREAM("mCamExposure changed: " << mCamExposure << " <- " << exposure);
                        mUpdateDynParams = true;
                    }

                    int gain = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAIN);
                    if (gain != mCamGain) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, mCamGain);
                        NODELET_DEBUG_STREAM("mCamGain changed: " << mCamGain << " <- " << gain);
                        mUpdateDynParams = true;
                    }
                }

                if (mCamAutoWB) {
                    if (mTriggerAutoWB) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);
                        mTriggerAutoWB = false;
                    }
                } else {
                    int wb = mZed.getCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE);
                    if (wb != mCamWB) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, mCamWB);
                        NODELET_DEBUG_STREAM("mCamWB changed: " << mCamWB << " <- " << wb);
                        mUpdateDynParams = true;
                    }
                }
                mDynParMutex.unlock();
                // NODELET_DEBUG_STREAM( "device_poll_thread_func MUTEX UNLOCK");
            }

            if (mUpdateDynParams) {
                NODELET_DEBUG("Update Dynamic Parameters");
                updateDynamicReconfigure();
            }
            // <---- Camera Settings

            // Publish the point cloud if someone has subscribed to
            if (cloudSubnumber > 0) {
                // Run the point cloud conversion asynchronously to avoid slowing down
                // all the program
                // Retrieve raw pointCloud data if latest Pointcloud is ready
                std::unique_lock<std::mutex> lock(mPcMutex, std::defer_lock);

                if (lock.try_lock()) {
                    mZed.retrieveMeasure(mCloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU, mMatResolDepth);

                    mPointCloudFrameId = mDepthFrameId;
                    mPointCloudTime = stamp;

                    // Signal Pointcloud thread that a new pointcloud is ready
                    mPcDataReadyCondVar.notify_one();
                    mPcDataReady = true;
                    mPcPublishing = true;
                }
            } else {
                mPcPublishing = false;
            }

            mObjDetMutex.lock();
            if (mObjDetRunning && objDetSubnumber > 0) {
                processDetectedObjects(stamp);
            }
            mObjDetMutex.unlock();

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
                    mPosTrackingStatus = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME::CAMERA);

                    sl::Translation translation = deltaOdom.getTranslation();
                    sl::Orientation quat = deltaOdom.getOrientation();

#if 0
                    NODELET_DEBUG("delta ODOM [%s] - %.2f,%.2f,%.2f %.2f,%.2f,%.2f,%.2f",
                                  sl::toString(mTrackingStatus).c_str(),
                                  translation(0), translation(1), translation(2),
                                  quat(0), quat(1), quat(2), quat(3));

                    NODELET_DEBUG_STREAM("ODOM -> Tracking Status: " << sl::toString(mTrackingStatus));
#endif

                    if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK || mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING || mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW) {
                        // Transform ZED delta odom pose in TF2 Transformation
                        geometry_msgs::Transform deltaTransf;
                        deltaTransf.translation.x = translation(0);
                        deltaTransf.translation.y = translation(1);
                        deltaTransf.translation.z = translation(2);
                        deltaTransf.rotation.x = quat(0);
                        deltaTransf.rotation.y = quat(1);
                        deltaTransf.rotation.z = quat(2);
                        deltaTransf.rotation.w = quat(3);
                        tf2::Transform deltaOdomTf;
                        tf2::fromMsg(deltaTransf, deltaOdomTf);
                        // delta odom from sensor to base frame
                        tf2::Transform deltaOdomTf_base = mSensor2BaseTransf.inverse() * deltaOdomTf * mSensor2BaseTransf;

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
                            publishOdom(mOdom2BaseTransf, deltaOdom, stamp);
                        }

                        mPosTrackingReady = true;
                    }
                } else if (mFloorAlignment) {
                    NODELET_WARN_THROTTLE(5.0, "Odometry will be published as soon as the floor as been detected for the first "
                                               "time");
                }
            }

            // Publish the zed camera pose if someone has subscribed to
            if (computeTracking) {
                mPosTrackingStatus = mZed.getPosition(mLastZedPose, sl::REFERENCE_FRAME::WORLD);

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

                static sl::POSITIONAL_TRACKING_STATE old_tracking_state = sl::POSITIONAL_TRACKING_STATE::OFF;
                if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK || mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING
                    /*|| status == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW*/) {
                    if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK && mPosTrackingStatus != old_tracking_state) {
                        NODELET_INFO_STREAM("Positional tracking -> OK [" << sl::toString(mPosTrackingStatus).c_str() << "]");
                        old_tracking_state = mPosTrackingStatus;
                    }
                    if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING && mPosTrackingStatus != old_tracking_state) {
                        NODELET_INFO_STREAM("Positional tracking -> Searching for a known position ["
                            << sl::toString(mPosTrackingStatus).c_str() << "]");
                        old_tracking_state = mPosTrackingStatus;
                    }
                    // Transform ZED pose in TF2 Transformation
                    geometry_msgs::Transform map2sensTransf;

                    map2sensTransf.translation.x = translation(0);
                    map2sensTransf.translation.y = translation(1);
                    map2sensTransf.translation.z = translation(2);
                    map2sensTransf.rotation.x = quat(0);
                    map2sensTransf.rotation.y = quat(1);
                    map2sensTransf.rotation.z = quat(2);
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

          NODELET_DEBUG("*** Base POSE [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}", mMapFrameId.c_str(),
                        mBaseFrameId.c_str(), mMap2BaseTransf.getOrigin().x(), mMap2BaseTransf.getOrigin().y(),
                        mMap2BaseTransf.getOrigin().z(), roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

                    bool initOdom = false;

                    if (!(mFloorAlignment)) {
                        initOdom = mInitOdomWithPose;
                    } else {
                        initOdom = (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK) & mInitOdomWithPose;
                    }

                    if (initOdom || mResetOdom) {
                        NODELET_INFO("Odometry aligned to last tracking pose");

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
                        // mMap2OdomTransf = mOdom2BaseTransf.inverse() * mMap2BaseTransf;
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
                        publishPose(stamp);
                    }

                    mPosTrackingReady = true;
                }
            }

            if (mZedRealCamModel == sl::MODEL::ZED) {
                // Publish pose tf only if enabled
                if (mPublishTf) {
                    // Note, the frame is published, but its values will only change if
                    // someone has subscribed to odom
                    publishOdomFrame(mOdom2BaseTransf, stamp); // publish the base Frame in odometry frame

                    if (mPublishMapTf) {
                        // Note, the frame is published, but its values will only change if
                        // someone has subscribed to map
                        publishPoseFrame(mMap2OdomTransf, stamp); // publish the odometry Frame in map frame
                    }
                }
            }

#if 0 //#ifndef NDEBUG // Enable for TF checking \
    // Double check: map_to_pose must be equal to mMap2BaseTransf

            tf2::Transform map_to_base;

            try {
                // Save the transformation from base to frame
                geometry_msgs::TransformStamped b2m =
                        mTfBuffer->lookupTransform(mMapFrameId, mBaseFrameId, ros::Time(0));
                // Get the TF2 transformation
                tf2::fromMsg(b2m.transform, map_to_base);
            } catch (tf2::TransformException& ex) {
                NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
                NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.",
                                      mMapFrameId.c_str(), mBaseFrameId.c_str());
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
                        NODELET_DEBUG_THROTTLE(1.0, "Working thread is not synchronized with the Camera frame rate");
                        NODELET_DEBUG_STREAM_THROTTLE(1.0, "Expected cycle time: " << loop_rate.expectedCycleTime() << " - Real cycle time: " << mean_elab_sec);
                        NODELET_WARN_STREAM_THROTTLE(10.0, "Elaboration takes longer (" << mean_elab_sec << " sec) than requested "
                                                                                                            "by the FPS rate ("
                                                                                        << loop_rate.expectedCycleTime() << " sec). Please consider to "
                                                                                                                            "lower the 'frame_rate' setting or to "
                                                                                                                            "reduce the power requirements reducing "
                                                                                                                            "the resolutions.");
                    }

                    loop_rate.reset();
                } else {
                    count_warn = 0;
                }
            }
        } else {
            NODELET_DEBUG_THROTTLE(5.0, "No topics subscribed by users");

            if (mZedRealCamModel == sl::MODEL::ZED || !mPublishImuTf) {
                // Publish odometry tf only if enabled
                if (mPublishTf) {
                    ros::Time t;

                    if (mSvoMode) {
                        t = ros::Time::now();
                    } else {
                        t = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
                    }

                    publishOdomFrame(mOdom2BaseTransf, mFrameTimestamp); // publish the base Frame in odometry frame

                    if (mPublishMapTf) {
                        publishPoseFrame(mMap2OdomTransf, mFrameTimestamp); // publish the odometry Frame in map frame
                    }

                    if (mPublishImuTf && !mStaticImuFramePublished) {
                        publishStaticImuFrame();
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // No subscribers, we just wait
            loop_rate.reset();
        }

        mDiagUpdater.update();
    } // while loop

    if (mSaveAreaMapOnClosing && mPosTrackingActivated) {
        saveAreaMap(mAreaMemDbPath);
    }

    mStopNode = true; // Stops other threads

    std::lock_guard<std::mutex> lock(mCloseZedMutex);
    NODELET_DEBUG("Closing ZED");

    if (mRecording) {
        mRecording = false;
        mZed.disableRecording();
    }
    mStopNode = true;
    mZed.close();

    NODELET_DEBUG("ZED pool thread finished");
}

void ZEDWrapperNodelet::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (mConnStatus != sl::ERROR_CODE::SUCCESS) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, sl::toString(mConnStatus).c_str());
        return;
    }

    if (mGrabActive) {
        if (mGrabStatus == sl::ERROR_CODE::SUCCESS /*|| mGrabStatus == sl::ERROR_CODE::NOT_A_NEW_FRAME*/) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera grabbing");

            double freq = 1000000. / mGrabPeriodMean_usec->getMean();
            double freq_perc = 100. * freq / mCamFrameRate;
            stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

            stat.addf("General Processing", "Mean Time: %.3f sec (Max. %.3f sec)", mElabPeriodMean_sec->getMean(),
                1. / mCamFrameRate);

            if (mPublishingData) {
                freq = 1. / mVideoDepthPeriodMean_sec->getMean();
                freq_perc = 100. * freq / mVideoDepthFreq;
                stat.addf("Video/Depth Publish", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
            }

            if (mSvoMode) {
                int frame = mZed.getSVOPosition();
                int totFrames = mZed.getSVONumberOfFrames();
                double svo_perc = 100. * (static_cast<double>(frame) / totFrames);

                stat.addf("Playing SVO", "Frame: %d/%d (%.1f%%)", frame, totFrames, svo_perc);
            }

            if (mComputeDepth) {
                stat.add("Depth status", "ACTIVE");

                if (mPcPublishing) {
                    double freq = 1000000. / mPcPeriodMean_usec->getMean();
                    double freq_perc = 100. * freq / mPointCloudFreq;
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

                if (mPosTrackingActivated) {
                    stat.addf("Tracking status", "%s", sl::toString(mPosTrackingStatus).c_str());
                } else {
                    stat.add("Pos. Tracking status", "INACTIVE");
                }

                if (mObjDetRunning) {
                    double freq = 1000. / mObjDetPeriodMean_msec->getMean();
                    double freq_perc = 100. * freq / mCamFrameRate;
                    stat.addf("Object detection", "Mean Frequency: %.3f Hz  (%.1f%%)", freq, freq_perc);
                } else {
                    stat.add("Object Detection", "INACTIVE");
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

    if (mSensPublishing) {
        double freq = 1000000. / mSensPeriodMean_usec->getMean();
        double freq_perc = 100. * freq / mSensPubRate;
        stat.addf("IMU", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
    } else {
        stat.add("IMU", "Topics not subscribed");
    }

    if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i) {
        stat.addf("Left CMOS Temp.", "%.1f °C", mTempLeft);
        stat.addf("Right CMOS Temp.", "%.1f °C", mTempRight);

        if (mTempLeft > 70.f || mTempRight > 70.f) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Camera temperature");
        }
    } else {
        stat.add("Left CMOS Temp.", "N/A");
        stat.add("Right CMOS Temp.", "N/A");
    }

    if (mRecording) {
        if (!mRecStatus.status) {
            if (mGrabActive) {
                stat.add("SVO Recording", "ERROR");
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Error adding frames to SVO file while recording. Check "
                                                                      "free disk space");
            } else {
                stat.add("SVO Recording", "WAITING");
            }
        } else {
            stat.add("SVO Recording", "ACTIVE");
            stat.addf("SVO compression time", "%g msec", mRecStatus.average_compression_time);
            stat.addf("SVO compression ratio", "%.1f%%", mRecStatus.average_compression_ratio);
        }
    } else {
        stat.add("SVO Recording", "NOT ACTIVE");
    }
}

bool ZEDWrapperNodelet::on_start_svo_recording(zed_interfaces::start_svo_recording::Request& req,
    zed_interfaces::start_svo_recording::Response& res)
{
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

    sl::RecordingParameters recParams;
    recParams.compression_mode = mSvoComprMode;
    recParams.video_filename = req.svo_filename.c_str();
    err = mZed.enableRecording(recParams);

    if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION) {
        recParams.compression_mode = mSvoComprMode == sl::SVO_COMPRESSION_MODE::H265 ? sl::SVO_COMPRESSION_MODE::H264 : sl::SVO_COMPRESSION_MODE::H265;

        NODELET_WARN_STREAM("The chosen " << sl::toString(mSvoComprMode).c_str() << "mode is not available. Trying "
                                          << sl::toString(recParams.compression_mode).c_str());

        err = mZed.enableRecording(recParams);

        if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION) {
            NODELET_WARN_STREAM(sl::toString(recParams.compression_mode).c_str()
                << "not available. Trying " << sl::toString(sl::SVO_COMPRESSION_MODE::H264).c_str());
            recParams.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
            err = mZed.enableRecording(recParams);

            if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION) {
                recParams.compression_mode = sl::SVO_COMPRESSION_MODE::LOSSLESS;
                err = mZed.enableRecording(recParams);
            }
        }
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
        res.result = false;
        res.info = sl::toString(err).c_str();
        mRecording = false;
        return false;
    }

    mSvoComprMode = recParams.compression_mode;
    mRecording = true;
    res.info = "Recording started (";
    res.info += sl::toString(mSvoComprMode).c_str();
    res.info += ")";
    res.result = true;

    NODELET_INFO_STREAM("SVO recording STARTED: " << req.svo_filename << " (" << sl::toString(mSvoComprMode).c_str()
                                                  << ")");

    return true;
}

bool ZEDWrapperNodelet::on_stop_svo_recording(zed_interfaces::stop_svo_recording::Request& req,
    zed_interfaces::stop_svo_recording::Response& res)
{
    std::lock_guard<std::mutex> lock(mRecMutex);

    if (!mRecording) {
        res.done = false;
        res.info = "Recording was not active";
        NODELET_WARN_STREAM("Can't stop SVO recording. Recording was not active");
        return false;
    }

    mZed.disableRecording();
    mRecording = false;
    res.info = "Recording stopped";
    res.done = true;

    NODELET_INFO_STREAM("SVO recording STOPPED");

    return true;
}

bool ZEDWrapperNodelet::on_start_remote_stream(zed_interfaces::start_remote_stream::Request& req,
    zed_interfaces::start_remote_stream::Response& res)
{
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

    if ((params.gop_size < -1) || (params.gop_size > 256)) {
        mStreaming = false;

        res.result = false;
        res.info = "`gop_size` wrong (";
        res.info += params.gop_size;
        res.info += "). Remote streaming not started";

        NODELET_ERROR_STREAM(res.info);
        return false;
    }

    if (params.port % 2 != 0) {
        mStreaming = false;

        res.result = false;
        res.info = "`port` must be an even number. Remote streaming not started";

        NODELET_ERROR_STREAM(res.info);
        return false;
    }

    sl::ERROR_CODE err = mZed.enableStreaming(params);

    if (err != sl::ERROR_CODE::SUCCESS) {
        mStreaming = false;

        res.result = false;
        res.info = sl::toString(err).c_str();

        NODELET_ERROR_STREAM("Remote streaming not started (" << res.info << ")");

        return false;
    }

    mStreaming = true;

    NODELET_INFO_STREAM("Remote streaming STARTED");

    res.result = true;
    res.info = "Remote streaming STARTED";
    return true;
}

bool ZEDWrapperNodelet::on_stop_remote_stream(zed_interfaces::stop_remote_stream::Request& req,
    zed_interfaces::stop_remote_stream::Response& res)
{
    if (mStreaming) {
        mZed.disableStreaming();
    }

    mStreaming = false;
    NODELET_INFO_STREAM("SVO remote streaming STOPPED");

    res.done = true;
    return true;
}

bool ZEDWrapperNodelet::on_set_led_status(zed_interfaces::set_led_status::Request& req,
    zed_interfaces::set_led_status::Response& res)
{
    if (mCamFwVersion < 1523) {
        NODELET_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
        return false;
    }

    mZed.setCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS, req.led_enabled ? 1 : 0);

    return true;
}

bool ZEDWrapperNodelet::on_toggle_led(zed_interfaces::toggle_led::Request& req,
    zed_interfaces::toggle_led::Response& res)
{
    if (mCamFwVersion < 1523) {
        NODELET_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
        return false;
    }

    int status = mZed.getCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS);
    int new_status = status == 0 ? 1 : 0;
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS, new_status);

    return (new_status == 1);
}

bool ZEDWrapperNodelet::on_start_3d_mapping(zed_interfaces::start_3d_mapping::Request& req,
    zed_interfaces::start_3d_mapping::Response& res)
{
    if (mMappingEnabled && mMappingRunning) {
        NODELET_WARN_STREAM("Spatial mapping was just running");

        res.done = false;
        return res.done;
    }

    mMappingRunning = false;

    mMappingRes = req.resolution;
    mMaxMappingRange = req.max_mapping_range;
    mFusedPcPubFreq = req.fused_pointcloud_freq;

    NODELET_DEBUG_STREAM(" * Received mapping resolution\t\t-> " << mMappingRes << " m");
    NODELET_DEBUG_STREAM(" * Received mapping max range\t-> " << mMaxMappingRange << " m");
    NODELET_DEBUG_STREAM(" * Received fused point cloud freq:\t-> " << mFusedPcPubFreq << " Hz");

    mMappingEnabled = true;
    res.done = true;

    return res.done;
}

bool ZEDWrapperNodelet::on_stop_3d_mapping(zed_interfaces::stop_3d_mapping::Request& req,
    zed_interfaces::stop_3d_mapping::Response& res)
{
    if (mMappingEnabled) {
        mPubFusedCloud.shutdown();
        mMappingMutex.lock();
        stop_3d_mapping();
        mMappingMutex.unlock();

        res.done = true;
    } else {
        res.done = false;
    }

    return res.done;
}

bool ZEDWrapperNodelet::on_save_3d_map(zed_interfaces::save_3d_map::Request& req,
    zed_interfaces::save_3d_map::Response& res)
{
    if (!mMappingEnabled) {
        res.result = false;
        res.info = "3D Mapping was not active";
        NODELET_WARN_STREAM("Can't save 3D map. Mapping was not active");
        return false;
    }

    mMapSave = true;

    std::lock_guard<std::mutex> lock(mMappingMutex);
    sl::String filename = req.map_filename.c_str();
    if (req.file_format < 0 || req.file_format > static_cast<int>(sl::MESH_FILE_FORMAT::OBJ)) {
        res.result = false;
        res.info = "File format not correct";
        NODELET_WARN_STREAM("Can't save 3D map. File format not correct");
        return false;
    }

    sl::MESH_FILE_FORMAT file_format = static_cast<sl::MESH_FILE_FORMAT>(req.file_format);

    bool success = mFusedPC.save(filename, file_format);

    if (!success) {
        res.result = false;
        res.info = "3D Map not saved";
        NODELET_ERROR_STREAM("3D Map not saved");
        return false;
    }

    res.info = "3D map saved";
    res.result = true;
    return true;
}

bool ZEDWrapperNodelet::on_start_object_detection(zed_interfaces::start_object_detection::Request& req,
    zed_interfaces::start_object_detection::Response& res)
{
    NODELET_INFO("Called 'start_object_detection' service");

    if (mZedRealCamModel == sl::MODEL::ZED) {
        mObjDetEnabled = false;
        mObjDetRunning = false;

        NODELET_ERROR_STREAM("Object detection not started. OD is not available for ZED camera model");
        res.done = false;
        return res.done;
    }

    if (mObjDetEnabled && mObjDetRunning) {
        NODELET_WARN_STREAM("Object Detection was just running");

        res.done = false;
        return res.done;
    }

    mObjDetRunning = false;

    mObjDetConfidence = req.confidence;
    mObjDetTracking = req.tracking;
    if (req.model < 0 || req.model >= static_cast<int>(sl::DETECTION_MODEL::LAST)) {
        NODELET_ERROR_STREAM("Object Detection model not valid.");
        res.done = false;
        return res.done;
    }
    mObjDetModel = static_cast<sl::DETECTION_MODEL>(req.model);

    mObjDetMaxRange = req.max_range;
    if (mObjDetMaxRange > mCamMaxDepth) {
        NODELET_WARN("Detection max range cannot be major than depth max range. Automatically fixed.");
        mObjDetMaxRange = mCamMaxDepth;
    }
    NODELET_INFO_STREAM(" * Detection max range\t\t-> " << mObjDetMaxRange);

    NODELET_INFO_STREAM(" * Object min. confidence\t-> " << mObjDetConfidence);
    NODELET_INFO_STREAM(" * Object tracking\t\t-> " << (mObjDetTracking ? "ENABLED" : "DISABLED"));
    NODELET_INFO_STREAM(" * Detection model\t\t-> " << sl::toString(mObjDetModel));

    if (mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE || mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_MEDIUM || mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_FAST) {
        mObjDetBodyFitting = req.sk_body_fitting;
        NODELET_INFO_STREAM(" * Body fitting\t\t\t-> " << (mObjDetBodyFitting ? "ENABLED" : "DISABLED"));
    } else {
        mObjDetPeopleEnable = req.mc_people;
        NODELET_INFO_STREAM(" * Detect people\t\t-> " << (mObjDetPeopleEnable ? "ENABLED" : "DISABLED"));
        mObjDetVehiclesEnable = req.mc_vehicles;
        NODELET_INFO_STREAM(" * Detect vehicles\t\t-> " << (mObjDetVehiclesEnable ? "ENABLED" : "DISABLED"));
        mObjDetBagsEnable = req.mc_bag;
        NODELET_INFO_STREAM(" * Detect bags\t\t\t-> " << (mObjDetBagsEnable ? "ENABLED" : "DISABLED"));
        mObjDetAnimalsEnable = req.mc_animal;
        NODELET_INFO_STREAM(" * Detect animals\t\t-> " << (mObjDetAnimalsEnable ? "ENABLED" : "DISABLED"));
        mObjDetElectronicsEnable = req.mc_electronics;
        NODELET_INFO_STREAM(" * Detect electronics\t\t-> " << (mObjDetElectronicsEnable ? "ENABLED" : "DISABLED"));
        mObjDetFruitsEnable = req.mc_fruit_vegetable;
        NODELET_INFO_STREAM(" * Detect fruit and vegetables\t-> " << (mObjDetFruitsEnable ? "ENABLED" : "DISABLED"));
        mObjDetSportsEnable = req.mc_sport;
        NODELET_INFO_STREAM(" * Detect sport related objects\t-> " << (mObjDetSportsEnable ? "ENABLED" : "DISABLED"));
    }

    mObjDetRunning = false;
    mObjDetEnabled = true;
    res.done = true;

    return res.done;
}

/*! \brief Service callback to stop_object_detection service
 */
bool ZEDWrapperNodelet::on_stop_object_detection(zed_interfaces::stop_object_detection::Request& req,
    zed_interfaces::stop_object_detection::Response& res)
{
    if (mObjDetEnabled) {
        mObjDetMutex.lock();
        stop_obj_detect();
        mObjDetMutex.unlock();

        res.done = true;
    } else {
        res.done = false;
    }

    return res.done;
}

void ZEDWrapperNodelet::processDetectedObjects(ros::Time t)
{
    static std::chrono::steady_clock::time_point old_time = std::chrono::steady_clock::now();

    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    objectTracker_parameters_rt.detection_confidence_threshold = mObjDetConfidence;
    objectTracker_parameters_rt.object_class_filter = mObjDetFilter;

    sl::Objects objects;

    sl::ERROR_CODE objDetRes = mZed.retrieveObjects(objects, objectTracker_parameters_rt);

    if (objDetRes != sl::ERROR_CODE::SUCCESS) {
        NODELET_WARN_STREAM("Object Detection error: " << sl::toString(objDetRes));
        return;
    }

    if (!objects.is_new) {
        return;
    }

    // ----> Diagnostic information update
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double elapsed_msec = std::chrono::duration_cast<std::chrono::milliseconds>(now - old_time).count();
    mObjDetPeriodMean_msec->addValue(elapsed_msec);
    old_time = now;
    // <---- Diagnostic information update

    NODELET_DEBUG_STREAM("Detected " << objects.object_list.size() << " objects");

    size_t objCount = objects.object_list.size();

    zed_interfaces::ObjectsStampedPtr objMsg = boost::make_shared<zed_interfaces::ObjectsStamped>();
    objMsg->header.stamp = t;
    objMsg->header.frame_id = mLeftCamFrameId;

    objMsg->objects.resize(objCount);

    size_t idx = 0;
    for (auto data : objects.object_list) {
        objMsg->objects[idx].label = sl::toString(data.label).c_str();
        objMsg->objects[idx].sublabel = sl::toString(data.sublabel).c_str();
        objMsg->objects[idx].label_id = data.id;
        objMsg->objects[idx].confidence = data.confidence;

        memcpy(&(objMsg->objects[idx].position[0]), &(data.position[0]), 3 * sizeof(float));
        memcpy(&(objMsg->objects[idx].position_covariance[0]), &(data.position_covariance[0]), 6 * sizeof(float));
        memcpy(&(objMsg->objects[idx].velocity[0]), &(data.velocity[0]), 3 * sizeof(float));

        objMsg->objects[idx].tracking_available = mObjDetTracking;
        objMsg->objects[idx].tracking_state = static_cast<int8_t>(data.tracking_state);
        // NODELET_INFO_STREAM( "[" << idx << "] Tracking: " <<
        // sl::toString(static_cast<sl::OBJECT_TRACKING_STATE>(data.tracking_state)));
        objMsg->objects[idx].action_state = static_cast<int8_t>(data.action_state);

        if (data.bounding_box_2d.size() == 4) {
            memcpy(&(objMsg->objects[idx].bounding_box_2d.corners[0]), &(data.bounding_box_2d[0]), 8 * sizeof(unsigned int));
        }
        if (data.bounding_box.size() == 8) {
            memcpy(&(objMsg->objects[idx].bounding_box_3d.corners[0]), &(data.bounding_box[0]), 24 * sizeof(float));
        }

        memcpy(&(objMsg->objects[idx].dimensions_3d[0]), &(data.dimensions[0]), 3 * sizeof(float));

        if (mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE || mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_MEDIUM || mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_FAST) {
            objMsg->objects[idx].skeleton_available = true;

            if (data.head_bounding_box_2d.size() == 4) {
                memcpy(&(objMsg->objects[idx].head_bounding_box_2d.corners[0]), &(data.head_bounding_box_2d[0]),
                    8 * sizeof(unsigned int));
            }
            if (data.head_bounding_box.size() == 8) {
                memcpy(&(objMsg->objects[idx].head_bounding_box_3d.corners[0]), &(data.head_bounding_box[0]),
                    24 * sizeof(float));
            }
            memcpy(&(objMsg->objects[idx].head_position[0]), &(data.head_position[0]), 3 * sizeof(float));

            if (data.keypoint_2d.size() == 18) {
                memcpy(&(objMsg->objects[idx].skeleton_2d.keypoints[0]), &(data.keypoint_2d[0]), 36 * sizeof(float));
            }
            if (data.keypoint_2d.size() == 18) {
                memcpy(&(objMsg->objects[idx].skeleton_3d.keypoints[0]), &(data.keypoint[0]), 54 * sizeof(float));
            }
        } else {
            objMsg->objects[idx].skeleton_available = false;
        }

        // at the end of the loop
        idx++;
    }

    mPubObjDet.publish(objMsg);
}

void ZEDWrapperNodelet::clickedPtCallback(geometry_msgs::PointStampedConstPtr msg)
{
    // ----> Check for result subscribers
    uint32_t markerSubNumber = mPubMarker.getNumSubscribers();
    uint32_t planeSubNumber = mPubMarker.getNumSubscribers();

    if ((markerSubNumber + planeSubNumber) == 0) {
        return;
    }
    // <---- Check for result subscribers

    ros::Time ts = ros::Time::now();

    float X = msg->point.x;
    float Y = msg->point.y;
    float Z = msg->point.z;

    NODELET_INFO_STREAM("Clicked 3D point [X FW, Y LF, Z UP]: [" << X << "," << Y << "," << Z << "]");

    // ----> Transform the point from `map` frame to `left_camera_optical_frame`
    double camX, camY, camZ;
    try {
        // Save the transformation
        geometry_msgs::TransformStamped m2o = mTfBuffer->lookupTransform(mLeftCamOptFrameId, msg->header.frame_id, ros::Time(0), ros::Duration(0.1));

        NODELET_INFO("'%s' -> '%s': {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f,%.3f}", msg->header.frame_id.c_str(), mLeftCamOptFrameId.c_str(),
            m2o.transform.translation.x, m2o.transform.translation.y, m2o.transform.translation.z,
            m2o.transform.rotation.x, m2o.transform.rotation.y, m2o.transform.rotation.z, m2o.transform.rotation.w);

        // Get the TF2 transformation
        geometry_msgs::PointStamped ptCam;

        tf2::doTransform(*msg, ptCam, m2o);

        camX = ptCam.point.x;
        camY = ptCam.point.y;
        camZ = ptCam.point.z;

        NODELET_INFO("Point in camera coordinates [Z FW, X RG, Y DW]: {%.3f,%.3f,%.3f}", camX, camY, camZ);
    } catch (tf2::TransformException& ex) {
        NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
        NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.", msg->header.frame_id.c_str(), mLeftCamOptFrameId.c_str());

        return;
    }
    // <---- Transform the point from `map` frame to `left_camera_optical_frame`

    // ----> Project the point into 2D image coordinates
    sl::CalibrationParameters zedParam;
    zedParam = mZed.getCameraInformation(mMatResolVideo).calibration_parameters; // ok

    float f_x = zedParam.left_cam.fx;
    float f_y = zedParam.left_cam.fy;
    float c_x = zedParam.left_cam.cx;
    float c_y = zedParam.left_cam.cy;

    float u = ((camX / camZ) * f_x + c_x) / mCamImageResizeFactor;
    float v = ((camY / camZ) * f_y + c_y) / mCamImageResizeFactor;
    NODELET_INFO_STREAM("Clicked point image coordinates: [" << u << "," << v << "]");
    // <---- Project the point into 2D image coordinates

    // ----> Extract plane from clicked point
    sl::Plane plane;
    sl::ERROR_CODE err = mZed.findPlaneAtHit(sl::uint2(u, v), plane);
    if (err != sl::ERROR_CODE::SUCCESS) {
        NODELET_WARN("Error extracting plane at point [%.3f,%.3f,%.3f]: %s", X, Y, Z, sl::toString(err).c_str());
        return;
    }

    sl::float3 center = plane.getCenter();
    sl::float2 dims = plane.getExtents();

    if (dims[0] == 0 || dims[1] == 0) {
        NODELET_INFO("Plane not found at point [%.3f,%.3f,%.3f]", X, Y, Z);
        return;
    }

    NODELET_INFO("Found plane at point [%.3f,%.3f,%.3f] -> Center: [%.3f,%.3f,%.3f], Dims: %.3fx%.3f", X, Y, Z, center.x, center.y, center.z, dims[0], dims[1]);
    // <---- Extract plane from clicked point

    if (markerSubNumber > 0) {
        // ----> Publish a blue sphere in the clicked point
        visualization_msgs::MarkerPtr pt_marker = boost::make_shared<visualization_msgs::Marker>();
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        static int hit_pt_id = 0;
        pt_marker->header.stamp = ts;
        // Set the marker action.  Options are ADD and DELETE
        pt_marker->action = visualization_msgs::Marker::ADD;
        pt_marker->lifetime = ros::Duration();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        pt_marker->ns = "plane_hit_points";
        pt_marker->id = hit_pt_id++;
        pt_marker->header.frame_id = mMapFrameId;

        // Set the marker type.
        pt_marker->type = visualization_msgs::Marker::SPHERE;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        pt_marker->pose.position.x = X;
        pt_marker->pose.position.y = Y;
        pt_marker->pose.position.z = Z;
        pt_marker->pose.orientation.x = 0.0;
        pt_marker->pose.orientation.y = 0.0;
        pt_marker->pose.orientation.z = 0.0;
        pt_marker->pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        pt_marker->scale.x = 0.025;
        pt_marker->scale.y = 0.025;
        pt_marker->scale.z = 0.025;

        // Set the color -- be sure to set alpha to something non-zero!
        pt_marker->color.r = 0.2f;
        pt_marker->color.g = 0.1f;
        pt_marker->color.b = 0.75f;
        pt_marker->color.a = 0.8;

        // Publish the marker
        mPubMarker.publish(pt_marker);
        // ----> Publish a blue sphere in the clicked point

        // ----> Publish the plane as green mesh
        visualization_msgs::MarkerPtr plane_marker = boost::make_shared<visualization_msgs::Marker>();
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        static int plane_mesh_id = 0;
        plane_marker->header.stamp = ts;
        // Set the marker action.  Options are ADD and DELETE
        plane_marker->action = visualization_msgs::Marker::ADD;
        plane_marker->lifetime = ros::Duration();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        plane_marker->ns = "plane_meshes";
        plane_marker->id = plane_mesh_id++;
        plane_marker->header.frame_id = mLeftCamFrameId;

        // Set the marker type.
        plane_marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        plane_marker->pose.position.x = 0;
        plane_marker->pose.position.y = 0;
        plane_marker->pose.position.z = 0;
        plane_marker->pose.orientation.x = 0.0;
        plane_marker->pose.orientation.y = 0.0;
        plane_marker->pose.orientation.z = 0.0;
        plane_marker->pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        plane_marker->color.r = 0.10f;
        plane_marker->color.g = 0.75f;
        plane_marker->color.b = 0.20f;
        plane_marker->color.a = 0.75;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        plane_marker->scale.x = 1.0;
        plane_marker->scale.y = 1.0;
        plane_marker->scale.z = 1.0;

        sl::Mesh mesh = plane.extractMesh();
        size_t triangCount = mesh.getNumberOfTriangles();
        size_t ptCount = triangCount * 3;
        plane_marker->points.resize(ptCount);
        plane_marker->colors.resize(ptCount);

        size_t ptIdx = 0;
        for (size_t t = 0; t < triangCount; t++) {
            for (int p = 0; p < 3; p++) {
                uint vIdx = mesh.triangles[t][p];
                plane_marker->points[ptIdx].x = mesh.vertices[vIdx][0];
                plane_marker->points[ptIdx].y = mesh.vertices[vIdx][1];
                plane_marker->points[ptIdx].z = mesh.vertices[vIdx][2];

                // Set the color -- be sure to set alpha to something non-zero!
                plane_marker->colors[ptIdx].r = 0.10f;
                plane_marker->colors[ptIdx].g = 0.75f;
                plane_marker->colors[ptIdx].b = 0.20f;
                plane_marker->colors[ptIdx].a = 0.75;

                ptIdx++;
            }
        }

        // Publish the marker
        mPubMarker.publish(plane_marker);
        // <---- Publish the plane as green mesh
    }

    if (planeSubNumber > 0) {
        // ----> Publish the plane as custom message

        zed_interfaces::PlaneStampedPtr planeMsg = boost::make_shared<zed_interfaces::PlaneStamped>();
        planeMsg->header.stamp = ts;
        planeMsg->header.frame_id = mLeftCamFrameId;

        // Plane equation
        sl::float4 sl_coeff = plane.getPlaneEquation();
        planeMsg->coefficients.coef[0] = static_cast<double>(sl_coeff[0]);
        planeMsg->coefficients.coef[1] = static_cast<double>(sl_coeff[1]);
        planeMsg->coefficients.coef[2] = static_cast<double>(sl_coeff[2]);
        planeMsg->coefficients.coef[3] = static_cast<double>(sl_coeff[3]);

        // Plane Normal
        sl::float3 sl_normal = plane.getNormal();
        planeMsg->normal.x = sl_normal[0];
        planeMsg->normal.y = sl_normal[1];
        planeMsg->normal.z = sl_normal[2];

        // Plane Center
        sl::float3 sl_center = plane.getCenter();
        planeMsg->center.x = sl_center[0];
        planeMsg->center.y = sl_center[1];
        planeMsg->center.z = sl_center[2];

        // Plane extents
        sl::float3 sl_extents = plane.getExtents();
        planeMsg->extents[0] = sl_extents[0];
        planeMsg->extents[1] = sl_extents[1];

        // Plane pose
        sl::Pose sl_pose = plane.getPose();
        sl::Orientation sl_rot = sl_pose.getOrientation();
        sl::Translation sl_tr = sl_pose.getTranslation();

        planeMsg->pose.rotation.x = sl_rot.ox;
        planeMsg->pose.rotation.y = sl_rot.oy;
        planeMsg->pose.rotation.z = sl_rot.oz;
        planeMsg->pose.rotation.w = sl_rot.ow;

        planeMsg->pose.translation.x = sl_tr.x;
        planeMsg->pose.translation.y = sl_tr.y;
        planeMsg->pose.translation.z = sl_tr.z;

        // Plane Bounds
        std::vector<sl::float3> sl_bounds = plane.getBounds();
        planeMsg->bounds.points.resize(sl_bounds.size());
        memcpy(planeMsg->bounds.points.data(), sl_bounds.data(), 3 * sl_bounds.size() * sizeof(float));

        // Plane mesh
        sl::Mesh sl_mesh = plane.extractMesh();
        size_t triangCount = sl_mesh.triangles.size();
        size_t ptsCount = sl_mesh.vertices.size();
        planeMsg->mesh.triangles.resize(triangCount);
        planeMsg->mesh.vertices.resize(ptsCount);

        // memcpy not allowed because data types are different
        for (size_t i = 0; i < triangCount; i++) {
            planeMsg->mesh.triangles[i].vertex_indices[0] = sl_mesh.triangles[i][0];
            planeMsg->mesh.triangles[i].vertex_indices[1] = sl_mesh.triangles[i][1];
            planeMsg->mesh.triangles[i].vertex_indices[2] = sl_mesh.triangles[i][2];
        }

        // memcpy not allowed because data types are different
        for (size_t i = 0; i < ptsCount; i++) {
            planeMsg->mesh.vertices[i].x = sl_mesh.vertices[i][0];
            planeMsg->mesh.vertices[i].y = sl_mesh.vertices[i][1];
            planeMsg->mesh.vertices[i].z = sl_mesh.vertices[i][2];
        }

        mPubPlane.publish(planeMsg);
        // <---- Publish the plane as custom message
    }
}

} // namespace zed_nodelets
