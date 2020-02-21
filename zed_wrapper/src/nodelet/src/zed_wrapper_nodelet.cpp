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

#include "zed_wrapper_nodelet.hpp"

#include <csignal>
#include <chrono>

#ifndef NDEBUG
#include <ros/console.h>
#endif

#include "zed_wrapper/object_stamped.h"
#include "zed_wrapper/objects.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace zed_wrapper {

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

#define MAG_FREQ    50.
#define BARO_FREQ   25.

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

    if( mVerMajor < 3 )  {
        NODELET_ERROR( "This version of the ZED ROS Wrapper is designed to be used with ZED SDK v3.x");
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
    std::string raw_suffix = "_raw";
    string left_topic = leftTopicRoot + img_topic;
    string left_raw_topic = leftTopicRoot + raw_suffix + img_raw_topic;
    string right_topic = rightTopicRoot + img_topic;
    string right_raw_topic = rightTopicRoot + raw_suffix + img_raw_topic;
    string rgb_topic = rgbTopicRoot + img_topic;
    string rgb_raw_topic = rgbTopicRoot + raw_suffix + img_raw_topic;
    string stereo_topic = stereoTopicRoot + img_topic;
    string stereo_raw_topic = stereoTopicRoot + raw_suffix + img_raw_topic;

    // Set the disparity topic name
    std::string disparityTopic = "disparity/disparity_image";

    // Set the depth topic names
    string depth_topic_root = "depth";

    if (mOpenniDepthMode) {
        NODELET_INFO_STREAM("Openni depth mode activated");
        depth_topic_root += "/depth_raw_registered";
    } else {
        depth_topic_root += "/depth_registered";
    }


    string pointcloud_topic = "point_cloud/cloud_registered";

    string pointcloud_fused_topic = "mapping/fused_cloud";

    string object_det_topic_root = "obj_det";
    string object_det_topic = object_det_topic_root + "/objects";
    string object_det_rviz_topic = object_det_topic_root + "/object_markers";

    std::string confImgRoot = "confidence";
    string conf_map_topic_name = "confidence_map";
    string conf_map_topic = confImgRoot + "/" + conf_map_topic_name;

    // Set the positional tracking topic names
    std::string poseTopic = "pose";
    string pose_cov_topic;
    pose_cov_topic = poseTopic + "_with_covariance";

    std::string odometryTopic = "odom";
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
            mZedParams.input.setFromSVOFile(mSvoFilepath.c_str());
            mZedParams.svo_real_time_mode = false;
        } else  if (!mRemoteStreamAddr.empty()) {
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

                if (prop.id < -1 ||
                        prop.camera_state == sl::CAMERA_STATE::NOT_AVAILABLE) {
                    std::string msg = "ZED SN" + to_string(mZedSerialNumber) +
                            " not detected ! Please connect this ZED";
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
    NODELET_INFO_STREAM(" * Camera coordinate system\t-> COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD");

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

    mDiagUpdater.add("ZED Diagnostic", this, &ZEDWrapperNodelet::updateDiagnostic);
    mDiagUpdater.setHardwareID("ZED camera");

    mConnStatus = sl::ERROR_CODE::CAMERA_NOT_DETECTED;

    NODELET_INFO_STREAM(" *** Opening " << sl::toString( mZedUserCamModel) << "...");
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
    NODELET_INFO_STREAM(" ...  " << sl::toString( mZedRealCamModel) << " ready");

    CUdevice devid;
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

        sl::Transform cam_imu_tr = mZed.getCameraInformation().camera_imu_transform;

        printf( "Camera-IMU Transform: \n %s", cam_imu_tr.getInfos().c_str() );
    } else if (mZedRealCamModel == sl::MODEL::ZED2) {
        if (mZedUserCamModel != mZedRealCamModel) {
            NODELET_WARN("Camera model does not match user parameter. Please modify "
                         "the value of the parameter 'camera_model' to 'zed2'");
        }

        sl::Transform cam_imu_tr = mZed.getCameraInformation().camera_imu_transform;

        printf( "Camera-IMU Transform: \n %s", cam_imu_tr.getInfos().c_str() );
    }

    NODELET_INFO_STREAM(" * CAMERA MODEL\t -> " << sl::toString(mZedRealCamModel).c_str());
    mZedSerialNumber = mZed.getCameraInformation().serial_number;
    NODELET_INFO_STREAM(" * Serial Number -> " << mZedSerialNumber);

    if (!mSvoMode) {
        mCamFwVersion = mZed.getCameraInformation().camera_firmware_version;
        NODELET_INFO_STREAM(" * Camera FW Version -> " << mCamFwVersion);
        if(mZedRealCamModel!=sl::MODEL::ZED) {
            mSensFwVersion = mZed.getCameraInformation().sensors_firmware_version;
            NODELET_INFO_STREAM(" * Sensors FW Version -> " << mSensFwVersion);
        }
    } else {
        NODELET_INFO_STREAM(" * Input type\t -> " << sl::toString(mZed.getCameraInformation().input_type).c_str());
    }

    // Set the IMU topic names using real camera model
    string imu_topic;
    string imu_topic_raw;
    string imu_temp_topic;
    string imu_mag_topic;
    string imu_mag_topic_raw;
    string pressure_topic;
    string temp_topic_root = "temperature";
    string temp_topic_left = temp_topic_root + "/left";
    string temp_topic_right = temp_topic_root + "/right";

    if (mZedRealCamModel != sl::MODEL::ZED) {
        std::string imuTopicRoot = "imu";
        string imu_topic_name = "data";
        string imu_topic_raw_name = "data_raw";
        string imu_topic_mag_name = "mag";
        string imu_topic_mag_raw_name = "mag_raw";
        string pressure_topic_name = "atm_press";
        imu_topic = imuTopicRoot + "/" + imu_topic_name;
        imu_topic_raw = imuTopicRoot + "/" + imu_topic_raw_name;
        imu_temp_topic = temp_topic_root + "/" + imuTopicRoot;
        imu_mag_topic = imuTopicRoot + "/" + imu_topic_mag_name;
        imu_mag_topic_raw = imuTopicRoot + "/" + imu_topic_mag_raw_name;
        pressure_topic = /*imuTopicRoot + "/" +*/ pressure_topic_name;
    }

    mDiagUpdater.setHardwareIDf("%s - s/n: %d [GPU #%d]", sl::toString(mZedRealCamModel).c_str(), mZedSerialNumber, mGpuId);

    // ----> Dynamic Reconfigure parameters
    mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<zed_wrapper::ZedConfig>>(mDynServerMutex);
    dynamic_reconfigure::Server<zed_wrapper::ZedConfig>::CallbackType f;
    f = boost::bind(&ZEDWrapperNodelet::dynamicReconfCallback, this, _1, _2);
    mDynRecServer->setCallback(f);
    // Update parameters
    zed_wrapper::ZedConfig config;
    mDynRecServer->getConfigDefault(config);
    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();
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
    mPubDisparity = mNhNs.advertise<stereo_msgs::DisparityImage>(disparityTopic, 1);
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
        mPubObjDet = mNhNs.advertise<zed_wrapper::objects>(object_det_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubObjDet.getTopic());
        mPubObjDetViz = mNhNs.advertise<visualization_msgs::MarkerArray>(object_det_rviz_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubObjDetViz.getTopic());
    }

    // Odometry and Pose publisher
    mPubPose = mNhNs.advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubPose.getTopic());

    if (mPublishPoseCovariance) {
        mPubPoseCov = mNhNs.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_cov_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubPoseCov.getTopic());
    }

    mPubOdom = mNhNs.advertise<nav_msgs::Odometry>(odometryTopic, 1);
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

    // Sensor publishers
    /*if (!mSvoMode)*/ {
        if (mSensPubRate > 0 && mZedRealCamModel != sl::MODEL::ZED) {
            // IMU Publishers
            mPubImu = mNhNs.advertise<sensor_msgs::Imu>(imu_topic, static_cast<int>(mSensPubRate));
            NODELET_INFO_STREAM("Advertised on topic " << mPubImu.getTopic() << " @ "
                                << mSensPubRate << " Hz");
            mPubImuRaw = mNhNs.advertise<sensor_msgs::Imu>(imu_topic_raw, static_cast<int>(mSensPubRate));
            NODELET_INFO_STREAM("Advertised on topic " << mPubImuRaw.getTopic() << " @ "
                                << mSensPubRate << " Hz");
            mPubImuMag = mNhNs.advertise<sensor_msgs::MagneticField>(imu_mag_topic, MAG_FREQ);
            NODELET_INFO_STREAM("Advertised on topic " << mPubImuMag.getTopic() << " @ "
                                << std::min(MAG_FREQ,mSensPubRate) << " Hz");
            mPubImuMagRaw = mNhNs.advertise<sensor_msgs::MagneticField>(imu_mag_topic_raw, static_cast<int>(MAG_FREQ));
            NODELET_INFO_STREAM("Advertised on topic " << mPubImuMagRaw.getTopic() << " @ "
                                << std::min(MAG_FREQ,mSensPubRate) << " Hz");

            if( mZedRealCamModel == sl::MODEL::ZED2 ) {
                // IMU temperature sensor
                mPubImuTemp = mNhNs.advertise<sensor_msgs::Temperature>(imu_temp_topic, static_cast<int>(mSensPubRate));
                NODELET_INFO_STREAM("Advertised on topic " << mPubImuTemp.getTopic() << " @ " << mSensPubRate << " Hz");

                // Atmospheric pressure
                mPubPressure = mNhNs.advertise<sensor_msgs::FluidPressure>(pressure_topic, static_cast<int>(BARO_FREQ));
                NODELET_INFO_STREAM("Advertised on topic " << mPubPressure.getTopic() << " @ "
                                    << std::min(BARO_FREQ,mSensPubRate ) << " Hz");

                // CMOS sensor temperatures
                mPubTempL = mNhNs.advertise<sensor_msgs::Temperature>(temp_topic_left, static_cast<int>(BARO_FREQ));
                NODELET_INFO_STREAM("Advertised on topic " << mPubTempL.getTopic() << " @ "
                                    << std::min(BARO_FREQ,mSensPubRate ) << " Hz");
                mPubTempR = mNhNs.advertise<sensor_msgs::Temperature>(temp_topic_right, static_cast<int>(BARO_FREQ));
                NODELET_INFO_STREAM("Advertised on topic " << mPubTempR.getTopic() << " @ "
                                    << std::min(BARO_FREQ,mSensPubRate ) << " Hz");
            }

            mFrameTimestamp = ros::Time::now();
            mImuTimer = mNhNs.createTimer(ros::Duration(1.0 / mSensPubRate),
                                          &ZEDWrapperNodelet::sensPubCallback, this);
            mSensPeriodMean_usec.reset(new sl_tools::CSmartMean(mSensPubRate / 2));


        } else if (mSensPubRate > 0 && mZedRealCamModel == sl::MODEL::ZED) {
            NODELET_WARN_STREAM(
                        "'sens_pub_rate' set to "
                        << mSensPubRate << " Hz"
                        << " but ZED camera model does not support IMU data publishing.");
        }
    }

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

    mSrvStartObjDet = mNhNs.advertiseService("start_object_detection", &ZEDWrapperNodelet::on_start_object_detection, this);
    mSrvStopObjDet = mNhNs.advertiseService("stop_object_detection", &ZEDWrapperNodelet::on_stop_object_detection, this);

    // Start Pointcloud thread
    mPcThread = std::thread(&ZEDWrapperNodelet::pointcloud_thread_func, this);

    // Start pool thread
    mDevicePollThread = std::thread(&ZEDWrapperNodelet::device_poll_thread_func, this);
}

void ZEDWrapperNodelet::readParameters() {

    NODELET_INFO_STREAM("*** PARAMETERS ***");

    // ----> General
    // Get parameters from param files
    mNhNs.getParam("general/camera_name", mCameraName);
    NODELET_INFO_STREAM(" * Camera Name\t\t\t-> " << mCameraName.c_str());
    int resol;
    mNhNs.getParam("general/resolution", resol);
    mCamResol = static_cast<sl::RESOLUTION>(resol);
    NODELET_INFO_STREAM(" * Camera Resolution\t\t-> " << sl::toString(mCamResol).c_str());
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
    mNhNs.getParam("general/camera_model", camera_model);

    if (camera_model == "zed") {
        mZedUserCamModel = sl::MODEL::ZED;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " <<  camera_model);
    } else if (camera_model == "zedm") {
        mZedUserCamModel = sl::MODEL::ZED_M;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " <<  camera_model);
    } else if (camera_model == "zed2") {
        mZedUserCamModel = sl::MODEL::ZED2;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " <<  camera_model);
    } else {
        NODELET_ERROR_STREAM("Camera model not valid: " << camera_model);
    }
    // <---- General

    // ----> Video
    mNhNs.getParam("video/img_resample_factor", mCamImageResizeFactor);
    NODELET_INFO_STREAM(" * Image resample factor\t-> " << mCamImageResizeFactor);
    // <---- Video

    // -----> Depth
    int depth_mode;
    mNhNs.getParam("depth/quality", depth_mode);
    mDepthMode = static_cast<sl::DEPTH_MODE>(depth_mode);
    NODELET_INFO_STREAM(" * Depth quality\t\t-> " << sl::toString(mDepthMode).c_str());
    int sensing_mode;
    mNhNs.getParam("depth/sensing_mode", sensing_mode );
    mCamSensingMode = static_cast<sl::SENSING_MODE>(sensing_mode);
    NODELET_INFO_STREAM(" * Depth Sensing mode\t\t-> " << sl::toString(mCamSensingMode).c_str());
    mNhNs.getParam("depth/openni_depth_mode", mOpenniDepthMode);
    NODELET_INFO_STREAM(" * OpenNI mode\t\t\t-> " << (mOpenniDepthMode ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("depth/depth_stabilization", mDepthStabilization);
    NODELET_INFO_STREAM(" * Depth Stabilization\t\t-> " << (mDepthStabilization ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("depth/min_depth", mCamMinDepth);
    NODELET_INFO_STREAM(" * Minimum depth\t\t-> " <<  mCamMinDepth << " m");
    mNhNs.getParam("depth/max_depth", mCamMaxDepth);
    NODELET_INFO_STREAM(" * Maximum depth\t\t-> " << mCamMaxDepth << " m");
    mNhNs.getParam("depth/depth_resample_factor", mCamDepthResizeFactor);
    NODELET_INFO_STREAM(" * Depth resample factor\t-> " << mCamDepthResizeFactor);
    // <----- Depth

    // ----> Tracking
    mNhNs.getParam("pos_tracking/path_pub_rate", mPathPubRate);
    NODELET_INFO_STREAM(" * Path rate\t\t\t-> " <<  mPathPubRate << " Hz");
    mNhNs.getParam("pos_tracking/path_max_count", mPathMaxCount);
    NODELET_INFO_STREAM(" * Path history size\t\t-> " << (mPathMaxCount == -1) ? std::string("INFINITE") : std::to_string(
                                                                                     mPathMaxCount));

    if (mPathMaxCount < 2 && mPathMaxCount != -1) {
        mPathMaxCount = 2;
    }

    mNhNs.getParam("pos_tracking/initial_base_pose", mInitialBasePose);

    mNhNs.getParam("pos_tracking/odometry_DB", mOdometryDb);
    NODELET_INFO_STREAM(" * Odometry DB path\t\t-> " << mOdometryDb.c_str());
    mNhNs.param<bool>("pos_tracking/area_memory", mAreaMemory, false);
    NODELET_INFO_STREAM(" * Spatial Memory\t\t-> " << (mAreaMemory ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/imu_fusion", mImuFusion, true);
    NODELET_INFO_STREAM(" * IMU Fusion\t\t\t-> " << (mImuFusion ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/floor_alignment", mFloorAlignment, false);
    NODELET_INFO_STREAM(" * Floor alignment\t\t-> " << (mFloorAlignment ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/init_odom_with_first_valid_pose", mInitOdomWithPose, true);
    NODELET_INFO_STREAM(" * Init Odometry with first valid pose data -> " << (mInitOdomWithPose ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/two_d_mode", mTwoDMode, false);
    NODELET_INFO_STREAM(" * Two D mode\t\t\t-> " << (mTwoDMode ? "ENABLED" : "DISABLED"));
    mNhNs.param<double>("pos_tracking/fixed_z_value", mFixedZValue, 0.0);

    if (mTwoDMode) {
        NODELET_INFO_STREAM(" * Fixed Z value\t\t-> " << mFixedZValue);
    }

    mNhNs.getParam("pos_tracking/publish_pose_covariance", mPublishPoseCovariance);
    NODELET_INFO_STREAM(" * Publish Pose Covariance\t-> " << (mPublishPoseCovariance ? "ENABLED" : "DISABLED"));
    // <---- Tracking

    // ----> Mapping
    mNhNs.param<bool>("mapping/mapping_enabled", mMappingEnabled, false);

    if (mMappingEnabled) {
        NODELET_INFO_STREAM(" * Mapping\t\t\t-> ENABLED");

        mNhNs.getParam("mapping/resolution_m", mMappingRes);
        NODELET_INFO_STREAM(" * Mapping resolution\t\t-> " << mMappingRes << " m" );

        mNhNs.getParam("mapping/max_mapping_range_m", mMaxMappingRange);
        NODELET_INFO_STREAM(" * Mapping max range\t\t-> " << mMaxMappingRange << " m" << ((mMaxMappingRange < 0.0)?" [AUTO]":""));

        mNhNs.getParam("mapping/fused_pointcloud_freq", mFusedPcPubFreq);
        NODELET_INFO_STREAM(" * Fused point cloud freq:\t-> " << mFusedPcPubFreq << " Hz");
    } else {
        NODELET_INFO_STREAM(" * Mapping\t\t\t-> DISABLED");
    }
    // <---- Mapping

    // ----> Object Detection
    mNhNs.param<bool>("object_detection/od_enabled", mObjDetEnabled, false);

    if (mObjDetEnabled) {
        NODELET_INFO_STREAM(" * Object Detection\t\t-> ENABLED");

        mNhNs.getParam("object_detection/confidence_threshold", mObjDetConfidence);
        NODELET_INFO_STREAM(" * Object confidence\t\t-> " << mObjDetConfidence);
        mNhNs.getParam("mObjDetEnable/object_tracking_enabled", mObjDetTracking);
        NODELET_INFO_STREAM(" * Object tracking\t\t-> " << (mObjDetTracking?"ENABLED":"DISABLED"));
        mNhNs.getParam("mObjDetEnable/people_detection", mObjDetPeople);
        NODELET_INFO_STREAM(" * People detection\t\t-> " << (mObjDetPeople?"ENABLED":"DISABLED"));
        mNhNs.getParam("mObjDetEnable/vehicle_detection", mObjDetVehicles);
        NODELET_INFO_STREAM(" * Vehicles detection\t\t-> " << (mObjDetVehicles?"ENABLED":"DISABLED"));
    } else {
        NODELET_INFO_STREAM(" * Object Detection\t\t-> DISABLED");
    }
    // <---- Object Detection


    // ----> Sensors
    mNhNs.getParam("sensors/sensors_timestamp_sync", mSensTimestampSync);
    NODELET_INFO_STREAM(" * Sensors timestamp sync\t-> " << (mSensTimestampSync ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("sensors/sens_pub_rate", mSensPubRate);
    NODELET_INFO_STREAM(" * Sensors data freq\t\t-> " << mSensPubRate << " Hz");
    // <---- Sensors

    // ----> SVO
    mNhNs.param<std::string>("svo_file", mSvoFilepath, std::string());
    NODELET_INFO_STREAM(" * SVO input file: \t\t-> " << mSvoFilepath.c_str());

    int svo_compr = 0;
    mNhNs.getParam("general/svo_compression", svo_compr);

    if (svo_compr >= static_cast<int>(sl::SVO_COMPRESSION_MODE::LAST)) {
        NODELET_WARN_STREAM("The parameter `general/svo_compression` has an invalid value. Please check it in the configuration file `common.yaml`");

        svo_compr = 0;
    }

    mSvoComprMode = static_cast<sl::SVO_COMPRESSION_MODE>(svo_compr);

    NODELET_INFO_STREAM(" * SVO REC compression\t\t-> " << sl::toString(mSvoComprMode));
    // <---- SVO

    // Remote Stream
    mNhNs.param<std::string>("stream", mRemoteStreamAddr, std::string());

    // ----> Coordinate frames
    mNhNs.param<std::string>("pos_tracking/world_frame", mWorldFrameId, "map");
    mNhNs.param<std::string>("pos_tracking/map_frame", mMapFrameId, "map");
    mNhNs.param<std::string>("pos_tracking/odometry_frame", mOdometryFrameId, "odom");
    mNhNs.param<std::string>("general/base_frame", mBaseFrameId, "base_link");

    mCameraFrameId = mCameraName + "_camera_center";
    mImuFrameId = mCameraName + "_imu_link";
    mLeftCamFrameId = mCameraName + "_left_camera_frame";
    mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
    mRightCamFrameId = mCameraName + "_right_camera_frame";
    mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";

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
    mNhNs.param<bool>("pos_tracking/publish_tf", mPublishTf, true);
    NODELET_INFO_STREAM(" * Broadcast odometry TF\t-> " << (mPublishTf ? "ENABLED" : "DISABLED"));
    mNhNs.param<bool>("pos_tracking/publish_map_tf", mPublishMapTf, true);
    NODELET_INFO_STREAM(" * Broadcast map pose TF\t-> " << (mPublishTf ? (mPublishMapTf ? "ENABLED" : "DISABLED") :
                                                                         "DISABLED"));
    // <---- TF broadcasting

    // ----> Dynamic
    mNhNs.getParam("depth_confidence", mCamDepthConfidence);
    NODELET_INFO_STREAM(" * [DYN] Depth confidence\t-> " << mCamDepthConfidence);

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
#if (ZED_SDK_MAJOR_VERSION==3 && ZED_SDK_MINOR_VERSION>=1)
    mNhNs.getParam("gamma", mCamGamma);
    NODELET_INFO_STREAM(" * [DYN] gamma\t\t-> " << mCamGamma);
#endif
    mNhNs.getParam("auto_exposure_gain", mCamAutoExposure);
    NODELET_INFO_STREAM(" * [DYN] auto_exposure_gain\t-> " << (mCamAutoExposure ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("gain", mCamGain);
    mNhNs.getParam("exposure", mCamExposure);
    if(!mCamAutoExposure) {
        NODELET_INFO_STREAM("  * [DYN] gain\t\t-> " << mCamGain);
        NODELET_INFO_STREAM("  * [DYN] exposure\t\t-> " << mCamExposure);
    }
    mNhNs.getParam("auto_whitebalance", mCamAutoWB);
    NODELET_INFO_STREAM(" * [DYN] auto_whitebalance\t-> " << (mCamAutoWB ? "ENABLED" : "DISABLED"));
    mNhNs.getParam("whitebalance_temperature", mCamWB);
    if(!mCamAutoWB) {
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

void ZEDWrapperNodelet::checkResolFps() {
    switch (mCamResol) {
    case sl::RESOLUTION::HD2K:
        if (mCamFrameRate != 15) {
            NODELET_WARN_STREAM("Wrong FrameRate ("
                                << mCamFrameRate
                                << ") for the resolution HD2K. Set to 15 FPS.");
            mCamFrameRate = 15;
        }

        break;

    case sl::RESOLUTION::HD1080:
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

    case sl::RESOLUTION::HD720:
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

    case sl::RESOLUTION::VGA:
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
        mCamResol = sl::RESOLUTION::HD720;
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
    NODELET_DEBUG("Getting static TF from '%s' to '%s'", mCameraFrameId.c_str(), mBaseFrameId.c_str());

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

        NODELET_INFO("Static transform Camera Center to Base [%s -> %s]",
                     mCameraFrameId.c_str(), mBaseFrameId.c_str());
        NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                     mCamera2BaseTransf.getOrigin().x(), mCamera2BaseTransf.getOrigin().y(), mCamera2BaseTransf.getOrigin().z());
        NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                     roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

    } catch (tf2::TransformException& ex) {
        if (++errCount % 50 == 0) {
            NODELET_WARN("The tf from '%s' to '%s' is not yet available, "
                         "will assume it as identity!",
                         mCameraFrameId.c_str(), mBaseFrameId.c_str());
            NODELET_WARN("Transform error: %s", ex.what());
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
    NODELET_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mCameraFrameId.c_str());

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

        NODELET_INFO("Static transform Sensor to Camera Center [%s -> %s]",
                     mDepthFrameId.c_str(), mCameraFrameId.c_str());
        NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                     mSensor2CameraTransf.getOrigin().x(), mSensor2CameraTransf.getOrigin().y(), mSensor2CameraTransf.getOrigin().z());
        NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                     roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    } catch (tf2::TransformException& ex) {
        if (++errCount % 50 == 0) {
            NODELET_WARN("The tf from '%s' to '%s' is not yet available, "
                         "will assume it as identity!",
                         mDepthFrameId.c_str(), mCameraFrameId.c_str());
            NODELET_WARN("Transform error: %s", ex.what());
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
    NODELET_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mBaseFrameId.c_str());

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

        NODELET_INFO("Static transform Sensor to Base [%s -> %s]",
                     mDepthFrameId.c_str(), mBaseFrameId.c_str());
        NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                     mSensor2BaseTransf.getOrigin().x(), mSensor2BaseTransf.getOrigin().y(), mSensor2BaseTransf.getOrigin().z());
        NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                     roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

    } catch (tf2::TransformException& ex) {
        if (++errCount % 50 == 0) {
            NODELET_WARN("The tf from '%s' to '%s' is not yet available, "
                         "will assume it as identity!",
                         mDepthFrameId.c_str(), mBaseFrameId.c_str());
            NODELET_WARN("Transform error: %s", ex.what());
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
    mZed.disablePositionalTracking();

    // Restart tracking
    start_pos_tracking();

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
    mZed.disablePositionalTracking();

    // Restart tracking
    start_pos_tracking();

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

bool ZEDWrapperNodelet::start_3d_mapping() {
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

    if(mMappingRes < lRes) {
        NODELET_WARN_STREAM( "'mapping/resolution_m' value (" << mMappingRes << " m) is lower than the allowed resolution values. Fixed automatically" );
        mMappingRes = lRes;
    }
    if(mMappingRes > hRes) {
        NODELET_WARN_STREAM( "'mapping/resolution_m' value (" << mMappingRes << " m) is higher than the allowed resolution values. Fixed automatically" );
        mMappingRes = hRes;
    }

    params.resolution_meter = mMappingRes;

    float lRng = spMapPar.allowed_range.first;
    float hRng = spMapPar.allowed_range.second;

    if(mMaxMappingRange < 0) {
        mMaxMappingRange = sl::SpatialMappingParameters::getRecommendedRange( mMappingRes, mZed );
        NODELET_INFO_STREAM("Mapping: max range set to " << mMaxMappingRange << " m for a resolution of " << mMappingRes << " m"  );
    } else if(mMaxMappingRange < lRng) {
        NODELET_WARN_STREAM( "'mapping/max_mapping_range_m' value (" << mMaxMappingRange << " m) is lower than the allowed resolution values. Fixed automatically" );
        mMaxMappingRange = lRng;
    } else if(mMaxMappingRange > hRng) {
        NODELET_WARN_STREAM( "'mapping/max_mapping_range_m' value (" << mMaxMappingRange << " m) is higher than the allowed resolution values. Fixed automatically" );
        mMaxMappingRange = hRng;
    }

    params.range_meter = mMaxMappingRange;

    sl::ERROR_CODE err = mZed.enableSpatialMapping(params);

    if (err == sl::ERROR_CODE::SUCCESS) {
        if(mPubFusedCloud.getTopic().empty()) {
            string pointcloud_fused_topic = "mapping/fused_cloud";
            mPubFusedCloud = mNhNs.advertise<sensor_msgs::PointCloud2>(pointcloud_fused_topic, 1);
            NODELET_INFO_STREAM("Advertised on topic " << mPubFusedCloud.getTopic() << " @ " << mFusedPcPubFreq << " Hz");
        }

        mMappingRunning = true;

        mFusedPcTimer = mNhNs.createTimer(ros::Duration(1.0 / mFusedPcPubFreq), &ZEDWrapperNodelet::pubFusedPointCloudCallback,
                                          this);

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

void ZEDWrapperNodelet::stop_3d_mapping() {
    mFusedPcTimer.stop();
    mMappingRunning = false;
    mMappingEnabled = false;
    mZed.disableSpatialMapping();

    NODELET_INFO("*** Spatial Mapping stopped ***");
}

bool ZEDWrapperNodelet::start_obj_detect() {
    if(mZedRealCamModel!=sl::MODEL::ZED2) {
        NODELET_ERROR_STREAM( "Object detection not started. OD is available only using a ZED2 camera model");
        return false;
    }

    if(!mObjDetEnabled) {
        return false;
    }

    NODELET_INFO_STREAM("*** Starting Object Detection ***");

    sl::ObjectDetectionParameters od_p;
    od_p.enable_mask_output = false;
    od_p.enable_tracking = mObjDetTracking;
    od_p.image_sync = true;

    sl::ERROR_CODE objDetError = mZed.enableObjectDetection(od_p);

    if (objDetError != sl::ERROR_CODE::SUCCESS) {
        NODELET_ERROR_STREAM("Object detection error: " << sl::toString(objDetError));

        mObjDetRunning = false;
        return false;
    }

    if(mPubObjDet.getTopic().empty()) {
        string object_det_topic_root = "obj_det";
        string object_det_topic = object_det_topic_root + "/objects";
        string object_det_rviz_topic = object_det_topic_root + "/object_markers";

        mPubObjDet = mNhNs.advertise<zed_wrapper::objects>(object_det_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubObjDet.getTopic());
        mPubObjDetViz = mNhNs.advertise<visualization_msgs::MarkerArray>(object_det_rviz_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << mPubObjDetViz.getTopic());
    }

    mObjDetFilter.clear();
    if(mObjDetPeople) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
    }
    if(mObjDetVehicles) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
    }

    mObjDetRunning = true;
    return false;
}

void ZEDWrapperNodelet::stop_obj_detect() {
    if (mObjDetRunning) {
        NODELET_INFO_STREAM("*** Stopping Object Detection ***");
        mObjDetRunning = false;
        mObjDetEnabled = false;
        mZed.disableObjectDetection();
    }
}

void ZEDWrapperNodelet::start_pos_tracking() {
    NODELET_INFO_STREAM("*** Starting Positional Tracking ***");

    NODELET_INFO(" * Waiting for valid static transformations...");

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
            NODELET_WARN(" !!! Failed to get static transforms. Is the 'ROBOT STATE PUBLISHER' node correctly working? ");
            break;
        }

    } while (transformOk == false);

    if (transformOk) {
        NODELET_DEBUG("Time required to get valid static transforms: %g sec", elapsed / 1000.);
    }

    NODELET_INFO("Initial ZED left camera pose (ZED pos. tracking): ");
    NODELET_INFO(" * T: [%g,%g,%g]",
                 mInitialPoseSl.getTranslation().x, mInitialPoseSl.getTranslation().y, mInitialPoseSl.getTranslation().z);
    NODELET_INFO(" * Q: [%g,%g,%g,%g]",
                 mInitialPoseSl.getOrientation().ox, mInitialPoseSl.getOrientation().oy,
                 mInitialPoseSl.getOrientation().oz, mInitialPoseSl.getOrientation().ow);

    if (mOdometryDb != "" && !sl_tools::file_exist(mOdometryDb)) {
        mOdometryDb = "";
        NODELET_WARN("odometry_DB path doesn't exist or is unreachable.");
    }

    // Tracking parameters
    sl::PositionalTrackingParameters trackParams;

    trackParams.area_file_path = mOdometryDb.c_str();

    mPoseSmoothing = false; // Always false. Pose Smoothing is to be enabled only for VR/AR applications
    trackParams.enable_pose_smoothing = mPoseSmoothing;

    trackParams.enable_area_memory = mAreaMemory;
    trackParams.enable_imu_fusion = mImuFusion;
    trackParams.initial_world_transform = mInitialPoseSl;

    trackParams.set_floor_as_origin = mFloorAlignment;

    sl::ERROR_CODE err = mZed.enablePositionalTracking(trackParams);

    if (err == sl::ERROR_CODE::SUCCESS) {
        mTrackingActivated = true;
    } else {
        mTrackingActivated = false;

        NODELET_WARN("Tracking not activated: %s", sl::toString(err).c_str());
    }
}

void ZEDWrapperNodelet::publishOdom(tf2::Transform odom2baseTransf, sl::Pose& slPose, ros::Time t) {

    if(!mOdomMsg) {
        mOdomMsg = boost::make_shared<nav_msgs::Odometry>();
    }

    mOdomMsg->header.stamp = t;
    mOdomMsg->header.frame_id = mOdometryFrameId; // frame
    mOdomMsg->child_frame_id = mBaseFrameId;      // camera_frame
    // conversion from Tranform to message
    geometry_msgs::Transform base2odom = tf2::toMsg(odom2baseTransf);
    // Add all value in odometry message
    mOdomMsg->pose.pose.position.x = base2odom.translation.x;
    mOdomMsg->pose.pose.position.y = base2odom.translation.y;
    mOdomMsg->pose.pose.position.z = base2odom.translation.z;
    mOdomMsg->pose.pose.orientation.x = base2odom.rotation.x;
    mOdomMsg->pose.pose.orientation.y = base2odom.rotation.y;
    mOdomMsg->pose.pose.orientation.z = base2odom.rotation.z;
    mOdomMsg->pose.pose.orientation.w = base2odom.rotation.w;

    // Odometry pose covariance if available
    if (mPublishPoseCovariance) {
        for (size_t i = 0; i < mOdomMsg->pose.covariance.size(); i++) {
            mOdomMsg->pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]);

            if (mTwoDMode) {
                if (i == 14 || i == 21 || i == 28) {
                    mOdomMsg->pose.covariance[i] = 1e-9;    // Very low covariance if 2D mode
                } else if ((i >= 2 && i <= 4) ||
                           (i >= 8 && i <= 10) ||
                           (i >= 12 && i <= 13) ||
                           (i >= 15 && i <= 16) ||
                           (i >= 18 && i <= 20) ||
                           (i == 22) ||
                           (i >= 24 && i <= 27)) {
                    mOdomMsg->pose.covariance[i] = 0.0;
                }
            }
        }
    }

    // Publish odometry message
    mPubOdom.publish(mOdomMsg);
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

            if(!mPoseCovMsg) {
                mPoseCovMsg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
            }

            mPoseCovMsg->header = header;
            mPoseCovMsg->pose.pose = pose;

            // Odometry pose covariance if available
            for (size_t i = 0; i < mPoseCovMsg->pose.covariance.size(); i++) {
                mPoseCovMsg->pose.covariance[i] = static_cast<double>(mLastZedPose.pose_covariance[i]);

                if (mTwoDMode) {
                    if (i == 14 || i == 21 || i == 28) {
                        mPoseCovMsg->pose.covariance[i] = 1e-9;    // Very low covariance if 2D mode
                    } else if ((i >= 2 && i <= 4) ||
                               (i >= 8 && i <= 10) ||
                               (i >= 12 && i <= 13) ||
                               (i >= 15 && i <= 16) ||
                               (i >= 18 && i <= 20) ||
                               (i == 22) ||
                               (i >= 24 && i <= 27)) {
                        mPoseCovMsg->pose.covariance[i] = 0.0;
                    }
                }
            }

            // Publish pose with covariance stamped message
            mPubPoseCov.publish(mPoseCovMsg);
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

void ZEDWrapperNodelet::publishImage(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img,
                                     image_transport::CameraPublisher& pubImg, sensor_msgs::CameraInfoPtr camInfoMsg,
                                     string imgFrameId, ros::Time t) {
    camInfoMsg->header.stamp = t;
    sl_tools::imageToROSmsg( imgMsgPtr, img, imgFrameId, t);
    pubImg.publish(imgMsgPtr, camInfoMsg);
}

void ZEDWrapperNodelet::publishDepth(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat depth, ros::Time t) {

    mDepthCamInfoMsg->header.stamp = t;

    if (!mOpenniDepthMode) {
        sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
        mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
        return;
    }

    // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
    if(!imgMsgPtr) {
        imgMsgPtr = boost::make_shared<sensor_msgs::Image>();
    }

    imgMsgPtr->header.stamp = t;
    imgMsgPtr->header.frame_id = mDepthOptFrameId;
    imgMsgPtr->height = depth.getHeight();
    imgMsgPtr->width = depth.getWidth();

    int num = 1; // for endianness detection
    imgMsgPtr->is_bigendian = !(*(char*)&num == 1);

    imgMsgPtr->step = imgMsgPtr->width * sizeof(uint16_t);
    imgMsgPtr->encoding = sensor_msgs::image_encodings::MONO16;

    size_t size = imgMsgPtr->step * imgMsgPtr->height;
    imgMsgPtr->data.resize(size);

    uint16_t* data = (uint16_t*)(&imgMsgPtr->data[0]);

    int dataSize = imgMsgPtr->width * imgMsgPtr->height;
    sl::float1* depthDataPtr = depth.getPtr<sl::float1>();

    for (int i = 0; i < dataSize; i++) {
        *(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++) * 1000));    // in mm, rounded
    }

    mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
}

void ZEDWrapperNodelet::publishDisparity(sl::Mat disparity, ros::Time t) {

    sl::CameraInformation zedParam = mZed.getCameraInformation(mMatResolDepth);

    if(!mDisparityImgMsg) {
        mDisparityImgMsg = boost::make_shared<sensor_msgs::Image>();
    }

    sl_tools::imageToROSmsg(mDisparityImgMsg,disparity, mDisparityFrameId, t);

    if(!mDisparityMsg) {
        mDisparityMsg = boost::make_shared<stereo_msgs::DisparityImage>();
    }

    mDisparityMsg->image = *mDisparityImgMsg;
    mDisparityMsg->header = mDisparityMsg->image.header;
    mDisparityMsg->f = zedParam.calibration_parameters.left_cam.fx;
    mDisparityMsg->T = zedParam.calibration_parameters.T.x;

    if (mDisparityMsg->T > 0) {
        mDisparityMsg->T *= -1.0f;
    }

    mDisparityMsg->min_disparity = mDisparityMsg->f * mDisparityMsg->T / mZed.getInitParameters().depth_minimum_distance;
    mDisparityMsg->max_disparity = mDisparityMsg->f * mDisparityMsg->T / mZed.getInitParameters().depth_maximum_distance;
    mPubDisparity.publish(mDisparityMsg);
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
    if( !mPointcloudMsg ) {
        mPointcloudMsg = boost::make_shared<sensor_msgs::PointCloud2>();
    }

    // Publish freq calculation
    static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
    last_time = now;

    mPcPeriodMean_usec->addValue(elapsed_usec);

    // Initialize Point Cloud message
    // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

    int ptsCount = mMatResolDepth.width * mMatResolDepth.height;

    mPointcloudMsg->header.stamp = mPointCloudTime;

    if (mPointcloudMsg->width != mMatResolDepth.width || mPointcloudMsg->height != mMatResolDepth.height) {
        mPointcloudMsg->header.frame_id = mPointCloudFrameId; // Set the header values of the ROS message

        mPointcloudMsg->is_bigendian = false;
        mPointcloudMsg->is_dense = false;

        mPointcloudMsg->width = mMatResolDepth.width;
        mPointcloudMsg->height = mMatResolDepth.height;

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

    // We can do a direct memcpy since data organization is the same
    memcpy(ptCloudPtr, (float*)cpu_cloud, 4 * ptsCount * sizeof(float));

    // Pointcloud publishing
    mPubCloud.publish(mPointcloudMsg);
}

void ZEDWrapperNodelet::pubFusedPointCloudCallback(const ros::TimerEvent& e) {
    if( !mPointcloudFusedMsg ) {
        mPointcloudFusedMsg = boost::make_shared<sensor_msgs::PointCloud2>();
    }

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

    while (mZed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::FAILURE) {
        //Mesh is still generating
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sl::ERROR_CODE res = mZed.retrieveSpatialMapAsync(mFusedPC);

    if (res != sl::ERROR_CODE::SUCCESS) {
        NODELET_WARN_STREAM("Fused point cloud not extracted: " << sl::toString(res).c_str());
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

    //NODELET_INFO_STREAM("Chunks: " << mFusedPC.chunks.size());

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

    //NODELET_INFO_STREAM("Updated: " << updated);


    double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();


    //        NODELET_INFO_STREAM("Data copy: " << elapsed_usec << " usec [" << ptsCount << "] - " << (static_cast<double>
    //                        (ptsCount) / elapsed_usec) << " pts/usec");

    // Pointcloud publishing
    mPubFusedCloud.publish(mPointcloudFusedMsg);
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
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters_raw;
    } else {
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters;
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
    leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatResolVideo.width);
    leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatResolVideo.height);
    leftCamInfoMsg->header.frame_id = leftFrameId;
    rightCamInfoMsg->header.frame_id = rightFrameId;
}

void ZEDWrapperNodelet::fillCamDepthInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr depth_info_msg,
                                         string frame_id ) {
    sl::CalibrationParameters zedParam;

    zedParam = zed.getCameraInformation(mMatResolDepth).calibration_parameters;

    float baseline = zedParam.T[0];
    depth_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    depth_info_msg->D.resize(5);
    depth_info_msg->D[0] = zedParam.left_cam.disto[0];   // k1
    depth_info_msg->D[1] = zedParam.left_cam.disto[1];   // k2
    depth_info_msg->D[2] = zedParam.left_cam.disto[4];   // k3
    depth_info_msg->D[3] = zedParam.left_cam.disto[2];   // p1
    depth_info_msg->D[4] = zedParam.left_cam.disto[3];   // p2
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

void ZEDWrapperNodelet::updateDynamicReconfigure() {
    //NODELET_DEBUG_STREAM( "updateDynamicReconfigure MUTEX LOCK");
    mDynParMutex.lock();
    zed_wrapper::ZedConfig config;

    config.auto_exposure_gain = mCamAutoExposure;
    config.auto_whitebalance = mCamAutoWB;
    config.brightness = mCamBrightness;
    config.depth_confidence = mCamDepthConfidence;
    config.contrast = mCamContrast;
    config.exposure = mCamExposure;
    config.gain = mCamGain;
    config.hue = mCamHue;
    config.saturation = mCamSaturation;
    config.sharpness = mCamSharpness;
    config.gamma = mCamGamma;
    config.whitebalance_temperature = mCamWB/100;
    config.point_cloud_freq = mPointCloudFreq;
    mDynParMutex.unlock();

    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();

    //NODELET_DEBUG_STREAM( "updateDynamicReconfigure MUTEX UNLOCK");
}

void ZEDWrapperNodelet::dynamicReconfCallback(zed_wrapper::ZedConfig& config, uint32_t level) {
    //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX LOCK");
    mDynParMutex.lock();
    DynParams param = static_cast<DynParams>(level);

    switch (param) {
//    case MAT_RESIZE_FACTOR: {
//        mCamMatResizeFactor = config.mat_resize_factor;
//        NODELET_INFO("Reconfigure mat_resize_factor: %g", mCamMatResizeFactor);
//        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
//        mDynParMutex.unlock();

//        mCamDataMutex.lock();
//        size_t w = static_cast<size_t>(mCamWidth * mCamMatResizeFactor);
//        size_t h = static_cast<size_t>(mCamHeight * mCamMatResizeFactor);
//        mMatResol = sl::Resolution(w,h);
//        NODELET_DEBUG_STREAM("Data Mat size : " << mMatResol.width << "x" << mMatResol.height);

//        // Update Camera Info
//        fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId,
//                    mRightCamOptFrameId);
//        fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId,
//                    mRightCamOptFrameId, true);
//        mRgbCamInfoMsg = mDepthCamInfoMsg = mLeftCamInfoMsg; // the reference camera is
//        // the Left one (next to
//        // the ZED logo)
//        mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
//        mCamDataMutex.unlock();
//    }
//        break;

    case CONFIDENCE:
        mCamDepthConfidence = config.depth_confidence;
        NODELET_INFO("Reconfigure confidence threshold: %d", mCamDepthConfidence);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case POINTCLOUD_FREQ:
        mPointCloudFreq = config.point_cloud_freq;
        NODELET_INFO("Reconfigure point cloud frequency: %g", mPointCloudFreq);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case BRIGHTNESS:
        mCamBrightness = config.brightness;
        NODELET_INFO("Reconfigure image brightness: %d", mCamBrightness);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case CONTRAST:
        mCamContrast = config.contrast;
        NODELET_INFO("Reconfigure image contrast: %d", mCamContrast);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case HUE:
        mCamHue = config.hue;
        NODELET_INFO("Reconfigure image hue: %d", mCamHue);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case SATURATION:
        mCamSaturation = config.saturation;
        NODELET_INFO("Reconfigure image saturation: %d", mCamSaturation);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case SHARPNESS:
        mCamSharpness = config.sharpness;
        NODELET_INFO("Reconfigure image sharpness: %d", mCamSharpness);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case GAMMA:
#if (ZED_SDK_MAJOR_VERSION==3 && ZED_SDK_MINOR_VERSION>=1)
        mCamGamma = config.gamma;
        NODELET_INFO("Reconfigure image gamma: %d", mCamGamma);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
#else
        NODELET_DEBUG_STREAM( "Gamma Control is not available for SDK older that v3.1");
        mDynParMutex.unlock();
#endif
        break;

    case AUTO_EXP_GAIN:
        mCamAutoExposure = config.auto_exposure_gain;
        NODELET_INFO_STREAM("Reconfigure auto exposure/gain: " << mCamAutoExposure?"ENABLED":"DISABLED");
        if( !mCamAutoExposure ) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0 );
            mTriggerAutoExposure = false;
        } else {
            mTriggerAutoExposure = true;
        }
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case GAIN:
        mCamGain = config.gain;
        if(mCamAutoExposure) {
            NODELET_WARN("Reconfigure gain has no effect if 'auto_exposure_gain' is enabled");
        } else {
            NODELET_INFO("Reconfigure gain: %d", mCamGain);
        }
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case EXPOSURE:
        mCamExposure = config.exposure;
        if(mCamAutoExposure) {
            NODELET_WARN("Reconfigure exposure has no effect if 'auto_exposure_gain' is enabled");
        } else {
            NODELET_INFO("Reconfigure exposure: %d", mCamExposure);
        }
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case AUTO_WB:
        mCamAutoWB = config.auto_whitebalance;
        NODELET_INFO_STREAM("Reconfigure auto white balance: " << mCamAutoWB?"ENABLED":"DISABLED");
        if( !mCamAutoWB ) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0 );
            mTriggerAutoWB = false;
        } else {
            mTriggerAutoWB = true;
        }
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    case WB_TEMP:
        mCamWB = config.whitebalance_temperature*100;
        if(mCamAutoWB) {
            NODELET_WARN("Reconfigure white balance temperature has no effect if 'auto_whitebalance' is enabled");
        } else {
            NODELET_INFO("Reconfigure white balance temperature: %d", mCamWB);
        }
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
        break;

    default:
        NODELET_DEBUG_STREAM( "dynamicReconfCallback Unknown param: " << level);
        mDynParMutex.unlock();
        //NODELET_DEBUG_STREAM( "dynamicReconfCallback MUTEX UNLOCK");
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

void ZEDWrapperNodelet::sensPubCallback(const ros::TimerEvent& e) {

    if (mStreaming) {
        return;
    }

    std::lock_guard<std::mutex> lock(mCloseZedMutex);

    if (!mZed.isOpened()) {
        return;
    }

    uint32_t imu_SubNumber = mPubImu.getNumSubscribers();
    uint32_t imu_RawSubNumber = mPubImuRaw.getNumSubscribers();
    uint32_t imu_TempSubNumber = 0;
    uint32_t imu_MagSubNumber = 0;
    uint32_t imu_MagRawSubNumber = 0;
    uint32_t pressSubNumber = 0;
    uint32_t tempLeftSubNumber = 0;
    uint32_t tempRightSubNumber = 0;

    if( mZedRealCamModel == sl::MODEL::ZED2 ) {
        imu_TempSubNumber = mPubImuTemp.getNumSubscribers();
        imu_MagSubNumber = mPubImuMag.getNumSubscribers();
        imu_MagRawSubNumber = mPubImuMagRaw.getNumSubscribers();
        pressSubNumber = mPubPressure.getNumSubscribers();
        tempLeftSubNumber = mPubTempL.getNumSubscribers();
        tempRightSubNumber = mPubTempR.getNumSubscribers();
    }

    int totSub = imu_SubNumber + imu_RawSubNumber + imu_TempSubNumber + imu_MagSubNumber + imu_MagRawSubNumber +
            pressSubNumber + tempLeftSubNumber + tempRightSubNumber;

    ros::Time ts_imu;
    ros::Time ts_baro;
    ros::Time ts_mag;
    ros::Time ts_mag_raw;

    static ros::Time lastTs_baro = ros::Time();
    static ros::Time lastT_mag = ros::Time();
    static ros::Time lastT_mag_raw = ros::Time();

    sl::SensorsData sens_data;

    if(mSvoMode) {
        if( mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS )
            return;
    } else {
        if ( mSensTimestampSync && mGrabActive) {
            if( mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS )
                return;
        } else if ( !mSensTimestampSync ) {
            if( mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS )
                return;
        } else {
            return;
        }
    }

    if (mSvoMode) {
        ts_imu = ros::Time::now();
        ts_baro = ros::Time::now();
        ts_mag = ros::Time::now();
        ts_mag_raw = ros::Time::now();
    } else {
        if (mSensTimestampSync && mGrabActive) {
            ts_imu = mFrameTimestamp;
            ts_baro = mFrameTimestamp;
            ts_mag = mFrameTimestamp;
            ts_mag_raw = mFrameTimestamp;
        } else {
            ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
            ts_baro = sl_tools::slTime2Ros(sens_data.barometer.timestamp);
            ts_mag = sl_tools::slTime2Ros(sens_data.magnetometer.timestamp);
            ts_mag_raw = sl_tools::slTime2Ros(sens_data.magnetometer.timestamp);
        }
    }

    if( mZedRealCamModel == sl::MODEL::ZED2 ) {
        // Update temperatures for Diagnostic
        sens_data.temperature.get( sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT, mTempLeft);
        sens_data.temperature.get( sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT, mTempRight);
    }

    if (totSub<1) { // Nothing to publish
        return;
    }

    if( imu_SubNumber > 0 || imu_RawSubNumber > 0 ||
            imu_TempSubNumber > 0 || pressSubNumber > 0 ||
            imu_MagSubNumber > 0 || imu_MagRawSubNumber > 0 ) {
        // Publish freq calculation
        static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

        double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        last_time = now;

        mSensPeriodMean_usec->addValue(elapsed_usec);

        mSensPublishing = true;
    } else {
        mSensPublishing = false;
    }

    if (imu_TempSubNumber>0) {
        if(!mImuTempMsg) {
            mImuTempMsg = boost::make_shared<sensor_msgs::Temperature>();
        }

        mImuTempMsg->header.stamp = ts_imu;
        mImuTempMsg->header.frame_id = mImuFrameId;
        float imu_temp;
        sens_data.temperature.get( sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, imu_temp);
        mImuTempMsg->temperature = static_cast<double>(imu_temp);
        mImuTempMsg->variance = 0.0;

        mPubImuTemp.publish(mImuTempMsg);
    }


    if( sens_data.barometer.is_available && lastTs_baro != ts_baro ) {
        lastTs_baro = ts_baro;

        if( pressSubNumber>0 ) {
            if(!mPressMsg) {
                mPressMsg = boost::make_shared<sensor_msgs::FluidPressure>();
            }

            mPressMsg->header.stamp = ts_baro;
            mPressMsg->header.frame_id = mCameraFrameId;
            mPressMsg->fluid_pressure = sens_data.barometer.pressure * 1e-2; // Pascal
            mPressMsg->variance = 1.0585e-2;

            mPubPressure.publish(mPressMsg);
        }

        if( tempLeftSubNumber>0 ) {

            if(!mTempLeftMsg) {
                mTempLeftMsg = boost::make_shared<sensor_msgs::Temperature>();
            }

            mTempLeftMsg->header.stamp = ts_baro;
            mTempLeftMsg->header.frame_id = mLeftCamFrameId;
            mTempLeftMsg->temperature = static_cast<double>(mTempLeft);
            mTempLeftMsg->variance = 0.0;

            mPubTempL.publish(mTempLeftMsg);
        }

        if( tempRightSubNumber>0 ) {

            if(!mTempRightMsg) {
                mTempRightMsg = boost::make_shared<sensor_msgs::Temperature>();
            }

            mTempRightMsg->header.stamp = ts_baro;
            mTempRightMsg->header.frame_id = mRightCamFrameId;
            mTempRightMsg->temperature = static_cast<double>(mTempRight);
            mTempRightMsg->variance = 0.0;

            mPubTempR.publish(mTempRightMsg);
        }
    }

    if( imu_MagSubNumber>0 ) {
        if( sens_data.magnetometer.is_available && lastT_mag != ts_mag ) {
            lastT_mag = ts_mag;

            if(!mMagMsg) {
                mMagMsg = boost::make_shared<sensor_msgs::MagneticField>();
            }

            mMagMsg->header.stamp = ts_mag;
            mMagMsg->header.frame_id = mImuFrameId;
            mMagMsg->magnetic_field.x = sens_data.magnetometer.magnetic_field_calibrated.x*1e-6; // Tesla
            mMagMsg->magnetic_field.y = sens_data.magnetometer.magnetic_field_calibrated.y*1e-6; // Tesla
            mMagMsg->magnetic_field.z = sens_data.magnetometer.magnetic_field_calibrated.z*1e-6; // Tesla
            mMagMsg->magnetic_field_covariance[0] = 0.039e-6;
            mMagMsg->magnetic_field_covariance[1] = 0.0f;
            mMagMsg->magnetic_field_covariance[2] = 0.0f;
            mMagMsg->magnetic_field_covariance[3] = 0.0f;
            mMagMsg->magnetic_field_covariance[4] = 0.037e-6;
            mMagMsg->magnetic_field_covariance[5] = 0.0f;
            mMagMsg->magnetic_field_covariance[6] = 0.0f;
            mMagMsg->magnetic_field_covariance[7] = 0.0f;
            mMagMsg->magnetic_field_covariance[8] = 0.047e-6;

            mPubImuMag.publish(mMagMsg);
        }
    }

    if( imu_MagRawSubNumber>0 ) {
        if( sens_data.magnetometer.is_available && lastT_mag_raw != ts_mag_raw ) {
            lastT_mag_raw = ts_mag_raw;

            if(!mMagRawMsg) {
                mMagRawMsg = boost::make_shared<sensor_msgs::MagneticField>();
            }

            mMagRawMsg->header.stamp = ts_mag;
            mMagRawMsg->header.frame_id = mImuFrameId;
            mMagRawMsg->magnetic_field.x = sens_data.magnetometer.magnetic_field_uncalibrated.x*1e-6; // Tesla
            mMagRawMsg->magnetic_field.y = sens_data.magnetometer.magnetic_field_uncalibrated.y*1e-6; // Tesla
            mMagRawMsg->magnetic_field.z = sens_data.magnetometer.magnetic_field_uncalibrated.z*1e-6; // Tesla
            mMagRawMsg->magnetic_field_covariance[0] = 0.039e-6;
            mMagRawMsg->magnetic_field_covariance[1] = 0.0f;
            mMagRawMsg->magnetic_field_covariance[2] = 0.0f;
            mMagRawMsg->magnetic_field_covariance[3] = 0.0f;
            mMagRawMsg->magnetic_field_covariance[4] = 0.037e-6;
            mMagRawMsg->magnetic_field_covariance[5] = 0.0f;
            mMagRawMsg->magnetic_field_covariance[6] = 0.0f;
            mMagRawMsg->magnetic_field_covariance[7] = 0.0f;
            mMagRawMsg->magnetic_field_covariance[8] = 0.047e-6;

            mPubImuMagRaw.publish(mMagRawMsg);
        }
    }

    if (imu_SubNumber > 0) {

        if(!mImuMsg) {
            mImuMsg = boost::make_shared<sensor_msgs::Imu>();
        }

        mImuMsg->header.stamp = ts_imu;
        mImuMsg->header.frame_id = mImuFrameId;

        mImuMsg->orientation.x = sens_data.imu.pose.getOrientation()[0];
        mImuMsg->orientation.y = sens_data.imu.pose.getOrientation()[1];
        mImuMsg->orientation.z = sens_data.imu.pose.getOrientation()[2];
        mImuMsg->orientation.w = sens_data.imu.pose.getOrientation()[3];

        mImuMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
        mImuMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
        mImuMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

        mImuMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
        mImuMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
        mImuMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

        for (int i = 0; i < 3; ++i) {

            int r = 0;

            if (i == 0) {
                r = 0;
            } else if (i == 1) {
                r = 1;
            } else {
                r = 2;
            }

            mImuMsg->orientation_covariance[i * 3 + 0] =
                    sens_data.imu.pose_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
            mImuMsg->orientation_covariance[i * 3 + 1] =
                    sens_data.imu.pose_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
            mImuMsg->orientation_covariance[i * 3 + 2] =
                    sens_data.imu.pose_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;

            mImuMsg->linear_acceleration_covariance[i * 3 + 0] =
                    sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
            mImuMsg->linear_acceleration_covariance[i * 3 + 1] =
                    sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
            mImuMsg->linear_acceleration_covariance[i * 3 + 2] =
                    sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

            mImuMsg->angular_velocity_covariance[i * 3 + 0] =
                    sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
            mImuMsg->angular_velocity_covariance[i * 3 + 1] =
                    sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
            mImuMsg->angular_velocity_covariance[i * 3 + 2] =
                    sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
        }

        mPubImu.publish(mImuMsg);
    }

    if (imu_RawSubNumber > 0) {

        if(!mImuRawMsg) {
            mImuRawMsg = boost::make_shared<sensor_msgs::Imu>();
        }

        mImuRawMsg->header.stamp = mFrameTimestamp; // t;
        mImuRawMsg->header.frame_id = mImuFrameId;
        mImuRawMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
        mImuRawMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
        mImuRawMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;
        mImuRawMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
        mImuRawMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
        mImuRawMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

        for (int i = 0; i < 3; ++i) {

            int r = 0;

            if (i == 0) {
                r = 0;
            } else if (i == 1) {
                r = 1;
            } else {
                r = 2;
            }

            mImuRawMsg->linear_acceleration_covariance[i * 3 + 0] =
                    sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
            mImuRawMsg->linear_acceleration_covariance[i * 3 + 1] =
                    sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
            mImuRawMsg->linear_acceleration_covariance[i * 3 + 2] =
                    sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];
            mImuRawMsg->angular_velocity_covariance[i * 3 + 0] =
                    sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
            mImuRawMsg->angular_velocity_covariance[i * 3 + 1] =
                    sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
            mImuRawMsg->angular_velocity_covariance[i * 3 + 2] =
                    sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
        }

        mImuRawMsg->orientation_covariance[0] =
                -1; // Orientation data is not available in "data_raw" -> See ROS REP145
        // http://www.ros.org/reps/rep-0145.html#topics
        mPubImuRaw.publish(mImuRawMsg);
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
                        10.0, "The tf from '%s' to '%s' is not yet available. "
                              "IMU TF not published!",
                        mCameraFrameId.c_str(), mMapFrameId.c_str());
            NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
            return;
        }

        // IMU Quaternion in Map frame
        tf2::Quaternion imu_q;
        imu_q.setX(sens_data.imu.pose.getOrientation()[0]);
        imu_q.setY(sens_data.imu.pose.getOrientation()[1]);
        imu_q.setZ(sens_data.imu.pose.getOrientation()[2]);
        imu_q.setW(sens_data.imu.pose.getOrientation()[3]);
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
    mObjDetPeriodMean_msec.reset(new sl_tools::CSmartMean(mCamFrameRate));

    // Timestamp initialization
    if (mSvoMode) {
        mFrameTimestamp = ros::Time::now();
    } else {
        mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
    }

    mPrevFrameTimestamp = mFrameTimestamp;

    mTrackingActivated = false;
    mMappingRunning = false;
    mRecording = false;

    // Get the parameters of the ZED images
    mCamWidth = mZed.getCameraInformation().camera_resolution.width;
    mCamHeight = mZed.getCameraInformation().camera_resolution.height;
    NODELET_DEBUG_STREAM("Camera Frame size : " << mCamWidth << "x" << mCamHeight);
    int v_w = static_cast<int>(mCamWidth * mCamImageResizeFactor);
    int v_h = static_cast<int>(mCamHeight * mCamImageResizeFactor);
    mMatResolVideo = sl::Resolution(v_w,v_h);
    NODELET_DEBUG_STREAM("Image Mat size : " << mMatResolVideo.width << "x" << mMatResolVideo.height);
    int d_w = static_cast<int>(mCamWidth * mCamDepthResizeFactor);
    int d_h = static_cast<int>(mCamHeight * mCamDepthResizeFactor);
    mMatResolDepth = sl::Resolution(d_w,d_h);
    NODELET_DEBUG_STREAM("Depth Mat size : " << mMatResolDepth.width << "x" << mMatResolDepth.height);

    // Create and fill the camera information messages
    fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
    fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId,
                mRightCamOptFrameId, true);
    fillCamDepthInfo(mZed,mDepthCamInfoMsg,mLeftCamOptFrameId);

    // the reference camera is the Left one (next to the ZED logo)

    mRgbCamInfoMsg = mLeftCamInfoMsg;
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
        uint32_t fusedCloudSubnumber = mPubFusedCloud.getNumSubscribers();
        uint32_t poseSubnumber = mPubPose.getNumSubscribers();
        uint32_t poseCovSubnumber = mPubPoseCov.getNumSubscribers();
        uint32_t odomSubnumber = mPubOdom.getNumSubscribers();
        uint32_t confMapSubnumber = mPubConfMap.getNumSubscribers();
        uint32_t pathSubNumber = mPubMapPath.getNumSubscribers() + mPubOdomPath.getNumSubscribers();
        uint32_t stereoSubNumber = mPubStereo.getNumSubscribers();
        uint32_t stereoRawSubNumber = mPubRawStereo.getNumSubscribers();

        uint32_t objDetSubnumber = 0;
        uint32_t objDetVizSubnumber = 0;
        bool objDetActive = false;
        if (mObjDetEnabled) {
            objDetSubnumber = mPubObjDet.getNumSubscribers();
            objDetVizSubnumber = mPubObjDetViz.getNumSubscribers();
            if (objDetSubnumber > 0 || objDetVizSubnumber > 0) {
                objDetActive = true;
            }
        }

        mGrabActive =  mRecording || mStreaming || mMappingEnabled || mObjDetEnabled || mTrackingActivated ||
                ((rgbSubnumber + rgbRawSubnumber + leftSubnumber +
                  leftRawSubnumber + rightSubnumber + rightRawSubnumber +
                  depthSubnumber + disparitySubnumber + cloudSubnumber +
                  poseSubnumber + poseCovSubnumber + odomSubnumber +
                  confMapSubnumber /*+ imuSubnumber + imuRawsubnumber*/ + pathSubNumber +
                  stereoSubNumber + stereoRawSubNumber) > 0);

        // Run the loop only if there is some subscribers or SVO is active
        if (mGrabActive) {
            std::lock_guard<std::mutex> lock(mPosTrkMutex);

            // Note: ones tracking is started it is never stopped anymore to not lose tracking information
            bool computeTracking = (mMappingEnabled || mObjDetEnabled || (mComputeDepth & mDepthStabilization) || poseSubnumber > 0 ||
                                    poseCovSubnumber > 0 || odomSubnumber > 0 || pathSubNumber > 0);

            // Start the tracking?
            if ((computeTracking) && !mTrackingActivated && (mDepthMode != sl::DEPTH_MODE::NONE)) {
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
            mComputeDepth = mDepthMode != sl::DEPTH_MODE::NONE &&
                    ((depthSubnumber + disparitySubnumber + cloudSubnumber + fusedCloudSubnumber +
                      poseSubnumber + poseCovSubnumber + odomSubnumber + confMapSubnumber) > 0);

            if (mComputeDepth) {
                runParams.confidence_threshold = mCamDepthConfidence;
                runParams.enable_depth = true; // Ask to compute the depth
            } else {
                runParams.enable_depth = false; // Ask to not compute the depth
            }

            mGrabStatus = mZed.grab(runParams);

            std::chrono::steady_clock::time_point start_elab = std::chrono::steady_clock::now();

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
                            if(mRecording) {
                                mRecording=false;
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

                    mTrackingActivated = false;

                    computeTracking = mDepthStabilization || poseSubnumber > 0 || poseCovSubnumber > 0 ||
                            odomSubnumber > 0;

                    if (computeTracking) {  // Start the tracking
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

            // ----> Camera Settings
            if( !mSvoMode && mFrameCount%5 == 0 ) {
                //NODELET_DEBUG_STREAM( "[" << mFrameCount << "] device_poll_thread_func MUTEX LOCK");
                mDynParMutex.lock();
                bool update_dyn_params = false;

                int brightness = mZed.getCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS);
                if( brightness != mCamBrightness ) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, mCamBrightness);
                    NODELET_DEBUG_STREAM( "mCamBrightness changed: " << mCamBrightness << " <- " << brightness);
                    update_dyn_params = true;
                }

                int contrast = mZed.getCameraSettings(sl::VIDEO_SETTINGS::CONTRAST);
                if( contrast != mCamContrast ) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, mCamContrast);
                    NODELET_DEBUG_STREAM( "mCamContrast changed: " << mCamContrast << " <- " << contrast);
                    update_dyn_params = true;
                }

                int hue = mZed.getCameraSettings(sl::VIDEO_SETTINGS::HUE);
                if( hue != mCamHue ) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, mCamHue);
                    NODELET_DEBUG_STREAM( "mCamHue changed: " << mCamHue << " <- " << hue);
                    update_dyn_params = true;
                }

                int saturation = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SATURATION);
                if( saturation != mCamSaturation ) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, mCamSaturation);
                    NODELET_DEBUG_STREAM( "mCamSaturation changed: " << mCamSaturation << " <- " << saturation);
                    update_dyn_params = true;
                }

                int sharpness = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS);
                if( sharpness != mCamSharpness ) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, mCamSharpness);
                    NODELET_DEBUG_STREAM( "mCamSharpness changed: " << mCamSharpness << " <- " << sharpness);
                    update_dyn_params = true;
                }

#if (ZED_SDK_MAJOR_VERSION==3 && ZED_SDK_MINOR_VERSION>=1)
                int gamma = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAMMA);
                if( gamma != mCamGamma ) {
                    mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, mCamGamma);
                    NODELET_DEBUG_STREAM( "mCamGamma changed: " << mCamGamma << " <- " << gamma);
                    update_dyn_params = true;
                }
#endif

                if (mCamAutoExposure) {
                    if( mTriggerAutoExposure ) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 1);
                        mTriggerAutoExposure = false;
                    }
                } else {
                    int exposure = mZed.getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE);
                    if (exposure != mCamExposure) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, mCamExposure);
                        NODELET_DEBUG_STREAM( "mCamExposure changed: " << mCamExposure << " <- " << exposure);
                        update_dyn_params = true;
                    }

                    int gain = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAIN);
                    if (gain != mCamGain) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, mCamGain);
                        NODELET_DEBUG_STREAM( "mCamGain changed: " << mCamGain << " <- " << gain);
                        update_dyn_params = true;
                    }
                }

                if (mCamAutoWB ) {
                    if(mTriggerAutoWB) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);
                        mTriggerAutoWB = false;
                    }
                } else {
                    int wb = mZed.getCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE);
                    if (wb != mCamWB) {
                        mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, mCamWB);
                        NODELET_DEBUG_STREAM( "mCamWB changed: " << mCamWB << " <- " << wb);
                        update_dyn_params = true;
                    }
                }
                mDynParMutex.unlock();
                //NODELET_DEBUG_STREAM( "device_poll_thread_func MUTEX UNLOCK");

                if(update_dyn_params) {
                    NODELET_DEBUG( "Update Dynamic Parameters");
                    updateDynamicReconfigure();
                }
            }
            // <---- Camera Settings

            mCamDataMutex.lock();

            // Publish the left == rgb image if someone has subscribed to
            if (leftSubnumber > 0 || rgbSubnumber > 0) {

                // Retrieve RGBA Left image
                mZed.retrieveImage(leftZEDMat, sl::VIEW::LEFT, sl::MEM::CPU, mMatResolVideo);

                if (leftSubnumber > 0) {
                    if(!mLeftImgMsg ) {
                        mLeftImgMsg = boost::make_shared<sensor_msgs::Image>();
                    }
                    if(!mLeftCamInfoMsg) {
                        mLeftCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                    }
                    publishImage(mLeftImgMsg, leftZEDMat, mPubLeft, mLeftCamInfoMsg, mLeftCamOptFrameId, mFrameTimestamp);
                }

                if (rgbSubnumber > 0) {
                    if(!mRgbImgMsg ) {
                        mRgbImgMsg = boost::make_shared<sensor_msgs::Image>();
                    }
                    if(!mRgbCamInfoMsg) {
                        mRgbCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                    }
                    publishImage(mRgbImgMsg, leftZEDMat, mPubRgb, mRgbCamInfoMsg, mDepthOptFrameId, mFrameTimestamp); // rgb is the left image
                }
            }

            // Publish the left_raw == rgb_raw image if someone has subscribed to
            if (leftRawSubnumber > 0 || rgbRawSubnumber > 0) {

                // Retrieve RGBA Left image
                mZed.retrieveImage(leftZEDMat, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);

                if (leftRawSubnumber > 0) {
                    if(!mRawLeftImgMsg ) {
                        mRawLeftImgMsg = boost::make_shared<sensor_msgs::Image>();
                    }
                    if(!mLeftCamInfoRawMsg) {
                        mLeftCamInfoRawMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                    }
                    publishImage(mRawLeftImgMsg, leftZEDMat, mPubRawLeft, mLeftCamInfoRawMsg, mLeftCamOptFrameId, mFrameTimestamp);
                }

                if (rgbRawSubnumber > 0) {
                    if(!mRawRgbImgMsg ) {
                        mRawRgbImgMsg = boost::make_shared<sensor_msgs::Image>();
                    }
                    if(!mRgbCamInfoRawMsg) {
                        mRgbCamInfoRawMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                    }
                    publishImage(mRawRgbImgMsg, leftZEDMat, mPubRawRgb, mRgbCamInfoRawMsg, mDepthOptFrameId, mFrameTimestamp);
                }
            }

            // Publish the right image if someone has subscribed to
            if (rightSubnumber > 0) {

                // Retrieve RGBA Right image
                mZed.retrieveImage(rightZEDMat, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResolVideo);
                if(!mRightImgMsg ) {
                    mRightImgMsg = boost::make_shared<sensor_msgs::Image>();
                }
                if(!mRightCamInfoMsg) {
                    mRightCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                }
                publishImage(mRightImgMsg, rightZEDMat, mPubRight, mRightCamInfoMsg, mRightCamOptFrameId, mFrameTimestamp);
            }

            // Publish the right image if someone has subscribed to
            if (rightRawSubnumber > 0) {

                // Retrieve RGBA Right image
                mZed.retrieveImage(rightZEDMat, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
                if(!mRawRightImgMsg ) {
                    mRawRightImgMsg = boost::make_shared<sensor_msgs::Image>();
                }
                if(!mRightCamInfoRawMsg) {
                    mRightCamInfoRawMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                }
                publishImage(mRawRightImgMsg, rightZEDMat, mPubRawRight, mRightCamInfoRawMsg, mRightCamOptFrameId, mFrameTimestamp);
            }

            // Stereo couple side-by-side
            if (stereoSubNumber > 0) {

                // Retrieve RGBA Right image
                mZed.retrieveImage(rightZEDMat, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResolVideo);
                mZed.retrieveImage(leftZEDMat, sl::VIEW::LEFT, sl::MEM::CPU, mMatResolVideo);
                if(!mStereoImgMsg ) {
                    mStereoImgMsg = boost::make_shared<sensor_msgs::Image>();
                }
                sl_tools::imagesToROSmsg(mStereoImgMsg, leftZEDMat, rightZEDMat, mCameraFrameId, mFrameTimestamp);
                mPubStereo.publish(mStereoImgMsg);
            }

            // Stereo RAW couple side-by-side
            if (stereoRawSubNumber > 0) {

                // Retrieve RGBA Right image
                mZed.retrieveImage(rightZEDMat, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
                mZed.retrieveImage(leftZEDMat, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
                if(!mRawStereoImgMsg ) {
                    mRawStereoImgMsg = boost::make_shared<sensor_msgs::Image>();
                }
                sl_tools::imagesToROSmsg(mRawStereoImgMsg, leftZEDMat, rightZEDMat, mCameraFrameId, mFrameTimestamp);
                mPubRawStereo.publish(mRawStereoImgMsg);
            }

            // Publish the depth image if someone has subscribed to
            if (depthSubnumber > 0 || disparitySubnumber > 0) {

                mZed.retrieveMeasure(depthZEDMat, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResolDepth);
                if(!mDepthImgMsg ) {
                    mDepthImgMsg = boost::make_shared<sensor_msgs::Image>();
                }
                publishDepth(mDepthImgMsg, depthZEDMat, mFrameTimestamp); // in meters
            }

            // Publish the disparity image if someone has subscribed to
            if (disparitySubnumber > 0) {

                mZed.retrieveMeasure(disparityZEDMat, sl::MEASURE::DISPARITY, sl::MEM::CPU, mMatResolDepth);
                publishDisparity(disparityZEDMat, mFrameTimestamp);
            }

            // Publish the confidence map if someone has subscribed to
            if (confMapSubnumber > 0) {

                mZed.retrieveMeasure(confMapZEDMat, sl::MEASURE::CONFIDENCE, sl::MEM::CPU, mMatResolDepth);
                sl_tools::imageToROSmsg(mConfMapMsg,confMapZEDMat, mConfidenceOptFrameId, mFrameTimestamp);
                if(!mConfMapMsg ) {
                    mConfMapMsg = boost::make_shared<sensor_msgs::Image>();
                }
                mPubConfMap.publish(mConfMapMsg);
            }

            // Publish the point cloud if someone has subscribed to
            if (cloudSubnumber > 0) {

                // Run the point cloud conversion asynchronously to avoid slowing down
                // all the program
                // Retrieve raw pointCloud data if latest Pointcloud is ready
                std::unique_lock<std::mutex> lock(mPcMutex, std::defer_lock);

                if (lock.try_lock()) {
                    mZed.retrieveMeasure(mCloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU, mMatResolDepth);

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

            mObjDetMutex.lock();
            if (mObjDetRunning) {
                std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                detectObjects(objDetSubnumber > 0, objDetVizSubnumber > 0);
                std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

                double elapsed_msec = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
                mObjDetPeriodMean_msec->addValue(elapsed_msec);
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
                    mTrackingStatus = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME::CAMERA);

                    sl::Translation translation = deltaOdom.getTranslation();
                    sl::Orientation quat = deltaOdom.getOrientation();

#if 0
                    NODELET_DEBUG("delta ODOM [%s] - %.2f,%.2f,%.2f %.2f,%.2f,%.2f,%.2f",
                                  sl::toString(mTrackingStatus).c_str(),
                                  translation(0), translation(1), translation(2),
                                  quat(0), quat(1), quat(2), quat(3));

                    NODELET_DEBUG_STREAM("ODOM -> Tracking Status: " << sl::toString(mTrackingStatus));
#endif

                    if (mTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK || mTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING ||
                            mTrackingStatus == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW) {
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
                static sl::POSITIONAL_TRACKING_STATE oldStatus;
                mTrackingStatus = mZed.getPosition(mLastZedPose, sl::REFERENCE_FRAME::WORLD);

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

                if (mTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK ||
                        mTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING /*|| status == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW*/) {
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

                    NODELET_DEBUG("*** Base POSE [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                  mMapFrameId.c_str(), mBaseFrameId.c_str(),
                                  mMap2BaseTransf.getOrigin().x(), mMap2BaseTransf.getOrigin().y(), mMap2BaseTransf.getOrigin().z(),
                                  roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

                    bool initOdom = false;

                    if (!(mFloorAlignment)) {
                        initOdom = mInitOdomWithPose;
                    } else {
                        initOdom = (mTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK) & mInitOdomWithPose;
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
                NODELET_DEBUG("The tf from '%s' to '%s' is not yet available, "
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
                    t = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
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

    if(mRecording) {
        mRecording=false;
        mZed.disableRecording();
    }
    mZed.close();

    NODELET_DEBUG("ZED pool thread finished");
}

void ZEDWrapperNodelet::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) {

    if (mConnStatus != sl::ERROR_CODE::SUCCESS) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, sl::toString(mConnStatus).c_str());
        return;
    }

    if (mGrabActive) {
        if (mGrabStatus == sl::ERROR_CODE::SUCCESS /*|| mGrabStatus == sl::ERROR_CODE::NOT_A_NEW_FRAME*/) {

            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera grabbing");

            double freq = 1000000. / mGrabPeriodMean_usec->getMean();
            double freq_perc = 100.*freq / mCamFrameRate;
            stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

            stat.addf("Processing Time", "Mean time: %.3f sec (Max. %.3f sec)", mElabPeriodMean_sec->getMean(), 1. / mCamFrameRate);

            if( mSvoMode ) {
                int frame = mZed.getSVOPosition();
                int totFrames = mZed.getSVONumberOfFrames();
                double svo_perc = 100.*(static_cast<double>(frame)/totFrames);

                stat.addf("Playing SVO", "Frame: %d/%d (%.1f%%)", frame,totFrames, svo_perc);
            }

            if (mComputeDepth) {
                stat.add("Depth status", "ACTIVE");

                if (mPcPublishing) {
                    double freq = 1000000. / mPcPeriodMean_usec->getMean();
                    double freq_perc = 100.*freq / mPointCloudFreq;
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

                if (mObjDetRunning) {
                    stat.addf("Object data processing", "%.3f sec", mObjDetPeriodMean_msec->getMean() / 1000.);
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
        double freq_perc = 100.*freq / mSensPubRate;
        stat.addf("IMU", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
    } else {
        stat.add("IMU", "Topics not subscribed");
    }

    if( mSensPubRate > 0 && mZedRealCamModel == sl::MODEL::ZED2 ) {
        stat.addf("Left CMOS Temp.", "%.1f °C", mTempLeft);
        stat.addf("Right CMOS Temp.", "%.1f °C", mTempRight);

        if( mTempLeft>70.f || mTempRight>70.f ) {
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
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                             "Error adding frames to SVO file while recording. Check free disk space");
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

    sl::RecordingParameters recParams;
    recParams.compression_mode = mSvoComprMode;
    recParams.video_filename = req.svo_filename.c_str();
    err = mZed.enableRecording(recParams);

    if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION) {
        recParams.compression_mode = mSvoComprMode == sl::SVO_COMPRESSION_MODE::H265 ? sl::SVO_COMPRESSION_MODE::H264 :
                                                                                       sl::SVO_COMPRESSION_MODE::H265;

        NODELET_WARN_STREAM("The chosen " << sl::toString(mSvoComprMode).c_str() << "mode is not available. Trying " <<
                            sl::toString(recParams.compression_mode).c_str());

        err = mZed.enableRecording(recParams);

        if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION) {
            NODELET_WARN_STREAM(sl::toString(recParams.compression_mode).c_str() << "not available. Trying " << sl::toString(
                                    sl::SVO_COMPRESSION_MODE::H264).c_str());
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

    NODELET_INFO_STREAM("SVO recording STARTED: " << req.svo_filename << " (" << sl::toString(mSvoComprMode).c_str() << ")");

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

    NODELET_INFO_STREAM("SVO recording STOPPED");

    return true;
}

bool ZEDWrapperNodelet::on_start_remote_stream(zed_wrapper::start_remote_stream::Request& req,
                                               zed_wrapper::start_remote_stream::Response& res) {
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

bool ZEDWrapperNodelet::on_stop_remote_stream(zed_wrapper::stop_remote_stream::Request& req,
                                              zed_wrapper::stop_remote_stream::Response& res) {
    if (mStreaming) {
        mZed.disableStreaming();
    }

    mStreaming = false;
    NODELET_INFO_STREAM("SVO remote streaming STOPPED");

    res.done = true;
    return true;
}

bool ZEDWrapperNodelet::on_set_led_status(zed_wrapper::set_led_status::Request& req,
                                          zed_wrapper::set_led_status::Response& res) {
    if (mCamFwVersion < 1523) {
        NODELET_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
        return false;
    }

    mZed.setCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS, req.led_enabled ? 1 : 0);

    return true;
}

bool ZEDWrapperNodelet::on_toggle_led(zed_wrapper::toggle_led::Request& req,
                                      zed_wrapper::toggle_led::Response& res) {
    if (mCamFwVersion < 1523) {
        NODELET_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
        return false;
    }

    int status = mZed.getCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS);
    int new_status = status == 0 ? 1 : 0;
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS, new_status);

    return (new_status == 1);
}


bool ZEDWrapperNodelet::on_start_3d_mapping(zed_wrapper::start_3d_mapping::Request& req,
                                            zed_wrapper::start_3d_mapping::Response& res) {
    if( mMappingEnabled && mMappingRunning) {
        NODELET_WARN_STREAM("Spatial mapping was just running");

        res.done = false;
        return res.done;
    }

    mMappingRunning = false;

    mMappingRes = req.resolution;
    mMaxMappingRange = req.max_mapping_range;
    mFusedPcPubFreq = req.fused_pointcloud_freq;

    NODELET_DEBUG_STREAM(" * Received mapping resolution\t\t-> " << mMappingRes << " m");



    NODELET_DEBUG_STREAM(" * Received mapping max range\t-> " << mMaxMappingRange << " m" );

    NODELET_DEBUG_STREAM(" * Received fused point cloud freq:\t-> " << mFusedPcPubFreq << " Hz");

    mMappingEnabled = true;
    res.done = true;

    return res.done;
}

bool ZEDWrapperNodelet::on_stop_3d_mapping(zed_wrapper::stop_3d_mapping::Request& req,
                                           zed_wrapper::stop_3d_mapping::Response& res) {

    if( mMappingEnabled ) {
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

bool ZEDWrapperNodelet::on_start_object_detection(zed_wrapper::start_object_detection::Request& req,
                                                  zed_wrapper::start_object_detection::Response& res) {
    if(mZedRealCamModel!=sl::MODEL::ZED2) {
        mObjDetEnabled = false;
        mObjDetRunning = false;

        NODELET_ERROR_STREAM( "Object detection not started. OD is available only using a ZED2 camera model");
        return false;
    }

    if( mObjDetEnabled && mObjDetRunning) {
        NODELET_WARN_STREAM("Object Detection was just running");

        res.done = false;
        return res.done;
    }

    mObjDetRunning = false;

    mObjDetConfidence = req.confidence;
    mObjDetTracking = req.tracking;
    mObjDetPeople = req.people;
    mObjDetVehicles = req.vehicles;

    NODELET_DEBUG_STREAM(" * Object min. confidence\t-> " << mObjDetConfidence);
    NODELET_DEBUG_STREAM(" * Object tracking\t\t-> " << (mObjDetTracking?"ENABLED":"DISABLED"));
    NODELET_DEBUG_STREAM(" * People detection\t\t-> " << (mObjDetPeople?"ENABLED":"DISABLED"));
    NODELET_DEBUG_STREAM(" * Vehicles detection\t\t-> " << (mObjDetVehicles?"ENABLED":"DISABLED"));

    mObjDetEnabled = true;
    res.done = true;

    return res.done;
}

/* \brief Service callback to stop_object_detection service
     */
bool ZEDWrapperNodelet::on_stop_object_detection(zed_wrapper::stop_object_detection::Request& req,
                                                 zed_wrapper::stop_object_detection::Response& res) {
    if( mObjDetEnabled ) {
        mObjDetMutex.lock();
        stop_obj_detect();
        mObjDetMutex.unlock();

        res.done = true;
    } else {
        res.done = false;
    }

    return res.done;
}

void ZEDWrapperNodelet::detectObjects(bool publishObj, bool publishViz) {

    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    objectTracker_parameters_rt.detection_confidence_threshold = mObjDetConfidence;
    objectTracker_parameters_rt.object_class_filter = mObjDetFilter;

    sl::Objects objects;

    sl::ERROR_CODE objDetRes = mZed.retrieveObjects(objects, objectTracker_parameters_rt);

    if (objDetRes != sl::ERROR_CODE::SUCCESS) {
        NODELET_WARN_STREAM("Object Detection error: " << sl::toString(objDetRes));
        return;
    }

    // NODELET_DEBUG_STREAM("Detected " << objects.object_list.size() << " objects");

    size_t objCount = objects.object_list.size();

    zed_wrapper::objects objMsg;

    objMsg.objects.resize(objCount);

    std_msgs::Header header;
    header.stamp = mFrameTimestamp;
    //header.frame_id = mCameraFrameId;
    header.frame_id = mLeftCamFrameId;

    visualization_msgs::MarkerArray objMarkersMsg;
    //objMarkersMsg.markers.resize(objCount * 3);

    for (size_t i = 0; i < objCount; i++) {
        sl::ObjectData data = objects.object_list[i];

        if (publishObj) {
            objMsg.objects[i].header = header;

            objMsg.objects[i].label = sl::toString(data.label).c_str();
            objMsg.objects[i].label_id = data.id;
            objMsg.objects[i].confidence = data.confidence;

            objMsg.objects[i].tracking_state = static_cast<int8_t>(data.tracking_state);

            objMsg.objects[i].position.x = data.position.x;
            objMsg.objects[i].position.y = data.position.y;
            objMsg.objects[i].position.z = data.position.z;

            objMsg.objects[i].linear_vel.x = data.velocity.x;
            objMsg.objects[i].linear_vel.y = data.velocity.y;
            objMsg.objects[i].linear_vel.z = data.velocity.z;

            for (int c = 0; c < data.bounding_box_2d.size(); c++) {
                objMsg.objects[i].bbox_2d[c].x = data.bounding_box_2d[c].x;
                objMsg.objects[i].bbox_2d[c].y = data.bounding_box_2d[c].y;
                objMsg.objects[i].bbox_2d[c].z = 0.0f;
            }

            for (int c = 0; c < data.bounding_box.size(); c++) {
                objMsg.objects[i].bbox_3d[c].x = data.bounding_box[c].x;
                objMsg.objects[i].bbox_3d[c].y = data.bounding_box[c].y;
                objMsg.objects[i].bbox_3d[c].z = data.bounding_box[c].z;
            }
        }

        if (publishViz) {

            if( data.bounding_box.size()!=8 ) {
                continue; // No 3D information available
            }

            visualization_msgs::Marker bbx_marker;

            bbx_marker.header = header;
            bbx_marker.ns = "bbox";
            bbx_marker.id = data.id;
            bbx_marker.type = visualization_msgs::Marker::CUBE;
            bbx_marker.action = visualization_msgs::Marker::ADD;
            bbx_marker.pose.position.x = data.position.x;
            bbx_marker.pose.position.y = data.position.y;
            bbx_marker.pose.position.z = data.position.z;
            bbx_marker.pose.orientation.x = 0.0;
            bbx_marker.pose.orientation.y = 0.0;
            bbx_marker.pose.orientation.z = 0.0;
            bbx_marker.pose.orientation.w = 1.0;

            bbx_marker.scale.x = fabs(data.bounding_box[0].x - data.bounding_box[1].x);
            bbx_marker.scale.y = fabs(data.bounding_box[0].y - data.bounding_box[3].y);
            bbx_marker.scale.z = fabs(data.bounding_box[0].z - data.bounding_box[4].z);
            sl::float3 color = generateColorClass(data.id);
            bbx_marker.color.b = color.b;
            bbx_marker.color.g = color.g;
            bbx_marker.color.r = color.r;
            bbx_marker.color.a = 0.4;
            bbx_marker.lifetime = ros::Duration(0.3);
            bbx_marker.frame_locked = true;

            objMarkersMsg.markers.push_back(bbx_marker);

            visualization_msgs::Marker label;
            label.header = header;
            label.lifetime = ros::Duration(0.3);
            label.action = visualization_msgs::Marker::ADD;
            label.id = data.id;
            label.ns = "label";
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.scale.z = 0.1;

            label.color.r = 255-color.r;
            label.color.g = 255-color.g;
            label.color.b = 255-color.b;
            label.color.a = 0.75f;

            label.pose.position.x = data.position.x;
            label.pose.position.y = data.position.y;
            label.pose.position.z = data.position.z+1.1*bbx_marker.scale.z/2;

            label.text = std::to_string(data.id) + ". " + std::string(sl::toString(data.label).c_str());

            objMarkersMsg.markers.push_back(label);

            //            visualization_msgs::Marker lines;
            //            visualization_msgs::Marker label;
            //            visualization_msgs::Marker spheres;

            //            lines.pose.orientation.w = label.pose.orientation.w = spheres.pose.orientation.w = 1.0;

            //            lines.header = header;
            //            lines.lifetime = ros::Duration(0.3);
            //            lines.action = visualization_msgs::Marker::ADD;
            //            lines.id = data.id;
            //            lines.ns = "bbox";
            //            lines.type = visualization_msgs::Marker::LINE_LIST;
            //            lines.scale.x = 0.005;

            //            label.header = header;
            //            label.lifetime = ros::Duration(0.3);
            //            label.action = visualization_msgs::Marker::ADD;
            //            label.id = data.id;
            //            label.ns = "label";
            //            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            //            //label.scale.x = 0.05;
            //            //label.scale.y = 0.05;
            //            label.scale.z = 0.1;

            //            spheres.header = header;
            //            spheres.lifetime = ros::Duration(0.3);
            //            spheres.action = visualization_msgs::Marker::ADD;
            //            spheres.id = data.id;
            //            spheres.ns = "spheres";
            //            spheres.type = visualization_msgs::Marker::SPHERE_LIST;
            //            spheres.scale.x = 0.02;
            //            spheres.scale.y = 0.02;
            //            spheres.scale.z = 0.02;

            //            spheres.color.r = data.tracking_state == sl::OBJECT_TRACKING_STATE::OK ?
            //                        0.1f : (data.tracking_state == sl::OBJECT_TRACKING_STATE::SEARCHING ?
            //                                    0.9f : 0.3f);
            //            spheres.color.g = data.tracking_state == sl::OBJECT_TRACKING_STATE::OK ?
            //                        0.9f : (data.tracking_state == sl::OBJECT_TRACKING_STATE::SEARCHING ?
            //                                    0.1f : 0.3f);
            //            spheres.color.b = data.tracking_state == sl::OBJECT_TRACKING_STATE::OK ?
            //                        0.1f : (data.tracking_state == sl::OBJECT_TRACKING_STATE::SEARCHING ?
            //                                    0.1f : 0.3f);
            //            spheres.color.a = 0.75f;

            //            sl::float3 color = generateColorClass(data.id);

            //            lines.color.r = color.r;
            //            lines.color.g = color.g;
            //            lines.color.b = color.b;
            //            lines.color.a = 0.75f;

            //            label.color.r = color.r;
            //            label.color.g = color.g;
            //            label.color.b = color.b;
            //            label.color.a = 0.75f;

            //            label.pose.position.x = data.position.x;
            //            label.pose.position.y = data.position.y;
            //            label.pose.position.z = data.position.z;

            //            label.text = std::to_string(data.id) + ". " + std::string(sl::toString(data.label).c_str());

            //            geometry_msgs::Point p0;
            //            geometry_msgs::Point p1;

            //            // Centroid
            //            p0.x = data.position.x;
            //            p0.y = data.position.y;
            //            p0.z = data.position.z;

            //            spheres.points.resize(9);
            //            spheres.points[8] = p0;;

            //            lines.points.resize(24);

            //            int linePtIdx = 0;

            //            // Top square
            //            for (int v = 0; v < 3; v++) {
            //                lines.points[linePtIdx].x = data.bounding_box[v].x;
            //                lines.points[linePtIdx].y = data.bounding_box[v].y;
            //                lines.points[linePtIdx].z = data.bounding_box[v].z;
            //                linePtIdx++;

            //                lines.points[linePtIdx].x = data.bounding_box[v + 1].x;
            //                lines.points[linePtIdx].y = data.bounding_box[v + 1].y;
            //                lines.points[linePtIdx].z = data.bounding_box[v + 1].z;
            //                linePtIdx++;
            //            }

            //            lines.points[linePtIdx].x = data.bounding_box[3].x;
            //            lines.points[linePtIdx].y = data.bounding_box[3].y;
            //            lines.points[linePtIdx].z = data.bounding_box[3].z;
            //            linePtIdx++;

            //            lines.points[linePtIdx].x = data.bounding_box[0].x;
            //            lines.points[linePtIdx].y = data.bounding_box[0].y;
            //            lines.points[linePtIdx].z = data.bounding_box[0].z;
            //            linePtIdx++;

            //            // Bottom square
            //            for (int v = 4; v < 7; v++) {
            //                lines.points[linePtIdx].x = data.bounding_box[v].x;
            //                lines.points[linePtIdx].y = data.bounding_box[v].y;
            //                lines.points[linePtIdx].z = data.bounding_box[v].z;
            //                linePtIdx++;

            //                lines.points[linePtIdx].x = data.bounding_box[v + 1].x;
            //                lines.points[linePtIdx].y = data.bounding_box[v + 1].y;
            //                lines.points[linePtIdx].z = data.bounding_box[v + 1].z;
            //                linePtIdx++;
            //            }

            //            lines.points[linePtIdx].x = data.bounding_box[7].x;
            //            lines.points[linePtIdx].y = data.bounding_box[7].y;
            //            lines.points[linePtIdx].z = data.bounding_box[7].z;
            //            linePtIdx++;

            //            lines.points[linePtIdx].x = data.bounding_box[4].x;
            //            lines.points[linePtIdx].y = data.bounding_box[4].y;
            //            lines.points[linePtIdx].z = data.bounding_box[4].z;
            //            linePtIdx++;

            //            // Lateral lines and vertex spheres
            //            for (int v = 0; v < 4; v++) {
            //                lines.points[linePtIdx].x = data.bounding_box[v].x;
            //                lines.points[linePtIdx].y = data.bounding_box[v].y;
            //                lines.points[linePtIdx].z = data.bounding_box[v].z;
            //                linePtIdx++;

            //                lines.points[linePtIdx].x = data.bounding_box[v + 4].x;
            //                lines.points[linePtIdx].y = data.bounding_box[v + 4].y;
            //                lines.points[linePtIdx].z = data.bounding_box[v + 4].z;
            //                linePtIdx++;

            //                spheres.points[v * 2].x = data.bounding_box[v].x;
            //                spheres.points[v * 2].y = data.bounding_box[v].y;
            //                spheres.points[v * 2].z = data.bounding_box[v].z;

            //                spheres.points[v * 2 + 1].x = data.bounding_box[v + 4].x;
            //                spheres.points[v * 2 + 1].y = data.bounding_box[v + 4].y;
            //                spheres.points[v * 2 + 1].z = data.bounding_box[v + 4].z;
            //            }

            //            objMarkersMsg.markers[i * 3] = lines;
            //            objMarkersMsg.markers[i * 3 + 1] = label;
            //            objMarkersMsg.markers[i * 3 + 2] = spheres;
        }
    }

    if (publishObj) {
        mPubObjDet.publish(objMsg);
    }

    if (mPubObjDetViz) {
        mPubObjDetViz.publish(objMarkersMsg);
    }

}

} // namespace
