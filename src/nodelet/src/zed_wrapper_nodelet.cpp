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
#include "sl_tools.h"

#include <opencv2/imgproc/imgproc.hpp>

#ifndef NDEBUG
#include <ros/console.h>
#endif

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <cv_bridge/cv_bridge.h>

// >>>>> Backward compatibility
#define COORDINATE_SYSTEM_IMAGE                     static_cast<sl::COORDINATE_SYSTEM>(0)
#define COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP         static_cast<sl::COORDINATE_SYSTEM>(3)
#define COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD   static_cast<sl::COORDINATE_SYSTEM>(5)
// <<<<< Backward compatibility

using namespace std;

namespace zed_wrapper {

    ZEDWrapperNodelet::ZEDWrapperNodelet() : Nodelet() {}

    ZEDWrapperNodelet::~ZEDWrapperNodelet() {
        mDevicePollThread.get()->join();
    }

    void ZEDWrapperNodelet::onInit() {
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
        mInitialTrackPose.resize(6);
        mOpenniDepthMode = false;
        mTerrainMap = false;
        mTrackingReady = false;
        mMappingReady = false;
        mLocalTerrainPubRate = 5.0;
        mGlobalTerrainPubRate = 1.0;

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
        mNhNs.param<std::string>("base_frame", mBaseFrameId, "zed_camera_center");
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
        mNhNs.getParam("terrain_mapping", mTerrainMap); // TODO Check SDK version
        int tmp_sn = 0;
        mNhNs.getParam("serial_number", tmp_sn);

        if (tmp_sn > 0) {
            mZedSerialNumber = static_cast<int>(tmp_sn);
        }

        mNhNs.getParam("camera_model", mZedUserCamModel);
        // Publish odometry tf
        mNhNs.param<bool>("publish_tf", mPublishTf, true);

        if (mZedSerialNumber > 0) {
            NODELET_INFO_STREAM("SN : " << mZedSerialNumber);
        }

        // Print order frames
        NODELET_INFO_STREAM("pose_frame \t\t   -> " << mMapFrameId);
        NODELET_INFO_STREAM("odometry_frame \t\t   -> " << mOdometryFrameId);
        NODELET_INFO_STREAM("base_frame \t\t   -> " << mBaseFrameId);
        NODELET_INFO_STREAM("imu_link \t\t   -> " << mImuFrameId);
        NODELET_INFO_STREAM("left_camera_frame \t   -> " << mLeftCamFrameId);
        NODELET_INFO_STREAM("left_camera_optical_frame  -> " << mLeftCamOptFrameId);
        NODELET_INFO_STREAM("right_camera_frame \t   -> " << mRightCamFrameId);
        NODELET_INFO_STREAM("right_camera_optical_frame -> " << mRightCamOptFrameId);
        NODELET_INFO_STREAM("depth_frame \t\t   -> " << mDepthFrameId);
        NODELET_INFO_STREAM("depth_optical_frame \t   -> " << mDepthOptFrameId);
        NODELET_INFO_STREAM("disparity_frame \t   -> " << mDisparityFrameId);
        NODELET_INFO_STREAM("disparity_optical_frame    -> " << mDisparityOptFrameId);
        // Status of map TF
        NODELET_INFO_STREAM("Publish " << mMapFrameId << " ["
                            << (mPublishTf ? "TRUE" : "FALSE") << "]");
        // Status of odometry TF
        // NODELET_INFO_STREAM("Publish " << odometry_frame_id << " [" << (publish_tf
        // ? "TRUE" : "FALSE") << "]");
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
        string pose_topic = "map";
        string odometry_topic = "odom";
        string imu_topic = "imu/data";
        string imu_topic_raw = "imu/data_raw";
        string loc_height_map_topic = "map/loc_map_heightmap";
        string loc_cost_map_topic = "map/loc_map_costmap";
        string glob_height_map_topic = "map/loc_map_heightmap";
        string glob_cost_map_topic = "map/loc_map_costmap";
        //string gridmap_topic = "map/gridmap";
        string height_map_image_topic = "map/height_map_image";
        string color_map_image_topic = "map/color_map_image";
        string travers_map_image_topic = "map/travers_map_image";
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
        mNhNs.getParam("odometry_topic", odometry_topic);
        mNhNs.getParam("imu_topic", imu_topic);
        mNhNs.getParam("imu_topic_raw", imu_topic_raw);
        mNhNs.getParam("imu_pub_rate", mImuPubRate);
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
        // Initialize tf2 transformation
        mNhNs.getParam("initial_tracking_pose", mInitialTrackPose);
        set_pose(mInitialTrackPose[0], mInitialTrackPose[1], mInitialTrackPose[2],
                 mInitialTrackPose[3], mInitialTrackPose[4], mInitialTrackPose[5]);
        // Initialization transformation listener
        mTfBuffer.reset(new tf2_ros::Buffer);
        mTfListener.reset(new tf2_ros::TransformListener(*mTfBuffer));

        // Try to initialize the ZED
        if (!mSvoFilepath.empty()) {
            mZedParams.svo_input_filename = mSvoFilepath.c_str();
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
                        mZed.close();
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

        if (mVerMajor < 2) {
            NODELET_WARN_STREAM("Please consider to upgrade to latest SDK version to "
                                "get better performances");
            mZedParams.coordinate_system = COORDINATE_SYSTEM_IMAGE;
            NODELET_INFO_STREAM("Camera coordinate system : COORDINATE_SYSTEM_IMAGE");
            mIdxX = 2;
            mIdxY = 0;
            mIdxZ = 1;
            mSignX = 1;
            mSignY = -1;
            mSignZ = -1;
        } else if (mVerMajor == 2 && mVerMinor < 5) {
            NODELET_WARN_STREAM("Please consider to upgrade to latest SDK version to "
                                "get latest features");
            mZedParams.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;
            NODELET_INFO_STREAM(
                "Camera coordinate system : COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP");
            mIdxX = 1;
            mIdxY = 0;
            mIdxZ = 2;
            mSignX = 1;
            mSignY = -1;
            mSignZ = 1;
        } else {
            mZedParams.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;
            NODELET_INFO_STREAM(
                "Camera coordinate system : COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD");
            mIdxX = 0;
            mIdxY = 1;
            mIdxZ = 2;
            mSignX = 1;
            mSignY = 1;
            mSignZ = 1;
        }

        mZedParams.coordinate_units = sl::UNIT_METER;
        mZedParams.depth_mode = static_cast<sl::DEPTH_MODE>(mCamQuality);
        mZedParams.sdk_verbose = mVerbose;
        mZedParams.sdk_gpu_id = mGpuId;
        mZedParams.depth_stabilization = mDepthStabilization;
        sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;

        while (err != sl::SUCCESS) {
            err = mZed.open(mZedParams);
            NODELET_INFO_STREAM(toString(err));
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            // Ctrl+C check
            if (!mNhNs.ok()) {
                mZed.close();
                return;
            }
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
        mPubOdom = mNh.advertise<nav_msgs::Odometry>(odometry_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << odometry_topic);

        // Terrain Mapping publishers
        mPubLocalHeightMap = mNh.advertise<nav_msgs::OccupancyGrid>(loc_height_map_topic, 1); // local height map
        NODELET_INFO_STREAM("Advertised on topic " << loc_height_map_topic);
        mPubLocalCostMap = mNh.advertise<nav_msgs::OccupancyGrid>(loc_cost_map_topic, 1); // local cost map
        NODELET_INFO_STREAM("Advertised on topic " << loc_cost_map_topic);
        mPubGlobalHeightMap = mNh.advertise<nav_msgs::OccupancyGrid>(glob_height_map_topic, 1, true); // global height map latched
        NODELET_INFO_STREAM("Advertised on topic " << glob_height_map_topic);
        mPubGlobalCostMap = mNh.advertise<nav_msgs::OccupancyGrid>(glob_cost_map_topic, 1, true); // global cost map latched
        NODELET_INFO_STREAM("Advertised on topic " << glob_cost_map_topic);
        //mPubGridMap = mNh.advertise<grid_map_msgs::GridMap>(gridmap_topic, 1);
        //NODELET_INFO_STREAM("Advertised on topic " << gridmap_topic);
        mPubGlobalHeightMapImg = mNh.advertise<sensor_msgs::Image>(height_map_image_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << height_map_image_topic);
        mPubGlobalColorMapImg = mNh.advertise<sensor_msgs::Image>(color_map_image_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << color_map_image_topic);
        mPubGlobalTravMapImg = mNh.advertise<sensor_msgs::Image>(travers_map_image_topic, 1);
        NODELET_INFO_STREAM("Advertised on topic " << travers_map_image_topic);

        // Imu publisher
        if (mImuPubRate > 0 && mZedRealCamModel == sl::MODEL_ZED_M) {
            mPubImu = mNh.advertise<sensor_msgs::Imu>(imu_topic, 500);
            NODELET_INFO_STREAM("Advertised on topic " << imu_topic << " @ "
                                << mImuPubRate << " Hz");
            mPubImuRaw = mNh.advertise<sensor_msgs::Imu>(imu_topic_raw, 500);
            NODELET_INFO_STREAM("Advertised on topic " << imu_topic_raw << " @ "
                                << mImuPubRate << " Hz");
            mImuTime = ros::Time::now();
            mPubImuTimer = mNhNs.createTimer(ros::Duration(1.0 / mImuPubRate),
                                             &ZEDWrapperNodelet::imuPubCallback, this);
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

        // Start pool thread
        mDevicePollThread = boost::shared_ptr<boost::thread>(
                                new boost::thread(boost::bind(&ZEDWrapperNodelet::device_poll, this)));
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

    sensor_msgs::ImagePtr
    ZEDWrapperNodelet::imageToROSmsg(cv::Mat img, const std::string encodingType,
                                     std::string frameId, ros::Time t) {
        sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::Image& imgMessage = *ptr;
        imgMessage.header.stamp = t;
        imgMessage.header.frame_id = frameId;
        imgMessage.height = static_cast<unsigned int>(img.rows);
        imgMessage.width = static_cast<unsigned int>(img.cols);
        imgMessage.encoding = encodingType;
        int num = 1; // for endianness detection
        imgMessage.is_bigendian = !(*(char*)&num == 1);
        imgMessage.step = static_cast<unsigned int>(img.step);
        size_t size = imgMessage.step * img.rows;
        imgMessage.data.resize(size);

        if (img.isContinuous()) {
            memcpy((char*)(&imgMessage.data[0]), img.data, size);
        } else {
            uchar* opencvData = img.data;
            uchar* rosData = (uchar*)(&imgMessage.data[0]);

            #pragma omp parallel for
            for (unsigned int i = 0; i < img.rows; i++) {
                memcpy(rosData, opencvData, imgMessage.step);
                rosData += imgMessage.step;
                opencvData += img.step;
            }
        }

        return ptr;
    }

    void ZEDWrapperNodelet::set_pose(float xt, float yt, float zt, float rr,
                                     float pr, float yr) {
        // ROS pose
        tf2::Quaternion q;
        q.setRPY(rr, pr, yr);
        tf2::Vector3 orig(xt, yt, zt);
        mBase2OdomTransf.setOrigin(orig);
        mBase2OdomTransf.setRotation(q);
        mOdom2MapTransf.setIdentity();
        // SL pose
        sl::float4 q_vec;
        q_vec[0] = q.x();
        q_vec[1] = q.y();
        q_vec[2] = q.z();
        q_vec[3] = q.w();
        sl::Orientation r(q_vec);
        mInitialPoseSl.setTranslation(sl::Translation(xt, yt, zt));
        mInitialPoseSl.setOrientation(r);
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
            mOdom2MapTransf.setIdentity();
            mBase2OdomTransf.setIdentity();
        } else {
            set_pose(mInitialTrackPose[0], mInitialTrackPose[1], mInitialTrackPose[2],
                     mInitialTrackPose[3], mInitialTrackPose[4], mInitialTrackPose[5]);
        }

        mZed.resetTracking(mInitialPoseSl);
        return true;
    }

    bool ZEDWrapperNodelet::on_reset_odometry(
        zed_wrapper::reset_odometry::Request& req,
        zed_wrapper::reset_odometry::Response& res) {
        mInitOdomWithPose = true;
        res.reset_done = true;
        return true;
    }

    void ZEDWrapperNodelet::start_tracking() {
        NODELET_INFO_STREAM("Starting Tracking");
        mNhNs.getParam("odometry_DB", mOdometryDb);
        mNhNs.getParam("pose_smoothing", mPoseSmoothing);
        mNhNs.getParam("spatial_memory", mSpatialMemory);
        mNhNs.getParam("floor_alignment", mFloorAlignment);

        if (mTerrainMap && !mFloorAlignment) {
            NODELET_INFO_STREAM("Floor Alignment required by Terrain Mapping algorithm");
            mFloorAlignment = true;
        }

        if (mZedRealCamModel == sl::MODEL_ZED_M) {
            mNhNs.getParam("init_odom_with_imu", mInitOdomWithPose);
            NODELET_INFO_STREAM(
                "Init Odometry with first IMU data : " << mInitOdomWithPose);
        } else {
            mInitOdomWithPose = false;
        }

        if (mInitialTrackPose.size() != 6) {
            NODELET_WARN_STREAM("Invalid Initial Pose size (" << mInitialTrackPose.size()
                                << "). Using Identity");
            mInitialPoseSl.setIdentity();
            mOdom2MapTransf.setIdentity();
            mOdom2MapTransf.setIdentity();
        } else {
            set_pose(mInitialTrackPose[0], mInitialTrackPose[1], mInitialTrackPose[2],
                     mInitialTrackPose[3], mInitialTrackPose[4], mInitialTrackPose[5]);
        }

        if (mOdometryDb != "" && !sl_tools::file_exist(mOdometryDb)) {
            mOdometryDb = "";
            NODELET_WARN("odometry_DB path doesn't exist or is unreachable.");
        }

        // Tracking parameters
        sl::TrackingParameters trackParams;
        trackParams.area_file_path = mOdometryDb.c_str();
        trackParams.enable_pose_smoothing = mPoseSmoothing;
        NODELET_INFO_STREAM("Pose Smoothing : " << trackParams.enable_pose_smoothing);
        trackParams.enable_spatial_memory = mSpatialMemory;
        NODELET_INFO_STREAM("Spatial Memory : " << trackParams.enable_spatial_memory);
        trackParams.enable_floor_alignment = mFloorAlignment;
        NODELET_INFO_STREAM("Floor Alignment : " << trackParams.enable_floor_alignment);
        trackParams.initial_world_transform = mInitialPoseSl;
        mZed.enableTracking(trackParams);
        mTrackingActivated = true;
        NODELET_INFO("Tracking ENABLED");

        if (mTerrainMap) { // TODO Check SDK version
            start_mapping();
        }
    }

    void ZEDWrapperNodelet::start_mapping() {
        if (!mTerrainMap) {
            return;
        }

        mNhNs.getParam("terrain_pub_rate", mLocalTerrainPubRate);

        sl::TerrainMappingParameters terrainParams;
        float agent_step = 0.05f; // TODO Expose parameter to launch file
        float agent_slope = 20.f/*degrees*/; // TODO Expose parameter to launch file
        float agent_radius = 0.18f; // TODO Expose parameter to launch file
        float agent_height = 0.8f; // TODO Expose parameter to launch file
        float agent_roughness = 0.05f; // TODO Expose parameter to launch file
        terrainParams.setAgentParameters(sl::UNIT_METER, agent_step, agent_slope, agent_radius,
                                         agent_height, agent_roughness);
        float max_depth = 3.5f; // TODO Expose parameter to launch file
        mMapMaxHeight = .5f; // TODO Expose parameter to launch file
        float height_resol = .025f; // TODO Expose parameter to launch file

        sl::TerrainMappingParameters::GRID_RESOLUTION grid_resolution = sl::TerrainMappingParameters::GRID_RESOLUTION::LOW; // TODO Expose parameter to launch file
        mTerrainMapRes = terrainParams.setGridResolution(grid_resolution); // TODO: Check this value when bug is fixed in SDK

        NODELET_INFO_STREAM("Terrain Grid Resolution " << mTerrainMapRes << "m");
        NODELET_INFO_STREAM("Terrain Cutting height " << terrainParams.setHeightThreshold(sl::UNIT_METER, mMapMaxHeight) << "m");
        NODELET_INFO_STREAM("Terrain Z Resolution " << terrainParams.setZResolution(sl::UNIT_METER, height_resol) << "m");
        NODELET_INFO_STREAM("Terrain Max range " << terrainParams.setRange(sl::UNIT_METER, max_depth) << "m");

        terrainParams.enable_traversability_cost_computation = true; // TODO Expose parameter to launch file
        terrainParams.enable_dynamic_extraction = true; // TODO Expose parameter to launch file
        terrainParams.enable_color_extraction = true; // TODO Expose parameter to launch file

        if (mZed.enableTerrainMapping(terrainParams) != sl::SUCCESS) {
            NODELET_WARN_STREAM("Terrain Mapping: NOT ENABLED");
            mMappingReady = false;
            return;
        }

        initMapMsgs(200);
        mMappingReady = true;
        //mGlobalMapsUpdateReq = false;

        // Start Terrain Mapping Timer
        mLocalTerrainTimer = mNhNs.createTimer(ros::Duration(1.0 / mLocalTerrainPubRate),
                                               &ZEDWrapperNodelet::localTerrainCallback, this);
        NODELET_INFO_STREAM("Local Terrain Mapping: ENABLED @ " << mLocalTerrainPubRate << "Hz");

        mGlobalTerrainTimer = mNhNs.createTimer(ros::Duration(1.0 / mGlobalTerrainPubRate),
                                                &ZEDWrapperNodelet::globalTerrainCallback, this);
        NODELET_INFO_STREAM("Global Terrain Mapping: ENABLED @ " << mGlobalTerrainPubRate << "Hz");
    }

    void ZEDWrapperNodelet::publishOdom(tf2::Transform odom_base_transform,
                                        ros::Time t) {
        nav_msgs::Odometry odom;
        odom.header.stamp = t;
        odom.header.frame_id = mOdometryFrameId; // odom_frame
        odom.child_frame_id = mBaseFrameId;      // base_frame
        // conversion from Tranform to message
        geometry_msgs::Transform base2 = tf2::toMsg(odom_base_transform);
        // Add all value in odometry message
        odom.pose.pose.position.x = base2.translation.x;
        odom.pose.pose.position.y = base2.translation.y;
        odom.pose.pose.position.z = base2.translation.z;
        odom.pose.pose.orientation.x = base2.rotation.x;
        odom.pose.pose.orientation.y = base2.rotation.y;
        odom.pose.pose.orientation.z = base2.rotation.z;
        odom.pose.pose.orientation.w = base2.rotation.w;
        // Publish odometry message
        mPubOdom.publish(odom);
    }

    void ZEDWrapperNodelet::publishOdomFrame(tf2::Transform baseTransform,
            ros::Time t) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = t;
        transformStamped.header.frame_id = mOdometryFrameId;
        transformStamped.child_frame_id = mBaseFrameId;
        // conversion from Tranform to message
        transformStamped.transform = tf2::toMsg(baseTransform);
        // Publish transformation
        mTransformOdomBroadcaster.sendTransform(transformStamped);
    }

    void ZEDWrapperNodelet::publishPose(tf2::Transform poseBaseTransform,
                                        ros::Time t) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = t;
        pose.header.frame_id = mMapFrameId; // map_frame
        // conversion from Tranform to message
        geometry_msgs::Transform base2 = tf2::toMsg(poseBaseTransform);
        // Add all value in Pose message
        pose.pose.position.x = base2.translation.x;
        pose.pose.position.y = base2.translation.y;
        pose.pose.position.z = base2.translation.z;
        pose.pose.orientation.x = base2.rotation.x;
        pose.pose.orientation.y = base2.rotation.y;
        pose.pose.orientation.z = base2.rotation.z;
        pose.pose.orientation.w = base2.rotation.w;
        // Publish odometry message
        mPubPose.publish(pose);
    }

    void ZEDWrapperNodelet::publishPoseFrame(tf2::Transform baseTransform,
            ros::Time t) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = t;
        transformStamped.header.frame_id = mMapFrameId;
        transformStamped.child_frame_id = mOdometryFrameId;
        // conversion from Tranform to message
        transformStamped.transform = tf2::toMsg(baseTransform);
        // Publish transformation
        mTransformPoseBroadcaster.sendTransform(transformStamped);
    }

    void ZEDWrapperNodelet::publishImuFrame(tf2::Transform baseTransform) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = mImuTime;
        transformStamped.header.frame_id = mBaseFrameId;
        transformStamped.child_frame_id = mImuFrameId;
        // conversion from Tranform to message
        transformStamped.transform = tf2::toMsg(baseTransform);
        // Publish transformation
        mTransformImuBroadcaster.sendTransform(transformStamped);
    }

    void ZEDWrapperNodelet::publishImage(cv::Mat img,
                                         image_transport::Publisher& pubImg,
                                         string imgFrameId, ros::Time t) {
        pubImg.publish(
            imageToROSmsg(img, sensor_msgs::image_encodings::BGR8, imgFrameId, t));
    }

    void ZEDWrapperNodelet::publishDepth(cv::Mat depth, ros::Time t) {
        string encoding;

        if (mOpenniDepthMode) {
            depth *= 1000.0f;
            depth.convertTo(depth, CV_16UC1); // in mm, rounded
            encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        } else {
            encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        }

        mPubDepth.publish(imageToROSmsg(depth, encoding, mDepthOptFrameId, t));
    }

    void ZEDWrapperNodelet::publishDisparity(cv::Mat disparity, ros::Time t) {
        sl::CameraInformation zedParam =
            mZed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight));
        sensor_msgs::ImagePtr disparity_image = imageToROSmsg(
                disparity, sensor_msgs::image_encodings::TYPE_32FC1, mDisparityFrameId, t);
        stereo_msgs::DisparityImage msg;
        msg.image = *disparity_image;
        msg.header = msg.image.header;
        msg.f = zedParam.calibration_parameters.left_cam.fx;
        msg.T = zedParam.calibration_parameters.T.x;
        msg.min_disparity = msg.f * msg.T / mZed.getDepthMaxRangeValue();
        msg.max_disparity = msg.f * msg.T / mZed.getDepthMinRangeValue();
        mPubDisparity.publish(msg);
    }

    void ZEDWrapperNodelet::publishPointCloud(uint32_t width, uint32_t height) {
        pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
        point_cloud.width = width;
        point_cloud.height = height;
        uint32_t size = width * height;
        point_cloud.points.resize(size);
        sl::Vector4<float>* cpu_cloud = mCloud.getPtr<sl::float4>();

        #pragma omp parallel for
        for (size_t i = 0; i < size; i++) {
            // COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD
            point_cloud.points[i].x = mSignX * cpu_cloud[i][mIdxX];
            point_cloud.points[i].y = mSignY * cpu_cloud[i][mIdxY];
            point_cloud.points[i].z = mSignZ * cpu_cloud[i][mIdxZ];
            point_cloud.points[i].rgb = cpu_cloud[i][3];
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(point_cloud,
                      output); // Convert the point cloud to a ROS message
        output.header.frame_id =
            mPointCloudFrameId; // Set the header values of the ROS message
        output.header.stamp = mPointCloudTime;
        output.height = height;
        output.width = width;
        output.is_bigendian = false;
        output.is_dense = false;
        mPubCloud.publish(output);
    }

    void ZEDWrapperNodelet::publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg,
                                           ros::Publisher pubCamInfo, ros::Time t) {
        static unsigned int seq = 0;
        camInfoMsg->header.stamp = t;
        camInfoMsg->header.seq = seq;
        pubCamInfo.publish(camInfoMsg);
        seq++;
    }

    void ZEDWrapperNodelet::fillCamInfo(
        sl::Camera& zed, sensor_msgs::CameraInfoPtr left_cam_info_msg,
        sensor_msgs::CameraInfoPtr right_cam_info_msg, string leftFrameId,
        string rightFrameId, bool rawParam /*= false*/) {
        sl::CalibrationParameters zedParam;

        if (rawParam)
            zedParam = zed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight))
                       .calibration_parameters_raw;
        else
            zedParam = zed.getCameraInformation(sl::Resolution(mMatWidth, mMatHeight))
                       .calibration_parameters;

        float baseline = zedParam.T[0];
        left_cam_info_msg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
        right_cam_info_msg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
        left_cam_info_msg->D.resize(5);
        right_cam_info_msg->D.resize(5);
        left_cam_info_msg->D[0] = zedParam.left_cam.disto[0];   // k1
        left_cam_info_msg->D[1] = zedParam.left_cam.disto[1];   // k2
        left_cam_info_msg->D[2] = zedParam.left_cam.disto[4];   // k3
        left_cam_info_msg->D[3] = zedParam.left_cam.disto[2];   // p1
        left_cam_info_msg->D[4] = zedParam.left_cam.disto[3];   // p2
        right_cam_info_msg->D[0] = zedParam.right_cam.disto[0]; // k1
        right_cam_info_msg->D[1] = zedParam.right_cam.disto[1]; // k2
        right_cam_info_msg->D[2] = zedParam.right_cam.disto[4]; // k3
        right_cam_info_msg->D[3] = zedParam.right_cam.disto[2]; // p1
        right_cam_info_msg->D[4] = zedParam.right_cam.disto[3]; // p2
        left_cam_info_msg->K.fill(0.0);
        right_cam_info_msg->K.fill(0.0);
        left_cam_info_msg->K[0] = static_cast<double>(zedParam.left_cam.fx);
        left_cam_info_msg->K[2] = static_cast<double>(zedParam.left_cam.cx);
        left_cam_info_msg->K[4] = static_cast<double>(zedParam.left_cam.fy);
        left_cam_info_msg->K[5] = static_cast<double>(zedParam.left_cam.cy);
        left_cam_info_msg->K[8] = 1.0;
        right_cam_info_msg->K[0] = static_cast<double>(zedParam.right_cam.fx);
        right_cam_info_msg->K[2] = static_cast<double>(zedParam.right_cam.cx);
        right_cam_info_msg->K[4] = static_cast<double>(zedParam.right_cam.fy);
        right_cam_info_msg->K[5] = static_cast<double>(zedParam.right_cam.cy);
        right_cam_info_msg->K[8] = 1.0;
        left_cam_info_msg->R.fill(0.0);
        right_cam_info_msg->R.fill(0.0);

        for (size_t i = 0; i < 3; i++) {
            // identity
            right_cam_info_msg->R[i + i * 3] = 1;
            left_cam_info_msg->R[i + i * 3] = 1;
        }

        if (rawParam) {
            cv::Mat R_ = sl_tools::convertRodrigues(zedParam.R);
            float* p = (float*)(R_.data);

            for (size_t i = 0; i < 9; i++) {
                right_cam_info_msg->R[i] = static_cast<double>(p[i]);
            }
        }

        left_cam_info_msg->P.fill(0.0);
        right_cam_info_msg->P.fill(0.0);
        left_cam_info_msg->P[0] = static_cast<double>(zedParam.left_cam.fx);
        left_cam_info_msg->P[2] = static_cast<double>(zedParam.left_cam.cx);
        left_cam_info_msg->P[5] = static_cast<double>(zedParam.left_cam.fy);
        left_cam_info_msg->P[6] = static_cast<double>(zedParam.left_cam.cy);
        left_cam_info_msg->P[10] = 1.0;
        // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
        right_cam_info_msg->P[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
        right_cam_info_msg->P[0] = static_cast<double>(zedParam.right_cam.fx);
        right_cam_info_msg->P[2] = static_cast<double>(zedParam.right_cam.cx);
        right_cam_info_msg->P[5] = static_cast<double>(zedParam.right_cam.fy);
        right_cam_info_msg->P[6] = static_cast<double>(zedParam.right_cam.cy);
        right_cam_info_msg->P[10] = 1.0;
        left_cam_info_msg->width = right_cam_info_msg->width = static_cast<uint32_t>(mMatWidth);
        left_cam_info_msg->height = right_cam_info_msg->height = static_cast<uint32_t>(mMatHeight);
        left_cam_info_msg->header.frame_id = leftFrameId;
        right_cam_info_msg->header.frame_id = rightFrameId;
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

    void ZEDWrapperNodelet::localTerrainCallback(const ros::TimerEvent& e) {
        if (!mTrackingActivated) {
            NODELET_DEBUG("Tracking not yet active");
            return;
        }

        if (!mMappingReady) {
            start_mapping();
        }

        /*if (mGlobalMapsUpdateReq) {
            return;
        }*/

        mMappingReady = true;

        sl::ERROR_CODE res;
        do {
            mTerrainMutex.lock();
            res = mZed.getTerrainRequestStatusAsync();
            if (res != sl::SUCCESS) {
                mZed.requestTerrainAsync(); // if an elaboration is in progress the request is ignored
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                mTerrainMutex.unlock();
            }
        } while (res != sl::SUCCESS);


        uint32_t heightSub = mPubLocalHeightMap.getNumSubscribers();
        uint32_t costSub = mPubLocalCostMap.getNumSubscribers();
        uint32_t run = heightSub + costSub;

        if (run > 0) {
            if (mZed.retrieveTerrainAsync(mTerrain) == sl::SUCCESS) {

                NODELET_DEBUG("Local Terrain available");
                sl::timeStamp t = mTerrain.getReferenceTS();

                // Request New Terrain calculation while elaborating data
                mZed.requestTerrainAsync();
                mTerrainMutex.unlock();

                // Process Updated Terrain Chuncks
                std::vector<sl::HashKey> chunks;
                chunks = mTerrain.getUpdatedChunks();

                NODELET_DEBUG_STREAM("Terrain chunks updated (local map): " << chunks.size());

                if (chunks.size() > 0) {

                    std::vector<sl::HashKey>::iterator it;

                    float minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;

                    // Local map limits
                    for (it = chunks.begin(); it != chunks.end(); it++) {
                        sl::HashKey key = *it;
                        sl::TerrainChunk& chunk = mTerrain.getChunk(key);
                        sl::Dimension dim = chunk.getDimension();

                        if (dim.getXmin() < minX) {
                            minX = dim.getXmin();
                        }

                        if (dim.getYmin() < minY) {
                            minY = dim.getYmin();
                        }

                        if (dim.getXmax() > maxX) {
                            maxX = dim.getXmax();
                        }

                        if (dim.getYmax() > maxY) {
                            maxY = dim.getYmax();
                        }
                    }

                    float mapW = maxX - minX;
                    float mapH = maxY - minY;

                    NODELET_DEBUG_STREAM("Local map size: " << mapW << "x" << mapH << " m");

                    //#pragma omp parallel
                    {
                        for (it = chunks.begin(); it != chunks.end(); it++) {
                            sl::HashKey key = *it;
                            sl::TerrainChunk chunk = mTerrain.getChunk(key);
                            chunk2maps(chunk);
                        }
                    }

                    mMapsValid = true;
                }
            } else {
                mTerrainMutex.unlock();
                NODELET_DEBUG_STREAM("Local terrain not available");
            }
        } else {
            mTerrainMutex.unlock();
        }
    }

    void ZEDWrapperNodelet::globalTerrainCallback(const ros::TimerEvent& e) {
        if (!mTrackingActivated) {
            NODELET_DEBUG("Tracking not yet active");
            return;
        }

        if (!mMappingReady) {
            start_mapping();
        }

        mMappingReady = true;
        //mGlobalMapsUpdateReq = true;

        sl::ERROR_CODE res;
        do {
            mTerrainMutex.lock();
            res = mZed.getTerrainRequestStatusAsync();
            if (res != sl::SUCCESS) {
                mZed.requestTerrainAsync(); // if an elaboration is in progress the request is ignored
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                mTerrainMutex.unlock();
            }
        } while (res != sl::SUCCESS);

        //uint32_t gridSub = mPubGridMap.getNumSubscribers();
        uint32_t heightSub = mPubGlobalHeightMapImg.getNumSubscribers();
        uint32_t colorSub = mPubGlobalColorMapImg.getNumSubscribers();
        uint32_t travSub = mPubGlobalTravMapImg.getNumSubscribers();
        uint32_t run = /*gridSub + */heightSub + colorSub + travSub;

        if (run > 0) {
            sl::Terrain terrain;
            sl::Mat sl_heightMap, sl_colorMap, sl_traversMap;
            cv::Mat cv_heightMap, cv_colorMap, cv_traversMap;

            if (mZed.retrieveTerrainAsync(terrain) == sl::SUCCESS) {
                //mGlobalMapsUpdateReq = false;

                NODELET_DEBUG("Global Terrain available");
                sl::timeStamp t = terrain.getReferenceTS();

                // Request New Terrain calculation while elaborating data
                mZed.requestTerrainAsync();
                mTerrainMutex.unlock();


                // Height Map Image
                if (heightSub > 0 /*|| gridSub > 0*/) {
                    terrain.generateTerrainMap(sl_heightMap, sl::MAT_TYPE_32F_C1, sl::LayerName::ELEVATION);

                    if (sl_heightMap.getResolution().area() > 0) {
                        cv_heightMap = sl_tools::toCVMat(sl_heightMap);
                        mPubGlobalHeightMapImg.publish(
                            imageToROSmsg(cv_heightMap, sensor_msgs::image_encodings::TYPE_32FC1,
                                          mMapFrameId, sl_tools::slTime2Ros(t)));
                    }
                }

                // Color Map Image
                if (colorSub > 0 /*|| gridSub > 0*/) {
                    terrain.generateTerrainMap(sl_colorMap, sl::MAT_TYPE_8U_C4, sl::LayerName::COLOR);

                    if (sl_colorMap.getResolution().area() > 0) {
                        cv_colorMap = sl_tools::toCVMat(sl_colorMap);
                        mPubGlobalColorMapImg.publish(imageToROSmsg(
                                                          cv_colorMap, sensor_msgs::image_encodings::TYPE_8UC4,
                                                          mMapFrameId, sl_tools::slTime2Ros(t)));
                    }
                }

                // Traversability Map Image
                if (travSub > 0 /*|| gridSub > 0*/) {
                    terrain.generateTerrainMap(sl_traversMap, sl::MAT_TYPE_16U_C1, sl::LayerName::TRAVERSABILITY_COST);

                    if (sl_traversMap.getResolution().area() > 0) {
                        cv_traversMap = sl_tools::toCVMat(sl_traversMap);
                        mPubGlobalTravMapImg.publish(imageToROSmsg(
                                                         cv_traversMap, sensor_msgs::image_encodings::TYPE_16UC1,
                                                         mMapFrameId, sl_tools::slTime2Ros(t)));
                    }
                }

                // Multilayer GridMap {https://github.com/ethz-asl/grid_map}
                //                    if (gridSub > 0) {

                //                        if (!cv_traversMap.empty() &&
                //                            !cv_colorMap.empty() &&
                //                            !cv_heightMap.empty())

                //                        {
                //                            // TODO USE CHUNK TO UPDATE SUBMAPS
                //                            // GridMap creation
                //                            grid_map::GridMap gridMap;
                //                            grid_map::Position pos(0, 0); // TODO: initialize with the initial position of the tracking
                //                            grid_map::GridMapCvConverter::initializeFromImage(cv_heightMap, mTerrainMapRes, gridMap, pos);
                //                            grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(cv_heightMap, "height_map", gridMap);
                //                            grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(cv_traversMap, "traversability_map", gridMap);
                //                            grid_map::GridMapCvConverter::addColorLayerFromImage<unsigned char, 4>(cv_colorMap, "color_map", gridMap);
                //                            // GridMap to ROS message
                //                            grid_map_msgs::GridMap gridMapMsg;
                //                            gridMap.setTimestamp(t);
                //                            gridMap.setFrameId(mMapFrameId);
                //                            grid_map::GridMapRosConverter::toMessage(gridMap, gridMapMsg);
                //                            // Publishing
                //                            mPubGridMap.publish(gridMapMsg);

                //                            /*sl::Mesh mesh;
                //                            terrain.toMesh(mesh);
                //                            mesh.save("./mesh.obj");*/
                //                        }
                //                    }
            } else {
                mTerrainMutex.unlock();
                NODELET_DEBUG_STREAM("Local terrain not available");
            }
        } else {
            mTerrainMutex.unlock();
        }
    }

    void ZEDWrapperNodelet::initMapMsgs(double map_size_m, bool initCvMat) {
        mMapsValid = false;

        // MetaData
        nav_msgs::MapMetaData mapInfo;
        mapInfo.resolution = mTerrainMapRes;

        uint32_t mapCellSize = static_cast<uint32_t>(map_size_m / mTerrainMapRes);

        NODELET_DEBUG_STREAM("Initializing maps of size " << map_size_m << "x" << map_size_m << " cells");

        mapInfo.height = mapCellSize;
        mapInfo.width = mapCellSize;
        mapInfo.origin.position.x = -(mapInfo.width / 2) * mapInfo.resolution; // TODO this is valid only if the map is centered in (0,0)
        mapInfo.origin.position.y = -(mapInfo.height / 2) * mapInfo.resolution;
        mapInfo.origin.position.z = 0.0;
        mapInfo.origin.orientation.x = 0.0;
        mapInfo.origin.orientation.y = 0.0;
        mapInfo.origin.orientation.z = 0.0;
        mapInfo.origin.orientation.w = 1.0;

        // Maps
        mHeightMapMsg.header.frame_id = mMapFrameId;
        mCostMapMsg.header.frame_id = mMapFrameId;
        mHeightMapMsg.info = mapInfo;
        mCostMapMsg.info = mapInfo;

        // CV Maps
        if (initCvMat) {
            mCvHeightMat = cv::Mat(mapCellSize, mapCellSize, CV_8SC1, cv::Scalar::all(-1));
            mCvCostMat = cv::Mat(mapCellSize, mapCellSize, CV_8SC1, cv::Scalar::all(-1));
        }
    }

    void ZEDWrapperNodelet::doubleMapsDims() {
        NODELET_DEBUG("Map resize required");
        uint32_t currMapCellSize = static_cast<uint32_t>(mHeightMapMsg.info.height * mTerrainMapRes);
        uint32_t newSize = currMapCellSize * 2;


        // Reinitialize OpenCV mats
        cv::Rect roi;
        roi.width = mHeightMapMsg.info.width;
        roi.height = mHeightMapMsg.info.height;
        roi.x = (newSize - roi.width) / 2;
        roi.y = (newSize - roi.height) / 2;

        // Copy old "small" Maps data to the new "big" Maps putting old data in the middle
        cv::Mat tmpHeight = mCvHeightMat.clone();
        cv::Mat tmpCost = mCvCostMat.clone();
        mCvHeightMat = cv::Mat(newSize, newSize, CV_8SC1, cv::Scalar::all(-1));
        mCvCostMat = cv::Mat(newSize, newSize, CV_8SC1, cv::Scalar::all(-1));
        cv::Mat subHeight = mCvHeightMat(roi);
        tmpHeight.copyTo(subHeight);
        cv::Mat subCost = mCvCostMat(roi);
        tmpCost.copyTo(subCost);

        // Reinitialize map messages to new size (no CV Mats)
        initMapMsgs(newSize, false);
    }

    bool ZEDWrapperNodelet::coord2cell(float xm, float ym, uint32_t& row, uint32_t& col) {
        row = static_cast<int>(std::floor(ym));

        // TODO Complete the function `coord2cell`
    }

    void ZEDWrapperNodelet::chunk2maps(sl::TerrainChunk& chunk) {
        //        sl::Dimension chunkDim = chunk.getDimension();
        //        // use "mMapMaxHeight" to normalize |value| in range [0,100]

        //        double mapWm = mHeightMapMsg.info.width * mHeightMapMsg.info.resolution;
        //        double mapMinX = mHeightMapMsg.info.origin.position.x;
        //        double mapMaxX = mapMinX + mapWm;

        //        double mapHm = mHeightMapMsg.info.height * mHeightMapMsg.info.resolution;
        //        double mapMinY = mHeightMapMsg.info.origin.position.y;
        //        double mapMaxY = mapMinY + mapHm;

        //        if (chunkDim.getXmin() < mapMinX ||
        //            chunkDim.getXmax() > mapMaxX ||
        //            chunkDim.getYmin() < mapMinY ||
        //            chunkDim.getYmax() > mapMaxY) {
        //            doubleMapsDims(); // Create new double sized map
        //        }

        //        unsigned int cellCount = chunkDim.getFullSizeIdx();

        //        #pragma omp for
        //        for (unsigned int i = 0; i < cellCount; i++) {
        //            float xm, ym;
        //            chunkDim.index2x_y(i, xm, ym);


        //        }
    }

    void ZEDWrapperNodelet::imuPubCallback(const ros::TimerEvent& e) {
        uint32_t imu_SubNumber = mPubImu.getNumSubscribers();
        uint32_t imu_RawSubNumber = mPubImuRaw.getNumSubscribers();

        if (imu_SubNumber < 1 && imu_RawSubNumber < 1) {
            return;
        }

        ros::Time t = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_IMAGE));
        sl::IMUData imu_data;
        mZed.getIMUData(imu_data, sl::TIME_REFERENCE_CURRENT);

        if (imu_SubNumber > 0) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = mImuTime; // t;
            imu_msg.header.frame_id = mImuFrameId;
            imu_msg.orientation.x = mSignX * imu_data.getOrientation()[mIdxX];
            imu_msg.orientation.y = mSignY * imu_data.getOrientation()[mIdxY];
            imu_msg.orientation.z = mSignZ * imu_data.getOrientation()[mIdxZ];
            imu_msg.orientation.w = imu_data.getOrientation()[3];
            imu_msg.angular_velocity.x = mSignX * imu_data.angular_velocity[mIdxX];
            imu_msg.angular_velocity.y = mSignY * imu_data.angular_velocity[mIdxY];
            imu_msg.angular_velocity.z = mSignZ * imu_data.angular_velocity[mIdxZ];
            imu_msg.linear_acceleration.x = mSignX * imu_data.linear_acceleration[mIdxX];
            imu_msg.linear_acceleration.y = mSignY * imu_data.linear_acceleration[mIdxY];
            imu_msg.linear_acceleration.z = mSignZ * imu_data.linear_acceleration[mIdxZ];

            for (int i = 0; i < 3; i += 3) {
                imu_msg.orientation_covariance[i * 3 + 0] =
                    imu_data.orientation_covariance.r[i * 3 + mIdxX];
                imu_msg.orientation_covariance[i * 3 + 1] =
                    imu_data.orientation_covariance.r[i * 3 + mIdxY];
                imu_msg.orientation_covariance[i * 3 + 2] =
                    imu_data.orientation_covariance.r[i * 3 + mIdxZ];
                imu_msg.linear_acceleration_covariance[i * 3 + 0] =
                    imu_data.linear_acceleration_convariance.r[i * 3 + mIdxX];
                imu_msg.linear_acceleration_covariance[i * 3 + 1] =
                    imu_data.linear_acceleration_convariance.r[i * 3 + mIdxY];
                imu_msg.linear_acceleration_covariance[i * 3 + 2] =
                    imu_data.linear_acceleration_convariance.r[i * 3 + mIdxZ];
                imu_msg.angular_velocity_covariance[i * 3 + 0] =
                    imu_data.angular_velocity_convariance.r[i * 3 + mIdxX];
                imu_msg.angular_velocity_covariance[i * 3 + 1] =
                    imu_data.angular_velocity_convariance.r[i * 3 + mIdxY];
                imu_msg.angular_velocity_covariance[i * 3 + 2] =
                    imu_data.angular_velocity_convariance.r[i * 3 + mIdxZ];
            }

            mPubImu.publish(imu_msg);
        }

        if (imu_RawSubNumber > 0) {
            sensor_msgs::Imu imu_raw_msg;
            imu_raw_msg.header.stamp = mImuTime; // t;
            imu_raw_msg.header.frame_id = mImuFrameId;
            imu_raw_msg.angular_velocity.x = mSignX * imu_data.angular_velocity[mIdxX];
            imu_raw_msg.angular_velocity.y = mSignY * imu_data.angular_velocity[mIdxY];
            imu_raw_msg.angular_velocity.z = mSignZ * imu_data.angular_velocity[mIdxZ];
            imu_raw_msg.linear_acceleration.x =
                mSignX * imu_data.linear_acceleration[mIdxX];
            imu_raw_msg.linear_acceleration.y =
                mSignY * imu_data.linear_acceleration[mIdxY];
            imu_raw_msg.linear_acceleration.z =
                mSignZ * imu_data.linear_acceleration[mIdxZ];

            for (int i = 0; i < 3; i += 3) {
                imu_raw_msg.linear_acceleration_covariance[i * 3 + 0] =
                    imu_data.linear_acceleration_convariance.r[i * 3 + mIdxX];
                imu_raw_msg.linear_acceleration_covariance[i * 3 + 1] =
                    imu_data.linear_acceleration_convariance.r[i * 3 + mIdxY];
                imu_raw_msg.linear_acceleration_covariance[i * 3 + 2] =
                    imu_data.linear_acceleration_convariance.r[i * 3 + mIdxZ];
                imu_raw_msg.angular_velocity_covariance[i * 3 + 0] =
                    imu_data.angular_velocity_convariance.r[i * 3 + mIdxX];
                imu_raw_msg.angular_velocity_covariance[i * 3 + 1] =
                    imu_data.angular_velocity_convariance.r[i * 3 + mIdxY];
                imu_raw_msg.angular_velocity_covariance[i * 3 + 2] =
                    imu_data.angular_velocity_convariance.r[i * 3 + mIdxZ];
            }

            imu_raw_msg.orientation_covariance[0] =
                -1; // Orientation data is not available in "data_raw" -> See ROS REP145
            // http://www.ros.org/reps/rep-0145.html#topics
            mPubImuRaw.publish(imu_raw_msg);
        }

        // Publish IMU tf only if enabled
        if (mPublishTf) {
            // Camera to map transform from TF buffer
            tf2::Transform base_to_map;

            // Look up the transformation from base frame to map link
            try {
                // Save the transformation from base to frame
                geometry_msgs::TransformStamped b2m =
                    mTfBuffer->lookupTransform(mMapFrameId, mBaseFrameId, ros::Time(0));
                // Get the TF2 transformation
                tf2::fromMsg(b2m.transform, base_to_map);
            } catch (tf2::TransformException& ex) {
                NODELET_WARN_THROTTLE(
                    10.0, "The tf from '%s' to '%s' does not seem to be available. "
                    "IMU TF not published!",
                    mBaseFrameId.c_str(), mMapFrameId.c_str());
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
            tf2::Quaternion map_q = base_to_map.getRotation();
            // Difference between IMU and ZED Quaternion
            tf2::Quaternion delta_q = imu_q * map_q.inverse();
            tf2::Transform imu_pose;
            imu_pose.setIdentity();
            imu_pose.setRotation(delta_q);
            // Note, the frame is published, but its values will only change if someone
            // has subscribed to IMU
            publishImuFrame(imu_pose); // publish the imu Frame
        }
    }

    void ZEDWrapperNodelet::device_poll() {
        ros::Rate loop_rate(mCamFrameRate);
        ros::Time old_t =
            sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));
        mImuTime = old_t;
        sl::ERROR_CODE grab_status;
        mTrackingActivated = false;
        // Get the parameters of the ZED images
        mCamWidth = mZed.getResolution().width;
        mCamHeight = mZed.getResolution().height;
        NODELET_DEBUG_STREAM("Camera Frame size : " << mCamWidth << "x" << mCamHeight);
        mMatWidth = static_cast<int>(mCamWidth * mCamMatResizeFactor);
        mMatHeight = static_cast<int>(mCamHeight * mCamMatResizeFactor);
        NODELET_DEBUG_STREAM("Data Mat size : " << mMatWidth << "x" << mMatHeight);
        cv::Size cvSize(mMatWidth, mMatWidth);
        mCvLeftImRGB = cv::Mat(cvSize, CV_8UC3);
        mCvRightImRGB = cv::Mat(cvSize, CV_8UC3);
        mCvConfImRGB = cv::Mat(cvSize, CV_8UC3);
        mCvConfMapFloat = cv::Mat(cvSize, CV_32FC1);
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
        sl::Mat leftZEDMat, rightZEDMat, depthZEDMat, disparityZEDMat, confImgZEDMat,
        confMapZEDMat;

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
            uint32_t odomSubnumber = mPubOdom.getNumSubscribers();
            uint32_t confImgSubnumber = mPubConfImg.getNumSubscribers();
            uint32_t confMapSubnumber = mPubConfMap.getNumSubscribers();
            uint32_t imuSubnumber = mPubImu.getNumSubscribers();
            uint32_t imuRawsubnumber = mPubImuRaw.getNumSubscribers();
            bool runLoop = mTerrainMap ||
                           ((rgbSubnumber + rgbRawSubnumber + leftSubnumber +
                             leftRawSubnumber + rightSubnumber + rightRawSubnumber +
                             depthSubnumber + disparitySubnumber + cloudSubnumber +
                             poseSubnumber + odomSubnumber + confImgSubnumber +
                             confMapSubnumber + imuSubnumber + imuRawsubnumber) > 0);
            runParams.enable_point_cloud = false;

            if (cloudSubnumber > 0) {
                runParams.enable_point_cloud = true;
            }

            // Run the loop only if there is some subscribers
            if (runLoop) {
                if ((mTerrainMap || mDepthStabilization || poseSubnumber > 0 || odomSubnumber > 0 ||
                     cloudSubnumber > 0 || depthSubnumber > 0) &&
                    !mTrackingActivated) { // Start the tracking
                    start_tracking();
                } else if (!mTerrainMap && !mDepthStabilization && poseSubnumber == 0 &&
                           odomSubnumber == 0 &&
                           mTrackingActivated) { // Stop the tracking
                    mZed.disableTracking();
                    mTrackingActivated = false;
                }

                // Detect if one of the subscriber need to have the depth information
                mComputeDepth = mTerrainMap ||
                                ((depthSubnumber + disparitySubnumber + cloudSubnumber +
                                  poseSubnumber + odomSubnumber + confImgSubnumber +
                                  confMapSubnumber) > 0);
                // Timestamp
                ros::Time t =
                    sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_IMAGE));
                mGrabbing = true;

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

                grab_status = mZed.grab(runParams); // Ask to not compute the depth
                mGrabbing = false;

                // cout << toString(grab_status) << endl;
                if (grab_status != sl::ERROR_CODE::SUCCESS) {
                    // Detect if a error occurred (for example:
                    // the zed have been disconnected) and
                    // re-initialize the ZED
                    if (grab_status != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                        NODELET_INFO_STREAM_ONCE(toString(grab_status));
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(2));

                    if ((t - old_t).toSec() > 5) {
                        mZed.close();
                        NODELET_INFO("Re-opening the ZED");
                        sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;

                        while (err != sl::SUCCESS) {
                            // Ctrl+C check
                            if (!mNhNs.ok()) {
                                mZed.close();
                                return;
                            }

                            int id = sl_tools::checkCameraReady(mZedSerialNumber);

                            if (id > 0) {
                                mZedParams.camera_linux_id = id;
                                err = mZed.open(mZedParams); // Try to initialize the ZED
                                NODELET_INFO_STREAM(toString(err));
                            } else {
                                NODELET_INFO_STREAM("Waiting for the ZED (S/N " << mZedSerialNumber << ") to be re-connected");
                            }

                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        }

                        mTrackingActivated = false;

                        if (mTerrainMap || mDepthStabilization ||
                            poseSubnumber > 0 || odomSubnumber > 0) { // Start the tracking
                            start_tracking();
                        }
                    }

                    continue;
                }

                // Time updatestatic_cast<double>(
                old_t = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));

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
                    mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT, sl::MEM_CPU, mMatWidth,
                                       mMatHeight);
                    cv::cvtColor(sl_tools::toCVMat(leftZEDMat), mCvLeftImRGB, CV_RGBA2RGB);

                    if (leftSubnumber > 0) {
                        publishCamInfo(mLeftCamInfoMsg, mPubLeftCamInfo, t);
                        publishImage(mCvLeftImRGB, mPubLeft, mLeftCamOptFrameId, t);
                    }

                    if (rgbSubnumber > 0) {
                        publishCamInfo(mRgbCamInfoMsg, mPubRgbCamInfo, t);
                        publishImage(mCvLeftImRGB, mPubRgb, mDepthOptFrameId,
                                     t); // rgb is the left image
                    }
                }

                // Publish the left_raw == rgb_raw image if someone has subscribed to
                if (leftRawSubnumber > 0 || rgbRawSubnumber > 0) {
                    // Retrieve RGBA Left image
                    mZed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED, sl::MEM_CPU,
                                       mMatWidth, mMatHeight);
                    cv::cvtColor(sl_tools::toCVMat(leftZEDMat), mCvLeftImRGB, CV_RGBA2RGB);

                    if (leftRawSubnumber > 0) {
                        publishCamInfo(mLeftCamInfoRawMsg, mPubLeftCamInfoRaw, t);
                        publishImage(mCvLeftImRGB, mPubRawLeft, mLeftCamOptFrameId, t);
                    }

                    if (rgbRawSubnumber > 0) {
                        publishCamInfo(mRgbCamInfoRawMsg, mPubRgbCamInfoRaw, t);
                        publishImage(mCvLeftImRGB, mPubRawRgb, mDepthOptFrameId, t);
                    }
                }

                // Publish the right image if someone has subscribed to
                if (rightSubnumber > 0) {
                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT, sl::MEM_CPU,
                                       mMatWidth, mMatHeight);
                    cv::cvtColor(sl_tools::toCVMat(rightZEDMat), mCvRightImRGB, CV_RGBA2RGB);
                    publishCamInfo(mRightCamInfoMsg, mPubRightCamInfo, t);
                    publishImage(mCvRightImRGB, mPubRight, mRightCamOptFrameId, t);
                }

                // Publish the right image if someone has subscribed to
                if (rightRawSubnumber > 0) {
                    // Retrieve RGBA Right image
                    mZed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED, sl::MEM_CPU,
                                       mMatWidth, mMatHeight);
                    cv::cvtColor(sl_tools::toCVMat(rightZEDMat), mCvRightImRGB, CV_RGBA2RGB);
                    publishCamInfo(mRightCamInfoRawMsg, mPubRightCamInfoRaw, t);
                    publishImage(mCvRightImRGB, mPubRawRight, mRightCamOptFrameId, t);
                }

                // Publish the depth image if someone has subscribed to
                if (depthSubnumber > 0 || disparitySubnumber > 0) {
                    mZed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH, sl::MEM_CPU,
                                         mMatWidth, mMatHeight);
                    publishCamInfo(mDepthCamInfoMsg, mPubDepthCamInfo, t);
                    publishDepth(sl_tools::toCVMat(depthZEDMat), t); // in meters
                }

                // Publish the disparity image if someone has subscribed to
                if (disparitySubnumber > 0) {
                    mZed.retrieveMeasure(disparityZEDMat, sl::MEASURE_DISPARITY, sl::MEM_CPU,
                                         mMatWidth, mMatHeight);
                    // Need to flip sign, but cause of this is not sure
                    cv::Mat disparity = sl_tools::toCVMat(disparityZEDMat) * -1.0;
                    publishDisparity(disparity, t);
                }

                // Publish the confidence image if someone has subscribed to
                if (confImgSubnumber > 0) {
                    mZed.retrieveImage(confImgZEDMat, sl::VIEW_CONFIDENCE, sl::MEM_CPU,
                                       mMatWidth, mMatHeight);
                    cv::cvtColor(sl_tools::toCVMat(confImgZEDMat), mCvConfImRGB, CV_RGBA2RGB);
                    publishImage(mCvConfImRGB, mPubConfImg, mConfidenceOptFrameId, t);
                }

                // Publish the confidence map if someone has subscribed to
                if (confMapSubnumber > 0) {
                    mZed.retrieveMeasure(confMapZEDMat, sl::MEASURE_CONFIDENCE, sl::MEM_CPU,
                                         mMatWidth, mMatHeight);
                    mCvConfMapFloat = sl_tools::toCVMat(confMapZEDMat);
                    mPubConfMap.publish(imageToROSmsg(
                                            mCvConfMapFloat, sensor_msgs::image_encodings::TYPE_32FC1,
                                            mConfidenceOptFrameId, t));
                }

                // Publish the point cloud if someone has subscribed to
                if (cloudSubnumber > 0) {
                    // Run the point cloud conversion asynchronously to avoid slowing down
                    // all the program
                    // Retrieve raw pointCloud data
                    mZed.retrieveMeasure(mCloud, sl::MEASURE_XYZBGRA, sl::MEM_CPU, mMatWidth,
                                         mMatHeight);
                    mPointCloudFrameId = mDepthFrameId;
                    mPointCloudTime = t;
                    publishPointCloud(mMatWidth, mMatHeight);
                }

                mCamDataMutex.unlock();
                // Transform from base to sensor
                tf2::Transform sensor_to_base_transf;

                // Look up the transformation from base frame to camera link
                try {
                    // Save the transformation from base to frame
                    geometry_msgs::TransformStamped s2b =
                        mTfBuffer->lookupTransform(mBaseFrameId, mDepthFrameId, t);
                    // Get the TF2 transformation
                    tf2::fromMsg(s2b.transform, sensor_to_base_transf);
                } catch (tf2::TransformException& ex) {
                    NODELET_WARN_THROTTLE(
                        10.0, "The tf from '%s' to '%s' does not seem to be available, "
                        "will assume it as identity!",
                        mDepthFrameId.c_str(), mBaseFrameId.c_str());
                    NODELET_DEBUG("Transform error: %s", ex.what());
                    sensor_to_base_transf.setIdentity();
                }

                // Publish the odometry if someone has subscribed to
                if (mTerrainMap || poseSubnumber > 0 || odomSubnumber > 0 || cloudSubnumber > 0 ||
                    depthSubnumber > 0 || imuSubnumber > 0 || imuRawsubnumber > 0) {
                    if (!mInitOdomWithPose) {
                        sl::Pose deltaOdom;
                        sl::TRACKING_STATE status = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME_CAMERA);

                        if (status == sl::TRACKING_STATE_OK || status == sl::TRACKING_STATE_SEARCHING || status == sl::TRACKING_STATE_FPS_TOO_LOW) {
                            // Transform ZED delta odom pose in TF2 Transformation
                            geometry_msgs::Transform deltaTransf;
                            sl::Translation translation = deltaOdom.getTranslation();
                            sl::Orientation quat = deltaOdom.getOrientation();
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
                            tf2::Transform deltaOdomTf_base = sensor_to_base_transf *
                                                              deltaOdomTf *
                                                              sensor_to_base_transf.inverse();
                            // Propagate Odom transform in time
                            mBase2OdomTransf = mBase2OdomTransf * deltaOdomTf_base;
                            // Publish odometry message
                            publishOdom(mBase2OdomTransf, t);
                            mTrackingReady = true;
                        } else {
                            NODELET_DEBUG_STREAM("ODOM -> Tracking Status: " << static_cast<int>(status));
                        }
                    }
                }

                // Publish the zed camera pose if someone has subscribed to
                if (mTerrainMap || poseSubnumber > 0 || odomSubnumber > 0 || cloudSubnumber > 0 ||
                    depthSubnumber > 0 || imuSubnumber > 0 || imuRawsubnumber > 0) {
                    sl::Pose zed_pose; // Sensor to Map transform
                    sl::TRACKING_STATE status = mZed.getPosition(zed_pose, sl::REFERENCE_FRAME_WORLD);

                    if (status == sl::TRACKING_STATE_OK || status == sl::TRACKING_STATE_SEARCHING) {
                        // Transform ZED pose in TF2 Transformation
                        geometry_msgs::Transform sens2mapTransf;
                        sl::Translation translation = zed_pose.getTranslation();
                        sl::Orientation quat = zed_pose.getOrientation();
                        sens2mapTransf.translation.x = mSignX * translation(mIdxX);
                        sens2mapTransf.translation.y = mSignY * translation(mIdxY);
                        sens2mapTransf.translation.z = mSignZ * translation(mIdxZ);
                        sens2mapTransf.rotation.x = mSignX * quat(mIdxX);
                        sens2mapTransf.rotation.y = mSignY * quat(mIdxY);
                        sens2mapTransf.rotation.z = mSignZ * quat(mIdxZ);
                        sens2mapTransf.rotation.w = quat(3);
                        tf2::Transform sens_to_map_transf;
                        tf2::fromMsg(sens2mapTransf, sens_to_map_transf);
                        // Transformation from camera sensor to base frame
                        tf2::Transform base_to_map_transform = sensor_to_base_transf *
                                                               sens_to_map_transf *
                                                               sensor_to_base_transf.inverse();

                        if (mInitOdomWithPose) {
                            // Propagate Odom transform in time
                            mBase2OdomTransf = base_to_map_transform;
                            base_to_map_transform.setIdentity();

                            if (odomSubnumber > 0) {
                                // Publish odometry message
                                publishOdom(mBase2OdomTransf, t);
                            }

                            mInitOdomWithPose = false;
                        } else {
                            // Transformation from map to odometry frame
                            mOdom2MapTransf =
                                base_to_map_transform * mBase2OdomTransf.inverse();
                        }

                        // Publish Pose message
                        publishPose(mOdom2MapTransf, t);
                        mTrackingReady = true;
                    } else {
                        NODELET_DEBUG_STREAM("MAP -> Tracking Status: " << static_cast<int>(status));
                    }
                }

                // Publish pose tf only if enabled
                if (mPublishTf) {
                    // Note, the frame is published, but its values will only change if
                    // someone has subscribed to odom
                    publishOdomFrame(mBase2OdomTransf,
                                     t); // publish the base Frame in odometry frame
                    // Note, the frame is published, but its values will only change if
                    // someone has subscribed to map
                    publishPoseFrame(mOdom2MapTransf,
                                     t); // publish the odometry Frame in map frame
                    mImuTime = t;
                }

                static int rateWarnCount = 0;

                if (!loop_rate.sleep()) {
                    rateWarnCount++;

                    if (rateWarnCount == 10) {
                        NODELET_DEBUG_THROTTLE(
                            1.0,
                            "Working thread is not synchronized with the Camera frame rate");
                        NODELET_DEBUG_STREAM_THROTTLE(
                            1.0, "Expected cycle time: " << loop_rate.expectedCycleTime()
                            << " - Real cycle time: "
                            << loop_rate.cycleTime());
                        NODELET_WARN_THROTTLE(10.0, "Elaboration takes longer than requested "
                                              "by the FPS rate. Please consider to "
                                              "lower the 'frame_rate' setting.");
                    }
                } else {
                    rateWarnCount = 0;
                }
            } else {
                NODELET_DEBUG_THROTTLE(5.0, "No topics subscribed by users");

                // Publish odometry tf only if enabled
                if (mPublishTf) {
                    ros::Time t =
                        sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE_CURRENT));
                    publishOdomFrame(mBase2OdomTransf,
                                     t); // publish the base Frame in odometry frame
                    publishPoseFrame(mOdom2MapTransf,
                                     t); // publish the odometry Frame in map frame
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(10)); // No subscribers, we just wait
                loop_rate.reset();
            }
        } // while loop

        mZed.close();
    }

} // namespace
