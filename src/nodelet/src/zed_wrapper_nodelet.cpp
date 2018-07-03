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

#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
#include <thread>
#include <chrono>
#include <memory>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// >>>>> Backward compatibility
#define COORDINATE_SYSTEM_IMAGE                     static_cast<sl::COORDINATE_SYSTEM>(0)
#define COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP         static_cast<sl::COORDINATE_SYSTEM>(3)
#define COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD	static_cast<sl::COORDINATE_SYSTEM>(5)
// <<<<< Backward compatibility

using namespace std;

namespace zed_wrapper {

ZEDWrapperNodelet::ZEDWrapperNodelet()
    : Nodelet() {
}

ZEDWrapperNodelet::~ZEDWrapperNodelet() {
    device_poll_thread.get()->join();
}

void ZEDWrapperNodelet::onInit() {
    // Launch file parameters
    resolution = sl::RESOLUTION_HD720;
    quality = sl::DEPTH_MODE_PERFORMANCE;
    sensing_mode = sl::SENSING_MODE_STANDARD;
    rate = 30;
    gpu_id = -1;
    zed_id = 0;
    serial_number = 0;
    odometry_DB = "";
    imu_pub_rate = 100.0;

    nh = getMTNodeHandle();
    nh_ns = getMTPrivateNodeHandle();

    // Set  default coordinate frames
    // If unknown left and right frames are set in the same camera coordinate frame
    nh_ns.param<std::string>("odometry_frame", odometry_frame_id, "odometry_frame");
    nh_ns.param<std::string>("base_frame", base_frame_id, "base_frame");
    nh_ns.param<std::string>("imu_frame", imu_frame_id, "imu_link");

    nh_ns.param<std::string>("left_camera_frame", left_cam_frame_id, "left_camera_optical_frame");
    nh_ns.param<std::string>("left_camera_optical_frame", left_cam_opt_frame_id, "left_camera_optical_frame");

    nh_ns.param<std::string>("right_camera_frame", right_cam_frame_id, "right_camera_frame");
    nh_ns.param<std::string>("right_camera_optical_frame", right_cam_opt_frame_id, "right_camera_optical_frame");

    depth_frame_id = left_cam_frame_id;
    depth_opt_frame_id = left_cam_opt_frame_id;

    // Note: Depth image frame id must match color image frame id
    cloud_frame_id = depth_opt_frame_id;
    rgb_frame_id = depth_frame_id;
    rgb_opt_frame_id  = cloud_frame_id;

    disparity_frame_id = depth_frame_id;
    disparity_opt_frame_id = depth_opt_frame_id;

    confidence_frame_id = depth_frame_id;
    confidence_opt_frame_id = depth_opt_frame_id;

    // Get parameters from launch file
    nh_ns.getParam("resolution", resolution);
    nh_ns.getParam("quality", quality);
    nh_ns.getParam("sensing_mode", sensing_mode);
    nh_ns.getParam("frame_rate", rate);
    nh_ns.getParam("openni_depth_mode", openniDepthMode);
    nh_ns.getParam("gpu_id", gpu_id);
    nh_ns.getParam("zed_id", zed_id);
    nh_ns.getParam("depth_stabilization", depth_stabilization);
    int tmp_sn = 0;
    nh_ns.getParam("serial_number", tmp_sn);
    if (tmp_sn > 0) serial_number = tmp_sn;
    nh_ns.getParam("camera_model", user_cam_model);

    // Publish odometry tf
    nh_ns.param<bool>("publish_tf", publish_tf, true);

    if (serial_number > 0)
        NODELET_INFO_STREAM("SN : " << serial_number);

    // Print order frames
    NODELET_INFO_STREAM("odometry_frame \t\t   -> " << odometry_frame_id);
    NODELET_INFO_STREAM("base_frame \t\t   -> " << base_frame_id);
    NODELET_INFO_STREAM("imu_link \t\t   -> " << imu_frame_id);
    NODELET_INFO_STREAM("left_camera_frame \t   -> " << left_cam_frame_id);
    NODELET_INFO_STREAM("left_camera_optical_frame  -> " << left_cam_opt_frame_id);
    NODELET_INFO_STREAM("right_camera_frame \t   -> " << right_cam_frame_id);
    NODELET_INFO_STREAM("right_camera_optical_frame -> " << right_cam_opt_frame_id);
    NODELET_INFO_STREAM("depth_frame \t\t   -> " << depth_frame_id);
    NODELET_INFO_STREAM("depth_optical_frame \t   -> " << depth_opt_frame_id);
    NODELET_INFO_STREAM("disparity_frame \t   -> " << disparity_frame_id);
    NODELET_INFO_STREAM("disparity_optical_frame    -> " << disparity_opt_frame_id);

    // Status of odometry TF
    NODELET_INFO_STREAM("Publish " << odometry_frame_id << " [" << (publish_tf ? "TRUE" : "FALSE") << "]");

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
    if (openniDepthMode) {
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

    string odometry_topic = "odom";
    string imu_topic = "imu";

    nh_ns.getParam("rgb_topic", rgb_topic);
    nh_ns.getParam("rgb_raw_topic", rgb_raw_topic);
    nh_ns.getParam("rgb_cam_info_topic", rgb_cam_info_topic);
    nh_ns.getParam("rgb_cam_info_raw_topic", rgb_cam_info_raw_topic);

    nh_ns.getParam("left_topic", left_topic);
    nh_ns.getParam("left_raw_topic", left_raw_topic);
    nh_ns.getParam("left_cam_info_topic", left_cam_info_topic);
    nh_ns.getParam("left_cam_info_raw_topic", left_cam_info_raw_topic);

    nh_ns.getParam("right_topic", right_topic);
    nh_ns.getParam("right_raw_topic", right_raw_topic);
    nh_ns.getParam("right_cam_info_topic", right_cam_info_topic);
    nh_ns.getParam("right_cam_info_raw_topic", right_cam_info_raw_topic);

    nh_ns.getParam("depth_topic", depth_topic);
    nh_ns.getParam("depth_cam_info_topic", depth_cam_info_topic);

    nh_ns.getParam("disparity_topic", disparity_topic);

    nh_ns.getParam("confidence_img_topic", conf_img_topic);
    nh_ns.getParam("confidence_map_topic", conf_map_topic);

    nh_ns.getParam("point_cloud_topic", point_cloud_topic);

    nh_ns.getParam("odometry_topic", odometry_topic);

    nh_ns.getParam("imu_topic", imu_topic);
    nh_ns.getParam("imu_pub_rate", imu_pub_rate);

    nh_ns.param<std::string>("svo_filepath", svo_filepath, std::string());

    // Initialization transformation listener
    tfBuffer.reset(new tf2_ros::Buffer);
    tf_listener.reset(new tf2_ros::TransformListener(*tfBuffer));

    // Initialize tf2 transformation
    base_transform.setIdentity();

    // Try to initialize the ZED
    if (!svo_filepath.empty())
        param.svo_input_filename = svo_filepath.c_str();
    else {
        param.camera_fps = rate;
        param.camera_resolution = static_cast<sl::RESOLUTION> (resolution);
        if (serial_number == 0)
            param.camera_linux_id = zed_id;
        else {
            bool waiting_for_camera = true;
            while (waiting_for_camera) {
                sl::DeviceProperties prop = sl_tools::getZEDFromSN(serial_number);
                if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                    std::string msg = "ZED SN" + to_string(serial_number) + " not detected ! Please connect this ZED";
                    NODELET_INFO_STREAM(msg.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                } else {
                    waiting_for_camera = false;
                    param.camera_linux_id = prop.id;
                }
            }
        }
    }

    std::string ver = sl_tools::getSDKVersion( ver_major, ver_minor, ver_sub_minor);
    NODELET_INFO_STREAM( "SDK version : " << ver );

    if( ver_major<2 ) {
        NODELET_WARN_STREAM( "Please consider to upgrade to latest SDK version to get better performances");
        param.coordinate_system = COORDINATE_SYSTEM_IMAGE;
        NODELET_INFO_STREAM("Camera coordinate system set to COORDINATE_SYSTEM_IMAGE");
    } else if( ver_major==2 && ver_minor<5) {
        NODELET_WARN_STREAM( "Please consider to upgrade to latest SDK version to get latest features");
        param.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;
        NODELET_INFO_STREAM("Camera coordinate system set to COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP");
    } else {
        param.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;
        NODELET_INFO_STREAM("Camera coordinate system set to COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD");
    }

    param.coordinate_units = sl::UNIT_METER;

    param.depth_mode = static_cast<sl::DEPTH_MODE> (quality);
    param.sdk_verbose = true;
    param.sdk_gpu_id = gpu_id;
    param.depth_stabilization = depth_stabilization;

    sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
    while (err != sl::SUCCESS) {
        err = zed.open(param);
        NODELET_INFO_STREAM(toString(err));
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        if( !nh_ns.ok() )
        {
            zed.close();
            return;
        }
    }

    sl::MODEL realCamModel = zed.getCameraInformation().camera_model;

    std::string camModelStr = "LAST";
    if( realCamModel == sl::MODEL_ZED  ){
        camModelStr = "ZED";
        if( user_cam_model != 0){
            NODELET_WARN("Camera model does not match user parameter. Please modify the value of the parameter 'camera_model' to 0");
        }
    } else if( realCamModel == sl::MODEL_ZED_M  ){
        camModelStr = "ZED M";
        if( user_cam_model != 1){
            NODELET_WARN("Camera model does not match user parameter. Please modify the value of the parameter 'camera_model' to 1");
        }
    }

    NODELET_INFO_STREAM("CAMERA MODEL : " << realCamModel);

    serial_number = zed.getCameraInformation().serial_number;

    // Dynamic Reconfigure parameters
    server = boost::make_shared<dynamic_reconfigure::Server < zed_wrapper::ZedConfig >> ();
    dynamic_reconfigure::Server<zed_wrapper::ZedConfig>::CallbackType f;
    f = boost::bind(&ZEDWrapperNodelet::dynamicReconfCallback, this, _1, _2);
    server->setCallback(f);

    nh_ns.getParam("confidence", confidence);
    nh_ns.getParam("exposure", exposure);
    nh_ns.getParam("gain", gain);
    nh_ns.getParam("auto_exposure", autoExposure);
    if (autoExposure)
        triggerAutoExposure = true;

    // Create all the publishers
    // Image publishers
    image_transport::ImageTransport it_zed(nh);
    pub_rgb = it_zed.advertise(rgb_topic, 1); //rgb
    NODELET_INFO_STREAM("Advertized on topic " << rgb_topic);
    pub_raw_rgb = it_zed.advertise(rgb_raw_topic, 1); //rgb raw
    NODELET_INFO_STREAM("Advertized on topic " << rgb_raw_topic);
    pub_left = it_zed.advertise(left_topic, 1); //left
    NODELET_INFO_STREAM("Advertized on topic " << left_topic);
    pub_raw_left = it_zed.advertise(left_raw_topic, 1); //left raw
    NODELET_INFO_STREAM("Advertized on topic " << left_raw_topic);
    pub_right = it_zed.advertise(right_topic, 1); //right
    NODELET_INFO_STREAM("Advertized on topic " << right_topic);
    pub_raw_right = it_zed.advertise(right_raw_topic, 1); //right raw
    NODELET_INFO_STREAM("Advertized on topic " << right_raw_topic);
    pub_depth = it_zed.advertise(depth_topic, 1); //depth
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);
    pub_conf_img = it_zed.advertise(conf_img_topic, 1); //confidence image
    NODELET_INFO_STREAM("Advertized on topic " << conf_img_topic);

    // Confidence Map publisher
    pub_conf_map = nh.advertise<sensor_msgs::Image>(conf_map_topic, 1); //confidence map
    NODELET_INFO_STREAM("Advertized on topic " << conf_map_topic);


    // Disparity publisher
    pub_disparity = nh.advertise<stereo_msgs::DisparityImage>(disparity_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << disparity_topic);

    //PointCloud publisher
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> (point_cloud_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << point_cloud_topic);

    // Camera info publishers
    pub_rgb_cam_info = nh.advertise<sensor_msgs::CameraInfo>(rgb_cam_info_topic, 1); //rgb
    NODELET_INFO_STREAM("Advertized on topic " << rgb_cam_info_topic);
    pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1); //left
    NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
    pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1); //right
    NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);
    pub_depth_cam_info = nh.advertise<sensor_msgs::CameraInfo>(depth_cam_info_topic, 1); //depth
    NODELET_INFO_STREAM("Advertized on topic " << depth_cam_info_topic);
    // Raw
    pub_rgb_cam_info_raw = nh.advertise<sensor_msgs::CameraInfo>(rgb_cam_info_raw_topic, 1); // raw rgb
    NODELET_INFO_STREAM("Advertized on topic " << rgb_cam_info_raw_topic);
    pub_left_cam_info_raw = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_raw_topic, 1); // raw left
    NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_raw_topic);
    pub_right_cam_info_raw = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_raw_topic, 1); // raw right
    NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_raw_topic);

    //Odometry publisher
    pub_odom = nh.advertise<nav_msgs::Odometry>(odometry_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << odometry_topic);

    // Imu publisher
    if(imu_pub_rate > 0 && realCamModel == sl::MODEL_ZED_M) {
        pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
        pub_imu_timer = nh_ns.createTimer(ros::Duration(1.0 / imu_pub_rate), &ZEDWrapperNodelet::imuPubCallback,this);
        NODELET_INFO_STREAM("Advertized on topic " << imu_topic << " @ " << imu_pub_rate << " Hz");
    } else if(imu_pub_rate > 0 && realCamModel == sl::MODEL_ZED) {
        NODELET_WARN_STREAM( "'imu_pub_rate' set to " << imu_pub_rate << " Hz" << " but ZED camera model does not support IMU data publishing.");
    }

    // Service
    srv_reset_tracking = nh.advertiseService( "reset_tracking", &ZEDWrapperNodelet::reset_tracking, this );

    // Start pool thread
    device_poll_thread = boost::shared_ptr<boost::thread> (new boost::thread(boost::bind(&ZEDWrapperNodelet::device_poll, this)));

}

sensor_msgs::ImagePtr ZEDWrapperNodelet::imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);

#pragma omp parallel for
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}

bool ZEDWrapperNodelet::reset_tracking(zed_wrapper::reset_tracking::Request  &req,
                                       zed_wrapper::reset_tracking::Response &res)
{
    if( !tracking_activated )
    {
        res.reset_done = false;
        return false;
    }

    nh_ns.getParam("initial_tracking_pose", initial_track_pose );

    sl::Transform initial_pose;

    if( initial_track_pose.size()!=6 ) {
        NODELET_WARN_STREAM("Invalid Initial Pose size (" << initial_track_pose.size() << "). Using Identity");
        initial_pose.setIdentity();
        base_transform.setIdentity();
    } else {
        // ROS initial pose
        initial_pose.setTranslation( sl::Translation( initial_track_pose[0], initial_track_pose[1], initial_track_pose[2] ) );

        tf2::Quaternion q;
        q.setRPY( initial_track_pose[3], initial_track_pose[4], initial_track_pose[5] );

        tf2::Vector3 orig( initial_track_pose[0], initial_track_pose[1], initial_track_pose[2] );
        base_transform.setOrigin(orig);
        base_transform.setRotation( q );

        // SL Tracking initial pose
        sl::float4 q_vec;
        q_vec[0] = q.getX();
        q_vec[1] = q.getY();
        q_vec[2] = q.getZ();
        q_vec[3] = q.getW();

        sl::Orientation r(q_vec);
        initial_pose.setOrientation( r );
    }

    zed.resetTracking( initial_pose );

    return true;
}

void ZEDWrapperNodelet::start_tracking() {
    NODELET_INFO_STREAM("Starting Tracking");

    nh_ns.getParam("odometry_DB", odometry_DB);
    nh_ns.getParam("pose_smoothing", pose_smoothing);
    NODELET_INFO_STREAM("Pose Smoothing : " << pose_smoothing);
    nh_ns.getParam("spatial_memory", spatial_memory);
    NODELET_INFO_STREAM("Spatial Memory : " << spatial_memory);

    nh_ns.getParam("initial_tracking_pose", initial_track_pose );

    sl::Transform initial_pose;

    if( initial_track_pose.size()!=6 ) {
        NODELET_WARN_STREAM("Invalid Initial Pose size (" << initial_track_pose.size() << "). Using Identity");
        initial_pose.setIdentity();
        base_transform.setIdentity();
    } else {
        NODELET_INFO_STREAM( "Initial pose: ["
                             << initial_track_pose[0] << ","
                                                      << initial_track_pose[1] << ","
                                                      << initial_track_pose[2] << ", "
                                                      << initial_track_pose[3] << ","
                                                      << initial_track_pose[4] << ","
                                                      << initial_track_pose[5] << "]");

        initial_pose.setTranslation( sl::Translation( initial_track_pose[0], initial_track_pose[1], initial_track_pose[2] ) );

        // ROS initial pose
        tf2::Quaternion q;
        q.setRPY( initial_track_pose[3], initial_track_pose[4], initial_track_pose[5] );

        tf2::Vector3 orig( initial_track_pose[0], initial_track_pose[1], initial_track_pose[2] );
        base_transform.setOrigin(orig);
        base_transform.setRotation( q );

        // SL Tracking initial pose
        sl::float4 q_vec;
        q_vec[0] = q.getX();
        q_vec[1] = q.getY();
        q_vec[2] = q.getZ();
        q_vec[3] = q.getW();

        sl::Orientation r(q_vec);
        initial_pose.setOrientation( r );
    }

    if (odometry_DB != "" && !sl_tools::file_exist(odometry_DB)) {
        odometry_DB = "";
        NODELET_WARN("odometry_DB path doesn't exist or is unreachable.");
    }

    // Tracking parameters
    sl::TrackingParameters trackParams;
    trackParams.area_file_path = odometry_DB.c_str();
    trackParams.enable_pose_smoothing = pose_smoothing;
    trackParams.enable_spatial_memory = spatial_memory;

    trackParams.initial_world_transform = initial_pose;



    zed.enableTracking(trackParams);
    tracking_activated = true;
}

void ZEDWrapperNodelet::publishOdom(tf2::Transform base_transform, ros::Time t) {
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odometry_frame_id; // odom_frame
    odom.child_frame_id = base_frame_id; // base_frame
    // conversion from Tranform to message
    geometry_msgs::Transform base2 = tf2::toMsg(base_transform);
    // Add all value in odometry message
    odom.pose.pose.position.x = base2.translation.x;
    odom.pose.pose.position.y = base2.translation.y;
    odom.pose.pose.position.z = base2.translation.z;
    odom.pose.pose.orientation.x = base2.rotation.x;
    odom.pose.pose.orientation.y = base2.rotation.y;
    odom.pose.pose.orientation.z = base2.rotation.z;
    odom.pose.pose.orientation.w = base2.rotation.w;
    // Publish odometry message
    pub_odom.publish(odom);
}

void ZEDWrapperNodelet::publishTrackedFrame(tf2::Transform base_transform, ros::Time t) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = t;
    transformStamped.header.frame_id = odometry_frame_id;
    transformStamped.child_frame_id = base_frame_id;
    // conversion from Tranform to message
    transformStamped.transform = tf2::toMsg(base_transform);
    // Publish transformation
    transform_odom_broadcaster.sendTransform(transformStamped);
}

void ZEDWrapperNodelet::publishImuFrame(tf2::Transform base_transform, ros::Time t) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = t;
    transformStamped.header.frame_id = base_frame_id;
    transformStamped.child_frame_id = imu_frame_id;
    // conversion from Tranform to message
    transformStamped.transform = tf2::toMsg(base_transform);
    // Publish transformation
    transform_imu_broadcaster.sendTransform(transformStamped);
}

void ZEDWrapperNodelet::publishImage(cv::Mat img, image_transport::Publisher &pub_img, string img_frame_id, ros::Time t) {
    pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::BGR8, img_frame_id, t));
}

void ZEDWrapperNodelet::publishDepth(cv::Mat depth, ros::Time t) {
    string encoding;
    if (openniDepthMode) {
        depth *= 1000.0f;
        depth.convertTo(depth, CV_16UC1); // in mm, rounded
        encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    } else {
        encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    }

    pub_depth.publish(imageToROSmsg(depth, encoding, depth_opt_frame_id, t));
}

void ZEDWrapperNodelet::publishDisparity(cv::Mat disparity, ros::Time t) {
    sl::CameraInformation zedParam = zed.getCameraInformation();
    sensor_msgs::ImagePtr disparity_image = imageToROSmsg(disparity,  sensor_msgs::image_encodings::TYPE_32FC1, disparity_frame_id, t);

    stereo_msgs::DisparityImage msg;
    msg.image = *disparity_image;
    msg.header = msg.image.header;
    msg.f = zedParam.calibration_parameters.left_cam.fx;
    msg.T = zedParam.calibration_parameters.T.x;
    msg.min_disparity = msg.f * msg.T / zed.getDepthMaxRangeValue();
    msg.max_disparity = msg.f * msg.T / zed.getDepthMinRangeValue();
    pub_disparity.publish(msg);
}

void ZEDWrapperNodelet::publishPointCloud(int width, int height) {
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    point_cloud.width = width;
    point_cloud.height = height;
    int size = width*height;
    point_cloud.points.resize(size);

    sl::Vector4<float>* cpu_cloud = cloud.getPtr<sl::float4>();

    if( param.coordinate_system == COORDINATE_SYSTEM_IMAGE ) {
#pragma omp parallel for
        for (int i = 0; i < size; i++) {
            // COORDINATE_SYSTEM_IMAGE
            point_cloud.points[i].x =  cpu_cloud[i][2];
            point_cloud.points[i].y = -cpu_cloud[i][0];
            point_cloud.points[i].z = -cpu_cloud[i][1];
            point_cloud.points[i].rgb = cpu_cloud[i][3];
        }
    } else if( param.coordinate_system == COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP ) {
#pragma omp parallel for
        for (int i = 0; i < size; i++) {
            // COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP
            point_cloud.points[i].x =  cpu_cloud[i][1];
            point_cloud.points[i].y = -cpu_cloud[i][0];
            point_cloud.points[i].z =  cpu_cloud[i][2];
            point_cloud.points[i].rgb = cpu_cloud[i][3];
        }
    } else if( param.coordinate_system == COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD ) {
#pragma omp parallel for
        for (int i = 0; i < size; i++) {
            // COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD
            point_cloud.points[i].x = cpu_cloud[i][0];
            point_cloud.points[i].y = cpu_cloud[i][1];
            point_cloud.points[i].z = cpu_cloud[i][2];
            point_cloud.points[i].rgb = cpu_cloud[i][3];
        }
        //memcpy( (float*)(&(point_cloud.points[0])), (float*)(&(cpu_cloud[0])), 4*size*sizeof(float)  );
    } else {
        NODELET_ERROR_STREAM("Camera coordinate system not supported");
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(point_cloud, output); // Convert the point cloud to a ROS message
    output.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
    output.header.stamp = point_cloud_time;
    output.height = height;
    output.width = width;
    output.is_bigendian = false;
    output.is_dense = false;
    pub_cloud.publish(output);
}

void ZEDWrapperNodelet::publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t) {
    static int seq = 0;
    cam_info_msg->header.stamp = t;
    cam_info_msg->header.seq = seq;
    pub_cam_info.publish(cam_info_msg);
    seq++;
}

void ZEDWrapperNodelet::fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,
                                    string left_frame_id, string right_frame_id, bool raw_param /*= false*/) {

    int width = zed.getResolution().width;
    int height = zed.getResolution().height;

    sl::CalibrationParameters zedParam;

    if (raw_param) zedParam = zed.getCameraInformation().calibration_parameters_raw;
    else zedParam = zed.getCameraInformation().calibration_parameters;

    float baseline = zedParam.T[0];

    left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    left_cam_info_msg->D.resize(5);
    right_cam_info_msg->D.resize(5);
    left_cam_info_msg->D[0] = zedParam.left_cam.disto[0]; // k1
    left_cam_info_msg->D[1] = zedParam.left_cam.disto[1]; // k2
    left_cam_info_msg->D[2] = zedParam.left_cam.disto[4]; // k3
    left_cam_info_msg->D[3] = zedParam.left_cam.disto[2]; // p1
    left_cam_info_msg->D[4] = zedParam.left_cam.disto[3]; // p2
    right_cam_info_msg->D[0] = zedParam.right_cam.disto[0]; // k1
    right_cam_info_msg->D[1] = zedParam.right_cam.disto[1]; // k2
    right_cam_info_msg->D[2] = zedParam.right_cam.disto[4]; // k3
    right_cam_info_msg->D[3] = zedParam.right_cam.disto[2]; // p1
    right_cam_info_msg->D[4] = zedParam.right_cam.disto[3]; // p2

    left_cam_info_msg->K.fill(0.0);
    right_cam_info_msg->K.fill(0.0);
    left_cam_info_msg->K[0] = zedParam.left_cam.fx;
    left_cam_info_msg->K[2] = zedParam.left_cam.cx;
    left_cam_info_msg->K[4] = zedParam.left_cam.fy;
    left_cam_info_msg->K[5] = zedParam.left_cam.cy;
    left_cam_info_msg->K[8] = right_cam_info_msg->K[8] = 1.0;
    right_cam_info_msg->K[0] = zedParam.right_cam.fx;
    right_cam_info_msg->K[2] = zedParam.right_cam.cx;
    right_cam_info_msg->K[4] = zedParam.right_cam.fy;
    right_cam_info_msg->K[5] = zedParam.right_cam.cy;

    left_cam_info_msg->R.fill(0.0);
    right_cam_info_msg->R.fill(0.0);
    for (int i = 0; i < 3; i++) {
        // identity
        right_cam_info_msg->R[i + i * 3] = 1;
        left_cam_info_msg->R[i + i * 3] = 1;
    }

    if (raw_param) {
        cv::Mat R_ = sl_tools::convertRodrigues(zedParam.R);
        float* p = (float*) R_.data;
        for (int i = 0; i < 9; i++)
            right_cam_info_msg->R[i] = p[i];
    }

    left_cam_info_msg->P.fill(0.0);
    right_cam_info_msg->P.fill(0.0);
    left_cam_info_msg->P[0] = zedParam.left_cam.fx;
    left_cam_info_msg->P[2] = zedParam.left_cam.cx;
    left_cam_info_msg->P[5] = zedParam.left_cam.fy;
    left_cam_info_msg->P[6] = zedParam.left_cam.cy;
    left_cam_info_msg->P[10] = right_cam_info_msg->P[10] = 1.0;
    //http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    right_cam_info_msg->P[3] = (-1 * zedParam.left_cam.fx * baseline);

    right_cam_info_msg->P[0] = zedParam.right_cam.fx;
    right_cam_info_msg->P[2] = zedParam.right_cam.cx;
    right_cam_info_msg->P[5] = zedParam.right_cam.fy;
    right_cam_info_msg->P[6] = zedParam.right_cam.cy;

    left_cam_info_msg->width = right_cam_info_msg->width = width;
    left_cam_info_msg->height = right_cam_info_msg->height = height;

    left_cam_info_msg->header.frame_id = left_frame_id;
    right_cam_info_msg->header.frame_id = right_frame_id;
}

void ZEDWrapperNodelet::dynamicReconfCallback(zed_wrapper::ZedConfig &config, uint32_t level) {
    switch (level) {
    case 0:
        NODELET_INFO("Reconfigure confidence : %d", config.confidence);
        confidence = config.confidence;
        break;
    case 1:
        NODELET_INFO("Reconfigure exposure : %d", config.exposure);
        exposure = config.exposure;
        break;
    case 2:
        NODELET_INFO("Reconfigure gain : %d", config.gain);
        gain = config.gain;
        break;
    case 3:
        NODELET_INFO("Reconfigure auto control of exposure and gain : %s", config.auto_exposure ? "Enable" : "Disable");
        autoExposure = config.auto_exposure;
        if (autoExposure)
            triggerAutoExposure = true;
        break;
    }
}

void ZEDWrapperNodelet::imuPubCallback(const ros::TimerEvent & e)
{
    int imu_SubNumber = pub_imu.getNumSubscribers();
    if (imu_SubNumber < 1) {
        return;
    }

    sl::IMUData imu_data;
    zed.getIMUData(imu_data, sl::TIME_REFERENCE_CURRENT);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time().fromNSec(imu_data.timestamp);
    imu_msg.header.frame_id = imu_frame_id;

    if( param.coordinate_system == COORDINATE_SYSTEM_IMAGE ) {
        // COORDINATE_SYSTEM_IMAGE
        imu_msg.orientation.x =  imu_data.getOrientation()[2];
        imu_msg.orientation.y = -imu_data.getOrientation()[0];
        imu_msg.orientation.z = -imu_data.getOrientation()[1];
        imu_msg.orientation.w =  imu_data.getOrientation()[3];

        imu_msg.angular_velocity.x =  imu_data.angular_velocity[2];
        imu_msg.angular_velocity.y = -imu_data.angular_velocity[0];
        imu_msg.angular_velocity.z = -imu_data.angular_velocity[1];

        imu_msg.linear_acceleration.x =  imu_data.linear_acceleration[2];
        imu_msg.linear_acceleration.y = -imu_data.linear_acceleration[0];
        imu_msg.linear_acceleration.z = -imu_data.linear_acceleration[1];

        for(int i = 0; i < 3; i+=3 )
        {
            imu_msg.orientation_covariance[i*3+0] = imu_data.orientation_covariance.r[i*3+2];
            imu_msg.orientation_covariance[i*3+1] = imu_data.orientation_covariance.r[i*3+0];
            imu_msg.orientation_covariance[i*3+2] = imu_data.orientation_covariance.r[i*3+1];

            imu_msg.linear_acceleration_covariance[i*3+0] = imu_data.linear_acceleration_convariance.r[i*3+2];
            imu_msg.linear_acceleration_covariance[i*3+1] = imu_data.linear_acceleration_convariance.r[i*3+0];
            imu_msg.linear_acceleration_covariance[i*3+2] = imu_data.linear_acceleration_convariance.r[i*3+1];

            imu_msg.angular_velocity_covariance[i*3+0] = imu_data.angular_velocity_convariance.r[i*3+2];
            imu_msg.angular_velocity_covariance[i*3+1] = imu_data.angular_velocity_convariance.r[i*3+0];
            imu_msg.angular_velocity_covariance[i*3+2] = imu_data.angular_velocity_convariance.r[i*3+1];
        }
    } else if( param.coordinate_system == COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP ) {
        // COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP
        imu_msg.orientation.x =  imu_data.getOrientation()[1];
        imu_msg.orientation.y = -imu_data.getOrientation()[0];
        imu_msg.orientation.z =  imu_data.getOrientation()[2];
        imu_msg.orientation.w =  imu_data.getOrientation()[3];

        imu_msg.angular_velocity.x =  imu_data.angular_velocity[1];
        imu_msg.angular_velocity.y = -imu_data.angular_velocity[0];
        imu_msg.angular_velocity.z =  imu_data.angular_velocity[2];

        imu_msg.linear_acceleration.x =  imu_data.linear_acceleration[1];
        imu_msg.linear_acceleration.y = -imu_data.linear_acceleration[0];
        imu_msg.linear_acceleration.z =  imu_data.linear_acceleration[2];

        for(int i = 0; i < 3; i+=3 )
        {
            imu_msg.orientation_covariance[i*3+0] = imu_data.orientation_covariance.r[i*3+1];
            imu_msg.orientation_covariance[i*3+1] = imu_data.orientation_covariance.r[i*3+0];
            imu_msg.orientation_covariance[i*3+2] = imu_data.orientation_covariance.r[i*3+2];

            imu_msg.linear_acceleration_covariance[i*3+0] = imu_data.linear_acceleration_convariance.r[i*3+1];
            imu_msg.linear_acceleration_covariance[i*3+1] = imu_data.linear_acceleration_convariance.r[i*3+0];
            imu_msg.linear_acceleration_covariance[i*3+2] = imu_data.linear_acceleration_convariance.r[i*3+2];

            imu_msg.angular_velocity_covariance[i*3+0] = imu_data.angular_velocity_convariance.r[i*3+1];
            imu_msg.angular_velocity_covariance[i*3+1] = imu_data.angular_velocity_convariance.r[i*3+0];
            imu_msg.angular_velocity_covariance[i*3+2] = imu_data.angular_velocity_convariance.r[i*3+2];
        }
    } else if( param.coordinate_system == COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD ) {
        // COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD
        imu_msg.orientation.x = imu_data.getOrientation()[0];
        imu_msg.orientation.y = imu_data.getOrientation()[1];
        imu_msg.orientation.z = imu_data.getOrientation()[2];
        imu_msg.orientation.w = imu_data.getOrientation()[3];

        imu_msg.angular_velocity.x = imu_data.angular_velocity[0];
        imu_msg.angular_velocity.y = imu_data.angular_velocity[1];
        imu_msg.angular_velocity.z = imu_data.angular_velocity[2];

        imu_msg.linear_acceleration.x = imu_data.linear_acceleration[0];
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration[1];
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration[2];

        for(int i = 0; i < 9; i++ )
        {
            imu_msg.orientation_covariance[i] = imu_data.orientation_covariance.r[i];
            imu_msg.linear_acceleration_covariance[i] = imu_data.linear_acceleration_convariance.r[i];
            imu_msg.angular_velocity_covariance[i] = imu_data.angular_velocity_convariance.r[i];
        }
    } else {
        NODELET_ERROR_STREAM("Camera coordinate system not supported");
    }

    pub_imu.publish(imu_msg);
}

void ZEDWrapperNodelet::device_poll() {
    ros::Rate loop_rate(rate);
    ros::Time old_t = ros::Time::now();
    sl::ERROR_CODE grab_status;
    tracking_activated = false;

    // Get the parameters of the ZED images
    int width = zed.getResolution().width;
    int height = zed.getResolution().height;
    NODELET_DEBUG_STREAM("Image size : " << width << "x" << height);

    cv::Size cvSize(width, height);
    cv::Mat leftImRGB(cvSize, CV_8UC3);
    cv::Mat rightImRGB(cvSize, CV_8UC3);
    cv::Mat confImRGB(cvSize, CV_8UC3);
    cv::Mat confMapFloat(cvSize, CV_32FC1);

    // Create and fill the camera information messages
    sensor_msgs::CameraInfoPtr rgb_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr rgb_cam_info_raw_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr left_cam_info_raw_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info_raw_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
    fillCamInfo(zed, left_cam_info_msg, right_cam_info_msg, left_cam_opt_frame_id, right_cam_opt_frame_id);
    fillCamInfo(zed, left_cam_info_raw_msg, right_cam_info_raw_msg, left_cam_opt_frame_id, right_cam_opt_frame_id, true);
    rgb_cam_info_msg = depth_cam_info_msg = left_cam_info_msg; // the reference camera is the Left one (next to the ZED logo)
    rgb_cam_info_raw_msg = left_cam_info_raw_msg;

    // Disable spatial mapping to improve performaces
    zed.disableSpatialMapping();

    sl::RuntimeParameters runParams;
    runParams.sensing_mode = static_cast<sl::SENSING_MODE> (sensing_mode);




    sl::Mat leftZEDMat, rightZEDMat, depthZEDMat, disparityZEDMat, confImgZEDMat, confMapZEDMat;
    // Main loop
    while (nh_ns.ok()) {
        // Check for subscribers
        int rgb_SubNumber = pub_rgb.getNumSubscribers();
        int rgb_raw_SubNumber = pub_raw_rgb.getNumSubscribers();
        int left_SubNumber = pub_left.getNumSubscribers();
        int left_raw_SubNumber = pub_raw_left.getNumSubscribers();
        int right_SubNumber = pub_right.getNumSubscribers();
        int right_raw_SubNumber = pub_raw_right.getNumSubscribers();
        int depth_SubNumber = pub_depth.getNumSubscribers();
        int disparity_SubNumber = pub_disparity.getNumSubscribers();
        int cloud_SubNumber = pub_cloud.getNumSubscribers();
        int odom_SubNumber = pub_odom.getNumSubscribers();
        int conf_img_SubNumber = pub_conf_img.getNumSubscribers();
        int conf_map_SubNumber = pub_conf_map.getNumSubscribers();
        bool runLoop = (rgb_SubNumber + rgb_raw_SubNumber + left_SubNumber + left_raw_SubNumber + right_SubNumber +
                        right_raw_SubNumber + depth_SubNumber + disparity_SubNumber + cloud_SubNumber + odom_SubNumber + conf_img_SubNumber + conf_map_SubNumber) > 0;

        runParams.enable_point_cloud = false;
        if (cloud_SubNumber > 0)
            runParams.enable_point_cloud = true;
        // Run the loop only if there is some subscribers
        if (runLoop) {
            if ((depth_stabilization || odom_SubNumber > 0 || cloud_SubNumber > 0 || depth_SubNumber > 0) && !tracking_activated) { //Start the tracking
                start_tracking();
            } else if (!depth_stabilization && odom_SubNumber == 0 && tracking_activated) { //Stop the tracking
                zed.disableTracking();
                tracking_activated = false;
            }
            computeDepth = (depth_SubNumber + disparity_SubNumber + cloud_SubNumber + odom_SubNumber + conf_img_SubNumber + conf_map_SubNumber) > 0; // Detect if one of the subscriber need to have the depth information
            ros::Time t = ros::Time::now(); // Get current time

            grabbing = true;
            if (computeDepth) {
                int actual_confidence = zed.getConfidenceThreshold();
                if (actual_confidence != confidence)
                    zed.setConfidenceThreshold(confidence);
                runParams.enable_depth = true; // Ask to compute the depth
            } else
                runParams.enable_depth = false;

            grab_status = zed.grab(runParams); // Ask to not compute the depth

            grabbing = false;

            //cout << toString(grab_status) << endl;
            if (grab_status != sl::ERROR_CODE::SUCCESS) { // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED

                if (grab_status == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                    NODELET_DEBUG("Wait for a new image to proceed");
                } else NODELET_INFO_STREAM_ONCE(toString(grab_status));

                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                if ((t - old_t).toSec() > 5) {
                    zed.close();

                    NODELET_INFO("Re-opening the ZED");
                    sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
                    while (err != sl::SUCCESS) {

                        if( !nh_ns.ok() )
                        {
                            zed.close();
                            return;
                        }

                        int id = sl_tools::checkCameraReady(serial_number);
                        if (id > 0) {
                            param.camera_linux_id = id;
                            err = zed.open(param); // Try to initialize the ZED
                            NODELET_INFO_STREAM(toString(err));
                        } else NODELET_INFO("Waiting for the ZED to be re-connected");
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    }
                    tracking_activated = false;
                    if (depth_stabilization || odom_SubNumber > 0) { //Start the tracking
                        start_tracking();
                    }
                }
                continue;
            }

            old_t = ros::Time::now();

            if (autoExposure) {
                // getCameraSettings() can't check status of auto exposure
                // triggerAutoExposure is used to execute setCameraSettings() only once
                if (triggerAutoExposure) {
                    zed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, 0, true);
                    triggerAutoExposure = false;
                }
            } else {
                int actual_exposure = zed.getCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE);
                if (actual_exposure != exposure)
                    zed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, exposure);

                int actual_gain = zed.getCameraSettings(sl::CAMERA_SETTINGS_GAIN);
                if (actual_gain != gain)
                    zed.setCameraSettings(sl::CAMERA_SETTINGS_GAIN, gain);
            }

            // Publish the left == rgb image if someone has subscribed to
            if (left_SubNumber > 0 || rgb_SubNumber > 0) {
                // Retrieve RGBA Left image
                zed.retrieveImage(leftZEDMat, sl::VIEW_LEFT);
                cv::cvtColor(sl_tools::toCVMat(leftZEDMat), leftImRGB, CV_RGBA2RGB);
                if (left_SubNumber > 0) {
                    publishCamInfo(left_cam_info_msg, pub_left_cam_info, t);
                    publishImage(leftImRGB, pub_left, left_cam_opt_frame_id, t);
                }
                if (rgb_SubNumber > 0) {
                    publishCamInfo(rgb_cam_info_msg, pub_rgb_cam_info, t);
                    publishImage(leftImRGB, pub_rgb, depth_opt_frame_id, t); // rgb is the left image
                }
            }

            // Publish the left_raw == rgb_raw image if someone has subscribed to
            if (left_raw_SubNumber > 0 || rgb_raw_SubNumber > 0) {
                // Retrieve RGBA Left image
                zed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED);
                cv::cvtColor(sl_tools::toCVMat(leftZEDMat), leftImRGB, CV_RGBA2RGB);
                if (left_raw_SubNumber > 0) {
                    publishCamInfo(left_cam_info_raw_msg, pub_left_cam_info_raw, t);
                    publishImage(leftImRGB, pub_raw_left, left_cam_opt_frame_id, t);
                }
                if (rgb_raw_SubNumber > 0) {
                    publishCamInfo(rgb_cam_info_raw_msg, pub_rgb_cam_info_raw, t);
                    publishImage(leftImRGB, pub_raw_rgb, depth_opt_frame_id, t);
                }
            }

            // Publish the right image if someone has subscribed to
            if (right_SubNumber > 0) {
                // Retrieve RGBA Right image
                zed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT);
                cv::cvtColor(sl_tools::toCVMat(rightZEDMat), rightImRGB, CV_RGBA2RGB);
                publishCamInfo(right_cam_info_msg, pub_right_cam_info, t);
                publishImage(rightImRGB, pub_right, right_cam_opt_frame_id, t);
            }

            // Publish the right image if someone has subscribed to
            if (right_raw_SubNumber > 0) {
                // Retrieve RGBA Right image
                zed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED);
                cv::cvtColor(sl_tools::toCVMat(rightZEDMat), rightImRGB, CV_RGBA2RGB);
                publishCamInfo(right_cam_info_raw_msg, pub_right_cam_info_raw, t);
                publishImage(rightImRGB, pub_raw_right, right_cam_opt_frame_id, t);
            }

            // Publish the depth image if someone has subscribed to
            if (depth_SubNumber > 0 || disparity_SubNumber > 0) {
                zed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH);
                publishCamInfo(depth_cam_info_msg, pub_depth_cam_info, t);
                publishDepth(sl_tools::toCVMat(depthZEDMat), t); // in meters
            }

            // Publish the disparity image if someone has subscribed to
            if (disparity_SubNumber > 0) {
                zed.retrieveMeasure(disparityZEDMat, sl::MEASURE_DISPARITY);
                // Need to flip sign, but cause of this is not sure
                cv::Mat disparity = sl_tools::toCVMat(disparityZEDMat) * -1.0;
                publishDisparity(disparity, t);
            }

            // Publish the confidence image if someone has subscribed to
            if (conf_img_SubNumber > 0) {
                zed.retrieveImage(confImgZEDMat, sl::VIEW_CONFIDENCE);
                cv::cvtColor(sl_tools::toCVMat(confImgZEDMat), confImRGB, CV_RGBA2RGB);
                publishImage(confImRGB, pub_conf_img, confidence_opt_frame_id, t);
            }

            // Publish the confidence map if someone has subscribed to
            if (conf_map_SubNumber > 0) {
                zed.retrieveMeasure(confMapZEDMat, sl::MEASURE_CONFIDENCE);
                confMapFloat = sl_tools::toCVMat(confMapZEDMat);
                pub_conf_map.publish(imageToROSmsg(confMapFloat, sensor_msgs::image_encodings::TYPE_32FC1, confidence_opt_frame_id, t));
            }

            // Publish the point cloud if someone has subscribed to
            if (cloud_SubNumber > 0) {
                // Run the point cloud conversion asynchronously to avoid slowing down all the program
                // Retrieve raw pointCloud data
                zed.retrieveMeasure(cloud, sl::MEASURE_XYZBGRA);
                point_cloud_frame_id = depth_frame_id;
                point_cloud_time = t;
                publishPointCloud(width, height );
            }

            // Transform from base to sensor
            tf2::Transform base_to_sensor;
            // Look up the transformation from base frame to camera link
            try {
                // Save the transformation from base to frame
                geometry_msgs::TransformStamped b2s = tfBuffer->lookupTransform(base_frame_id, depth_frame_id, t);
                // Get the TF2 transformation
                tf2::fromMsg(b2s.transform, base_to_sensor);

            } catch (tf2::TransformException &ex) {
                NODELET_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                                            "will assume it as identity!",
                                      base_frame_id.c_str(),
                                      depth_frame_id.c_str());
                NODELET_DEBUG("Transform error: %s", ex.what());
                base_to_sensor.setIdentity();
            }

            // Publish IMU tf only if enabled
            if (publish_tf) {
                // TODO take R,t from SDK
                //zed.get..........

                geometry_msgs::Transform i2b;
                // TODO transform....

                tf2::Transform imu_transform;

                tf2::fromMsg(i2b, imu_transform);


                imu_transform.setIdentity(); // TODO remove!!!

                //Note, the frame is published, but its values will only change if someone has subscribed to odom
                publishImuFrame(imu_transform, t); //publish the tracked Frame
            }

            // Publish the odometry if someone has subscribed to
            if (odom_SubNumber > 0 || cloud_SubNumber > 0 || depth_SubNumber > 0) {
                zed.getPosition(pose);

                // Transform ZED pose in TF2 Transformation
                tf2::Transform camera_transform;
                geometry_msgs::Transform c2s;

                sl::Translation translation = pose.getTranslation();
                sl::Orientation quat = pose.getOrientation();

                if( param.coordinate_system == COORDINATE_SYSTEM_IMAGE ) {
                    // COORDINATE_SYSTEM_IMAGE
                    c2s.translation.x =  translation(2);
                    c2s.translation.y = -translation(0);
                    c2s.translation.z = -translation(1);

                    c2s.rotation.x =  quat(2);
                    c2s.rotation.y = -quat(0);
                    c2s.rotation.z = -quat(1);
                    c2s.rotation.w =  quat(3);
                } else if( param.coordinate_system == COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP ) {
                    // COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP
                    c2s.translation.x =  translation(1);
                    c2s.translation.y = -translation(0);
                    c2s.translation.z =  translation(2);

                    c2s.rotation.x =  quat(1);
                    c2s.rotation.y = -quat(0);
                    c2s.rotation.z =  quat(2);
                    c2s.rotation.w =  quat(3);
                } else if( param.coordinate_system == COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD ) {
                    // COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD
                    c2s.translation.x = translation(0);
                    c2s.translation.y = translation(1);
                    c2s.translation.z = translation(2);

                    c2s.rotation.x = quat(0);
                    c2s.rotation.y = quat(1);
                    c2s.rotation.z = quat(2);
                    c2s.rotation.w = quat(3);
                } else {
                    NODELET_ERROR_STREAM("Camera coordinate system not supported");
                }

                tf2::fromMsg(c2s, camera_transform);
                // Transformation from camera sensor to base frame
                base_transform = base_to_sensor * camera_transform * base_to_sensor.inverse();
                // Publish odometry message
                publishOdom(base_transform, t);
            }

            // Publish odometry tf only if enabled
            if (publish_tf) {
                //Note, the frame is published, but its values will only change if someone has subscribed to odom
                publishTrackedFrame(base_transform, t); //publish the tracked Frame
            }

            loop_rate.sleep();
        } else {
            // Publish odometry tf only if enabled
            if (publish_tf) {
                publishTrackedFrame(base_transform, ros::Time::now()); //publish the tracked Frame before the sleep
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // No subscribers, we just wait
        }
    } // while loop

    zed.close();
}

} // namespace
