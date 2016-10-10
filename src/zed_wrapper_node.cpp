///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, STEREOLABS.
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




/****************************************************************************************************
 ** This sample is a wrapper for the ZED library in order to use the ZED Camera with ROS.          **
 ** You can publish Left+Depth or Left+Right images and camera info on the topics of your choice.  **
 **                                                                                                **
 ** A set of parameters can be specified in the launch file.                                       **
 ****************************************************************************************************/

//standard includes
#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
#include <thread>
#include <chrono>
#include <memory>
#include <sys/stat.h> // file exists

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <zed_wrapper/ZedConfig.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//Boost includes
#include <boost/make_shared.hpp>

//ZED Includes
#include <zed/Camera.hpp>

using namespace sl::zed;
using namespace std;


int confidence;
bool computeDepth;
int openniDepthMode = 0; // 16 bit UC data in mm else 32F in m, for more info http://www.ros.org/reps/rep-0118.html

// Point cloud thread variables
#include "point_cloud.h"

/* \brief Test if a file exist
 * \param name : the path to the file
 */
bool file_exist(const std::string& name) {
  struct stat buffer;   
  return (stat(name.c_str(), &buffer) == 0); 
}

/* \brief Image to ros message conversion 
 * \param img : the image to publish
 * \param encodingType : the sensor_msgs::image_encodings encoding type
 * \param frameId : the id of the reference frame of the image
 * \param t : the ros::Time to stamp the image
*/
sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t){
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *)&num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*)(&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*)(&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}

/* \brief Publish the pose of the camera with a ros Publisher
 * \param Path : the 4x4 matrix representing the camera pose
 * \param pub_odom : the publisher object to use
 * \param odom_frame_id : the id of the reference frame of the pose
 * \param t : the ros::Time to stamp the image
 */
void publishOdom(Eigen::Matrix4f Path, ros::Publisher &pub_odom, string odom_frame_id, ros::Time t) {
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_id;
    //odom.child_frame_id = "zed_optical_frame";

    odom.pose.pose.position.y = -Path(0, 3);
    odom.pose.pose.position.z = Path(1, 3);
    odom.pose.pose.position.x = -Path(2, 3);
    Eigen::Quaternionf quat(Path.block<3, 3>(0, 0));
    odom.pose.pose.orientation.x = -quat.z();
    odom.pose.pose.orientation.y = -quat.x();
    odom.pose.pose.orientation.z = quat.y();
    odom.pose.pose.orientation.w = quat.w();
    pub_odom.publish(odom);
}

/* \brief Publish the pose of the camera as a transformation
 * \param Path : the 4x4 matrix representing the camera pose
 * \param trans_br : the TransformBroadcaster object to use
 * \param odometry_transform_frame_id : the id of the transformation
 * \param t : the ros::Time to stamp the image
 */
void publishTrackedFrame(Eigen::Matrix4f Path, tf2_ros::TransformBroadcaster &trans_br, string odometry_transform_frame_id, ros::Time t) {

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "zed_initial_frame";
    transformStamped.child_frame_id = odometry_transform_frame_id;
    transformStamped.transform.translation.x = -Path(2, 3);
    transformStamped.transform.translation.y = -Path(0, 3);
    transformStamped.transform.translation.z = Path(1, 3);
    Eigen::Quaternionf quat(Path.block<3, 3>(0, 0));
    transformStamped.transform.rotation.x = -quat.z();
    transformStamped.transform.rotation.y = -quat.x();
    transformStamped.transform.rotation.z = quat.y();
    transformStamped.transform.rotation.w = quat.w();
    trans_br.sendTransform(transformStamped);
}

/* \brief Publish a cv::Mat image with a ros Publisher
 * \param img : the image to publish
 * \param pub_img : the publisher object to use
 * \param img_frame_id : the id of the reference frame of the image
 * \param t : the ros::Time to stamp the image
 */
void publishImage(cv::Mat img, image_transport::Publisher &pub_img, string img_frame_id, ros::Time t) {
    pub_img.publish(imageToROSmsg(img
                                , sensor_msgs::image_encodings::BGR8
                                , img_frame_id
                                , t ));
}

/* \brief Publish a cv::Mat depth image with a ros Publisher
 * \param depth : the depth image to publish
 * \param pub_depth : the publisher object to use
 * \param depth_frame_id : the id of the reference frame of the depth image
 * \param t : the ros::Time to stamp the depth image
 */
void publishDepth(cv::Mat depth, image_transport::Publisher &pub_depth, string depth_frame_id, ros::Time t) {
    string encoding;
    if(openniDepthMode){
        depth *= 1000.0f;
        depth.convertTo(depth, CV_16UC1); // in mm, rounded
        encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    }
    else {
        encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    }
    pub_depth.publish(imageToROSmsg(depth
                                , encoding
                                , depth_frame_id
                                , t ));
}

/* \brief Publish the informations of a camera with a ros Publisher
 * \param cam_info_msg : the information message to publish
 * \param pub_cam_info : the publisher object to use
 * \param t : the ros::Time to stamp the message
 */
void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t) {
    static int seq = 0;
    cam_info_msg->header.stamp = t;
    cam_info_msg->header.seq = seq;
    pub_cam_info.publish(cam_info_msg);
    seq++;
}

/* \brief Get the information of the ZED cameras and store them in an information message
 * \param zed : the sl::zed::Camera* pointer to an instance
 * \param left_cam_info_msg : the information message to fill with the left camera informations
 * \param right_cam_info_msg : the information message to fill with the right camera informations
 * \param left_frame_id : the id of the reference frame of the left camera
 * \param right_frame_id : the id of the reference frame of the right camera
 */
void fillCamInfo(Camera* zed, sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,
        string left_frame_id, string right_frame_id) {

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    sl::zed::StereoParameters* zedParam = zed->getParameters();

    float baseline = zedParam->baseline * 0.001; // baseline converted in meters

    float fx = zedParam->LeftCam.fx;
    float fy = zedParam->LeftCam.fy;
    float cx = zedParam->LeftCam.cx;
    float cy = zedParam->LeftCam.cy;

    // There is no distorsions since the images are rectified
    double k1 = 0;
    double k2 = 0;
    double k3 = 0;
    double p1 = 0;
    double p2 = 0;

    left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    left_cam_info_msg->D.resize(5);
    right_cam_info_msg->D.resize(5);
    left_cam_info_msg->D[0] = right_cam_info_msg->D[0] = k1;
    left_cam_info_msg->D[1] = right_cam_info_msg->D[1] = k2;
    left_cam_info_msg->D[2] = right_cam_info_msg->D[2] = k3;
    left_cam_info_msg->D[3] = right_cam_info_msg->D[3] = p1;
    left_cam_info_msg->D[4] = right_cam_info_msg->D[4] = p2;

    left_cam_info_msg->K.fill(0.0);
    right_cam_info_msg->K.fill(0.0);
    left_cam_info_msg->K[0] = right_cam_info_msg->K[0] = fx;
    left_cam_info_msg->K[2] = right_cam_info_msg->K[2] = cx;
    left_cam_info_msg->K[4] = right_cam_info_msg->K[4] = fy;
    left_cam_info_msg->K[5] = right_cam_info_msg->K[5] = cy;
    left_cam_info_msg->K[8] = right_cam_info_msg->K[8] = 1.0;

    left_cam_info_msg->R.fill(0.0);
    right_cam_info_msg->R.fill(0.0);

    left_cam_info_msg->P.fill(0.0);
    right_cam_info_msg->P.fill(0.0);
    left_cam_info_msg->P[0] = right_cam_info_msg->P[0] = fx;
    left_cam_info_msg->P[2] = right_cam_info_msg->P[2] = cx;
    left_cam_info_msg->P[5] = right_cam_info_msg->P[5] = fy;
    left_cam_info_msg->P[6] = right_cam_info_msg->P[6] = cy;
    left_cam_info_msg->P[10] = right_cam_info_msg->P[10] = 1.0;
    right_cam_info_msg->P[3] = (-1 * fx * baseline);

    left_cam_info_msg->width = right_cam_info_msg->width = width;
    left_cam_info_msg->height = right_cam_info_msg->height = height;

    left_cam_info_msg->header.frame_id = left_frame_id;
    right_cam_info_msg->header.frame_id = right_frame_id;
}

void callback(zed_ros_wrapper::ZedConfig &config, uint32_t level) {
	config.confidence = 100;
    ROS_INFO("Reconfigure confidence : %d", config.confidence);
    confidence = config.confidence;
}

int main(int argc, char **argv) {
    // Launch file parameters
    int resolution = sl::zed::HD720;
    int quality = sl::zed::MODE::PERFORMANCE;
    int sensing_mode = sl::zed::SENSING_MODE::STANDARD;
    int rate = 30;
    string odometry_DB = "";

    std::string img_topic = "image_rect";

    // Set the default topic names
    string rgb_topic = "rgb/" + img_topic;
    string rgb_cam_info_topic = "rgb/camera_info";
    string rgb_frame_id = "/zed_tracked_frame";

    string left_topic = "left/" + img_topic;
    string left_cam_info_topic = "left/camera_info";
    string left_frame_id = "/zed_tracked_frame";

    string right_topic = "right/" + img_topic;
    string right_cam_info_topic = "right/camera_info";
    string right_frame_id = "/zed_tracked_frame";

    string depth_topic = "depth/";
    if (openniDepthMode)
        depth_topic += "image_raw";
    else
        depth_topic += img_topic;

    string depth_cam_info_topic = "depth/camera_info";
    string depth_frame_id = "/zed_depth_optical_frame";

    string point_cloud_topic = "point_cloud/" + img_topic;
    string cloud_frame_id = "/zed_tracked_frame";

    string odometry_topic = "odom";
    string odometry_frame_id = "/zed_initial_frame";
    string odometry_transform_frame_id = "/zed_tracked_frame";

    ros::init(argc, argv, "zed_depth_stereo_wrapper_node");
    ROS_INFO("ZED_WRAPPER Node initialized");

    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    // Get parameters from launch file
    nh_ns.getParam("resolution", resolution);
    nh_ns.getParam("quality", quality);
    nh_ns.getParam("sensing_mode", sensing_mode);
    nh_ns.getParam("frame_rate", rate);
    nh_ns.getParam("odometry_DB", odometry_DB);
    nh_ns.getParam("openni_depth_mode", openniDepthMode);
    if (openniDepthMode)
        ROS_INFO_STREAM("Openni depth mode activated");

    nh_ns.getParam("rgb_topic", rgb_topic);
    nh_ns.getParam("rgb_cam_info_topic", rgb_cam_info_topic);
    nh_ns.getParam("rgb_frame_id", rgb_frame_id);

    nh_ns.getParam("left_topic", left_topic);
    nh_ns.getParam("left_cam_info_topic", left_cam_info_topic);
    nh_ns.getParam("left_frame_id", left_frame_id);

    nh_ns.getParam("right_topic", right_topic);
    nh_ns.getParam("right_cam_info_topic", right_cam_info_topic);
    nh_ns.getParam("right_frame_id", right_frame_id);

    nh_ns.getParam("depth_topic", depth_topic);
    nh_ns.getParam("depth_cam_info_topic", depth_cam_info_topic);
    nh_ns.getParam("depth_frame_id", depth_frame_id);

    nh_ns.getParam("point_cloud_topic", point_cloud_topic);
    nh_ns.getParam("cloud_frame_id", cloud_frame_id);

    nh_ns.getParam("odometry_topic", odometry_topic);
    nh_ns.getParam("odometry_frame_id", odometry_frame_id);
    nh_ns.getParam("odometry_transform_frame_id", odometry_transform_frame_id);

    // Create the ZED object
    std::unique_ptr<sl::zed::Camera> zed;
    if (argc == 2) {
        zed.reset(new sl::zed::Camera(argv[1])); // Argument "svo_file" in launch file
        ROS_INFO_STREAM("Reading SVO file : " << argv[1]);
    } else {
        zed.reset(new sl::zed::Camera(static_cast<sl::zed::ZEDResolution_mode> (resolution), rate));
        ROS_INFO_STREAM("Using ZED Camera");
    }

    // Try to initialize the ZED
    sl::zed::InitParams param;
    param.unit = UNIT::METER;
    param.coordinate = COORDINATE_SYSTEM::RIGHT_HANDED;
    param.mode = static_cast<sl::zed::MODE> (quality);
    param.verbose = true;

    ERRCODE err = ERRCODE::ZED_NOT_AVAILABLE;
    while (err != SUCCESS) {
        err = zed->init(param);
        ROS_INFO_STREAM(errcode2str(err));
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    zed->grab(static_cast<sl::zed::SENSING_MODE> (sensing_mode), true, true, true); //call the first grab

    //Tracking variables
    sl::zed::TRACKING_STATE track_state;
    sl::zed::TRACKING_FRAME_STATE frame_state;
    Eigen::Matrix4f Path;
    Path.setIdentity(4, 4);

    //ERRCODE display
    dynamic_reconfigure::Server<zed_ros_wrapper::ZedConfig> server;
    dynamic_reconfigure::Server<zed_ros_wrapper::ZedConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    confidence = 100;

    // Get the parameters of the ZED images
    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;
    ROS_DEBUG_STREAM("Image size : " << width << "x" << height);

    cv::Size cvSize(width, height);
    cv::Mat leftImRGB(cvSize, CV_8UC3);
    cv::Mat rightImRGB(cvSize, CV_8UC3);


    // Create all the publishers
    // Image publishers
    image_transport::ImageTransport it_zed(nh);
    image_transport::Publisher pub_rgb = it_zed.advertise(rgb_topic, 1); //rgb
    ROS_INFO_STREAM("Advertized on topic " << rgb_topic);
    image_transport::Publisher pub_left = it_zed.advertise(left_topic, 1); //left
    ROS_INFO_STREAM("Advertized on topic " << left_topic);
    image_transport::Publisher pub_right = it_zed.advertise(right_topic, 1); //right
    ROS_INFO_STREAM("Advertized on topic " << right_topic);
    image_transport::Publisher pub_depth = it_zed.advertise(depth_topic, 1); //depth
    ROS_INFO_STREAM("Advertized on topic " << depth_topic);

    //PointCloud publisher
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2> (point_cloud_topic, 1);
    ROS_INFO_STREAM("Advertized on topic " << point_cloud_topic);

    // Camera info publishers
    ros::Publisher pub_rgb_cam_info = nh.advertise<sensor_msgs::CameraInfo>(rgb_cam_info_topic, 1); //rgb
    ROS_INFO_STREAM("Advertized on topic " << rgb_cam_info_topic);
    ros::Publisher pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1); //left
    ROS_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
    ros::Publisher pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1); //right
    ROS_INFO_STREAM("Advertized on topic " << right_cam_info_topic);
    ros::Publisher pub_depth_cam_info = nh.advertise<sensor_msgs::CameraInfo>(depth_cam_info_topic, 1); //depth
    ROS_INFO_STREAM("Advertized on topic " << depth_cam_info_topic);

    //Odometry publisher
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>(odometry_topic, 1);
    tf2_ros::TransformBroadcaster transform_odom_broadcaster;
    ROS_INFO_STREAM("Advertized on topic " << odometry_topic);

    // Create and fill the camera information messages
    sensor_msgs::CameraInfoPtr rgb_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
    fillCamInfo(zed.get(), left_cam_info_msg, right_cam_info_msg, left_frame_id, right_frame_id);
    rgb_cam_info_msg = depth_cam_info_msg = left_cam_info_msg; // the reference camera is the Left one (next to the ZED logo)

    ros::Rate loop_rate(rate);
    ros::Time old_t = ros::Time::now();
    bool old_image = false;
    bool tracking_activated = false;
    
    zw_cloud::start(pub_cloud);

    try {
        // Main loop
        while (ros::ok()) {
            // Check for subscribers
            int rgb_SubNumber = pub_rgb.getNumSubscribers();
            int left_SubNumber = pub_left.getNumSubscribers();
            int right_SubNumber = pub_right.getNumSubscribers();
            int depth_SubNumber = pub_depth.getNumSubscribers();
            int cloud_SubNumber = pub_cloud.getNumSubscribers();
            int odom_SubNumber = pub_odom.getNumSubscribers();
            bool runLoop = (rgb_SubNumber + left_SubNumber + right_SubNumber + depth_SubNumber + cloud_SubNumber + odom_SubNumber) > 0;
            // Run the loop only if there is some subscribers
            if (runLoop) {
                if (odom_SubNumber > 0 && !tracking_activated) { //Start the tracking
                    if (odometry_DB != "" && !file_exist(odometry_DB)) {
                        odometry_DB = "";
                        ROS_WARN("odometry_DB path doesn't exist or is unreachable.");
                    }
                    zed->enableTracking(Path, true, odometry_DB);
                    tracking_activated = true;
                } else if (odom_SubNumber == 0 && tracking_activated) { //Stop the tracking
                    zed->stopTracking();
                    tracking_activated = false;
                }
                computeDepth = (depth_SubNumber + cloud_SubNumber + odom_SubNumber) > 0; // Detect if one of the subscriber need to have the depth information
                ros::Time t = ros::Time::now(); // Get current time

                if (computeDepth) {
                    int actual_confidence = zed->getConfidenceThreshold();
                    if (actual_confidence != confidence)
                        zed->setConfidenceThreshold(100 /*confidence*/);
                    old_image = zed->grab(static_cast<sl::zed::SENSING_MODE> (sensing_mode), true, true, cloud_SubNumber > 0); // Ask to compute the depth
                } else
                    old_image = zed->grab(static_cast<sl::zed::SENSING_MODE> (sensing_mode), false, false); // Ask to not compute the depth


                if (old_image) { // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED
                    ROS_DEBUG("Wait for a new image to proceed");
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    if ((t - old_t).toSec() > 5) {
                        // delete the old object before constructing a new one
                        zed.reset();
                        if (argc == 2) {
                            zed.reset(new sl::zed::Camera(argv[1])); // Argument "svo_file" in launch file
                            ROS_INFO_STREAM("Reading SVO file : " << argv[1]);
                        } else {
                            zed.reset(new sl::zed::Camera(static_cast<sl::zed::ZEDResolution_mode> (resolution), rate));
                            ROS_INFO_STREAM("Using ZED Camera");
                        }
                        ROS_INFO("Reinit camera");
                        ERRCODE err = ERRCODE::ZED_NOT_AVAILABLE;
                        while (err != SUCCESS) {
                            err = zed->init(param); // Try to initialize the ZED
                            ROS_INFO_STREAM(errcode2str(err));
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        }
                        zed->grab(static_cast<sl::zed::SENSING_MODE> (sensing_mode), true, true, cloud_SubNumber > 0); //call the first grab
                        tracking_activated = false;
                        if (odom_SubNumber > 0) { //Start the tracking
                            if (odometry_DB != "" && !file_exist(odometry_DB)) {
                                odometry_DB = "";
                                ROS_WARN("odometry_DB path doesn't exist or is unreachable.");
                            }
                            zed->enableTracking(Path, true, odometry_DB);
                            tracking_activated = true;
                        }
                    }
                    continue;
                }

                old_t = ros::Time::now();

                // Publish the left == rgb image if someone has subscribed to
                if (left_SubNumber > 0 || rgb_SubNumber > 0) {
                    // Retrieve RGBA Left image
                    cv::cvtColor(slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::LEFT)), leftImRGB, CV_RGBA2RGB); // Convert to RGB
                    if (left_SubNumber > 0) {
                        publishCamInfo(left_cam_info_msg, pub_left_cam_info, t);
                        publishImage(leftImRGB, pub_left, left_frame_id, t);
                    }
                    if (rgb_SubNumber > 0) {
                        publishCamInfo(rgb_cam_info_msg, pub_rgb_cam_info, t);
                        publishImage(leftImRGB, pub_rgb, rgb_frame_id, t); // rgb is the left image
                    }
                }

                // Publish the right image if someone has subscribed to
                if (right_SubNumber > 0) {
                    // Retrieve RGBA Right image
                    cv::cvtColor(slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::RIGHT)), rightImRGB, CV_RGBA2RGB); // Convert to RGB
                    publishCamInfo(right_cam_info_msg, pub_right_cam_info, t);
                    publishImage(rightImRGB, pub_right, right_frame_id, t);
                }

                // Publish the depth image if someone has subscribed to
                if (depth_SubNumber > 0) {
                    publishCamInfo(depth_cam_info_msg, pub_depth_cam_info, t);
                    publishDepth(slMat2cvMat(zed->retrieveMeasure(sl::zed::MEASURE::DEPTH)), pub_depth, depth_frame_id, t); // in meters
                }

                // Publish the point cloud if someone has subscribed to
                if (cloud_SubNumber > 0) {
					zw_cloud::store(*zed, cloud_frame_id, t);
                }

                // Publish the odometry if someone has subscribed to
                if (odom_SubNumber > 0) {
                    track_state = zed->getPosition(Path, MAT_TRACKING_TYPE::PATH);
                    publishOdom(Path, pub_odom, odometry_frame_id, t);
                }

                //Note, the frame is published, but its values will only change if someone has subscribed to odom
                publishTrackedFrame(Path, transform_odom_broadcaster, odometry_transform_frame_id, t); //publish the tracked Frame

                ros::spinOnce();
                loop_rate.sleep();
            } else {

                publishTrackedFrame(Path, transform_odom_broadcaster, odometry_transform_frame_id, ros::Time::now()); //publish the tracked Frame before the sleep
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // No subscribers, we just wait
            }
        }
    } catch (...) {
		zw_cloud::quit();
        ROS_ERROR("Unknown error.");
        return 1;
    }
    
    zw_cloud::quit();
    ROS_INFO("Quitting zed_depth_stereo_wrapper_node ...\n");
    return 0;
}
