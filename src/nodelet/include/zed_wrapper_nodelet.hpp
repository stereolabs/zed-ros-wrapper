#ifndef ZED_WRAPPER_NODELET_H
#define ZED_WRAPPER_NODELET_H

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

/****************************************************************************************************
 ** This sample is a wrapper for the ZED library in order to use the ZED Camera with ROS.          **
 ** A set of parameters can be specified in the launch file.                                       **
 ****************************************************************************************************/

#include <sl/Camera.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <zed_wrapper/ZedConfig.h>
#include <zed_wrapper/reset_tracking.h>
#include <zed_wrapper/set_pose.h>

#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>

#include <mutex>

using namespace std;

namespace zed_wrapper {
    
    class ZEDWrapperNodelet : public nodelet::Nodelet {
        
    public:
        /* \brief Default constructor
         */
        ZEDWrapperNodelet();

        /* \brief \ref ZEDWrapperNodelet destructor
         */
        virtual ~ZEDWrapperNodelet();

        /* \brief Image to ros message conversion
         * \param img : the image to publish
         * \param encodingType : the sensor_msgs::image_encodings encoding type
         * \param frameId : the id of the reference frame of the image
         * \param t : the ros::Time to stamp the image
         */
        static sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t);

    private:
        /* \brief Initialization function called by the Nodelet base class
         */
        virtual void onInit();

        /* \brief ZED camera polling thread function
         */
        void device_poll();        

    protected:       

        /* \brief Publish the pose of the camera with a ros Publisher
         * \param base_transform : Transformation representing the camera pose from base frame
         * \param t : the ros::Time to stamp the image
         */
        void publishOdom(tf2::Transform base_transform, ros::Time t);

        /* \brief Publish the pose of the camera as a transformation
         * \param base_transform : Transformation representing the camera pose from base frame
         * \param t : the ros::Time to stamp the image
         */
        void publishTrackedFrame(tf2::Transform base_transform, ros::Time t);

        /* \brief Publish the pose of the imu as a transformation
         * \param base_transform : Transformation representing the imu pose from camera frame
         * \param t : the ros::Time to stamp the image
         */
        void publishImuFrame(tf2::Transform base_transform, ros::Time t);

        /* \brief Publish a cv::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use (different image publishers exist)
         * \param img_frame_id : the id of the reference frame of the image (different image frames exist)
         * \param t : the ros::Time to stamp the image
         */
        void publishImage(cv::Mat img, image_transport::Publisher &pub_img, string img_frame_id, ros::Time t);

        /* \brief Publish a cv::Mat depth image with a ros Publisher
         * \param depth : the depth image to publish
         * \param t : the ros::Time to stamp the depth image
         */
        void publishDepth(cv::Mat depth, ros::Time t);

        /* \brief Publish a cv::Mat confidence image with a ros Publisher
         * \param conf : the confidence image to publish
         * \param t : the ros::Time to stamp the depth image
         */
        void publishConf(cv::Mat conf, ros::Time t);

        /* \brief Publish a pointCloud with a ros Publisher
         * \param width : the width of the point cloud
         * \param height : the height of the point cloud
         */
        void publishPointCloud(int width, int height);

        /* \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param t : the ros::Time to stamp the message
         */
        void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t);

        /* \brief Publish a cv::Mat disparity image with a ros Publisher
         * \param disparity : the disparity image to publish
         * \param t : the ros::Time to stamp the depth image
         */
        void publishDisparity(cv::Mat disparity, ros::Time t);

        /* \brief Get the information of the ZED cameras and store them in an information message
         * \param zed : the sl::zed::Camera* pointer to an instance
         * \param left_cam_info_msg : the information message to fill with the left camera informations
         * \param right_cam_info_msg : the information message to fill with the right camera informations
         * \param left_frame_id : the id of the reference frame of the left camera
         * \param right_frame_id : the id of the reference frame of the right camera
         */
        void fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,
                string left_frame_id, string right_frame_id, bool raw_param = false);        

        /* \brief Callback to handle dynamic reconfigure events in ROS
         */
        void dynamicReconfCallback(zed_wrapper::ZedConfig &config, uint32_t level);

        /* \brief Callback to publish IMU raw data with a ROS publisher.
         * \param e : the ros::TimerEvent binded to the callback
         */
        void imuPubCallback(const ros::TimerEvent & e);

        /* \brief Service callback to reset_tracking service
         * Tracking pose is reinitialized to the value available in the ROS Param server
         */
        bool on_reset_tracking(zed_wrapper::reset_tracking::Request  &req,
                               zed_wrapper::reset_tracking::Response &res);

        /* \brief Service callback to set_pose service
         * Tracking pose is set to the new values
         */
        bool on_set_pose(zed_wrapper::set_pose::Request &req,
                         zed_wrapper::set_pose::Response &res);

        /* \brief Utility to initialize the pose variables
         */
        void set_pose( float xt, float yt, float zt, float rr, float pr, float yr);

        /* \bried Start tracking loading the parameters from param server
         */
        void start_tracking();

    private:
        // SDK version
        int ver_major;
        int ver_minor;
        int ver_sub_minor;

        // ROS
        ros::NodeHandle nh;
        ros::NodeHandle nh_ns;
        boost::shared_ptr<boost::thread> device_poll_thread;

        // Publishers
        image_transport::Publisher pub_rgb;
        image_transport::Publisher pub_raw_rgb;
        image_transport::Publisher pub_left;
        image_transport::Publisher pub_raw_left;
        image_transport::Publisher pub_right;
        image_transport::Publisher pub_raw_right;
        image_transport::Publisher pub_depth;
        image_transport::Publisher pub_conf_img;
        ros::Publisher pub_conf_map;
        ros::Publisher pub_disparity;
        ros::Publisher pub_cloud;
        ros::Publisher pub_rgb_cam_info;
        ros::Publisher pub_left_cam_info;
        ros::Publisher pub_right_cam_info;
        ros::Publisher pub_rgb_cam_info_raw;
        ros::Publisher pub_left_cam_info_raw;
        ros::Publisher pub_right_cam_info_raw;
        ros::Publisher pub_depth_cam_info;
        ros::Publisher pub_odom;
        ros::Publisher pub_imu;
        ros::Timer  pub_imu_timer;

        // Service
        bool tracking_activated;
        ros::ServiceServer srv_reset_tracking;
        ros::ServiceServer srv_set_pose;

        // Camera info
        sensor_msgs::CameraInfoPtr rgb_cam_info_msg;
        sensor_msgs::CameraInfoPtr left_cam_info_msg;
        sensor_msgs::CameraInfoPtr right_cam_info_msg;
        sensor_msgs::CameraInfoPtr rgb_cam_info_raw_msg;
        sensor_msgs::CameraInfoPtr left_cam_info_raw_msg;
        sensor_msgs::CameraInfoPtr right_cam_info_raw_msg;
        sensor_msgs::CameraInfoPtr depth_cam_info_msg;

        // tf
        tf2_ros::TransformBroadcaster transform_odom_broadcaster;
        tf2_ros::TransformBroadcaster transform_imu_broadcaster;
        std::string rgb_frame_id;
        std::string rgb_opt_frame_id;
        
        std::string depth_frame_id;
        std::string depth_opt_frame_id;

        std::string disparity_frame_id;
        std::string disparity_opt_frame_id;

        std::string confidence_frame_id;
        std::string confidence_opt_frame_id;

        std::string cloud_frame_id;

        std::string odometry_frame_id;
        std::string base_frame_id;
        std::string right_cam_frame_id;
        std::string right_cam_opt_frame_id;
        std::string left_cam_frame_id;
        std::string left_cam_opt_frame_id;
        std::string imu_frame_id;

        // initialization Transform listener
        boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> tf_listener;
        bool publish_tf;

        // Launch file parameters
        int resolution;
        int quality;
        int sensing_mode;
        int rate;
        int gpu_id;
        int zed_id;
        int depth_stabilization;
        std::string odometry_DB;
        std::string svo_filepath;
        double imu_pub_rate;

        bool pose_smoothing;
        bool spatial_memory;

        //Tracking variables
        sl::Pose pose;
        std::vector<float> initial_track_pose;
        tf2::Transform base_transform;
        sl::Transform initial_pose_sl;

        // zed object
        sl::InitParameters param;
        sl::Camera zed;
        unsigned int serial_number;
        int user_cam_model; // Camera model set by ROS Param 

        // flags
        double mat_resize_factor;
        int confidence;
        int exposure;
        int gain;
        bool autoExposure;
        bool triggerAutoExposure;
        bool computeDepth;
        bool grabbing = false;
        int openniDepthMode = 0; // 16 bit UC data in mm else 32F in m, for more info http://www.ros.org/reps/rep-0118.html

        // Frame and Mat
        int cam_width;
        int cam_height;
        int mat_width;
        int mat_height;
        cv::Mat leftImRGB;
        cv::Mat rightImRGB;
        cv::Mat confImRGB;
        cv::Mat confMapFloat;

        // Mutex
        std::mutex dataMutex;

        // Point cloud variables
        sl::Mat cloud;
        string point_cloud_frame_id = "";
        ros::Time point_cloud_time;

        // Dynamic reconfigure
        boost::shared_ptr<dynamic_reconfigure::Server<zed_wrapper::ZedConfig>> server;

    }; // class ZEDROSWrapperNodelet
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_wrapper::ZEDWrapperNodelet, nodelet::Nodelet);

#endif // ZED_WRAPPER_NODELET_H
