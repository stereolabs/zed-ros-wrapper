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
#include <sensor_msgs/PointCloud2.h>

#include <zed_wrapper/ZedConfig.h>
#include <zed_wrapper/reset_tracking.h>
#include <zed_wrapper/set_initial_pose.h>
#include <zed_wrapper/reset_odometry.h>

#include <mutex>
#include <thread>
#include <condition_variable>

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
        static sensor_msgs::ImagePtr imageToROSmsg(sl::Mat img, std::string frameId, ros::Time t);

      private:
        /* \brief Initialization function called by the Nodelet base class
         */
        virtual void onInit();

        /* \brief ZED camera polling thread function
         */
        void device_poll();

        /* \brief Pointcloud publishing function
         */
        void pointcloud_thread();

      protected:

        /* \brief Publish the pose of the camera in "Map" frame with a ros Publisher
         * \param poseBaseTransform : Transformation representing the camera pose from odom frame to map frame
         * \param t : the ros::Time to stamp the image
         */
        void publishPose(tf2::Transform poseBaseTransform, ros::Time t);

        /* \brief Publish the pose of the camera in "Odom" frame with a ros Publisher
         * \param odom_base_transform : Transformation representing the camera pose from base frame to odom frame
         * \param t : the ros::Time to stamp the image
         */
        void publishOdom(tf2::Transform baseToOdomTransform, ros::Time t);

        /* \brief Publish the pose of the camera in "Map" frame as a transformation
         * \param base_transform : Transformation representing the camera pose from odom frame to map frame
         * \param t : the ros::Time to stamp the image
         */
        void publishPoseFrame(tf2::Transform baseTransform, ros::Time t);

        /* \brief Publish the odometry of the camera in "Odom" frame as a transformation
         * \param base_transform : Transformation representing the camera pose from base frame to odom frame
         * \param t : the ros::Time to stamp the image
         */
        void publishOdomFrame(tf2::Transform baseToOdomTransform, ros::Time t);

        /* \brief Publish the pose of the imu in "Odom" frame as a transformation
         * \param base_transform : Transformation representing the imu pose from base frame to odom frame
         * \param t : the ros::Time to stamp the image
         */
        void publishImuFrame(tf2::Transform baseTransform);

        /* \brief Publish a sl::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use (different image publishers exist)
         * \param img_frame_id : the id of the reference frame of the image (different image frames exist)
         * \param t : the ros::Time to stamp the image
         */
        void publishImage(sl::Mat img, image_transport::Publisher& pubImg, string imgFrameId, ros::Time t);

        /* \brief Publish a sl::Mat depth image with a ros Publisher
         * \param depth : the depth image to publish
         * \param t : the ros::Time to stamp the depth image
         */
        void publishDepth(sl::Mat depth, ros::Time t);

        /* \brief Publish a sl::Mat confidence image with a ros Publisher
         * \param conf : the confidence image to publish
         * \param t : the ros::Time to stamp the depth image
         */
        void publishConf(sl::Mat conf, ros::Time t);

        /* \brief Publish a pointCloud with a ros Publisher
         * \param width : the width of the point cloud
         * \param height : the height of the point cloud
         */
        void publishPointCloud();

        /* \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param t : the ros::Time to stamp the message
         */
        void publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t);

        /* \brief Publish a sl::Mat disparity image with a ros Publisher
         * \param disparity : the disparity image to publish
         * \param t : the ros::Time to stamp the depth image
         */
        void publishDisparity(sl::Mat disparity, ros::Time t);

        /* \brief Get the information of the ZED cameras and store them in an information message
         * \param zed : the sl::zed::Camera* pointer to an instance
         * \param left_cam_info_msg : the information message to fill with the left camera informations
         * \param right_cam_info_msg : the information message to fill with the right camera informations
         * \param left_frame_id : the id of the reference frame of the left camera
         * \param right_frame_id : the id of the reference frame of the right camera
         */
        void fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr leftCamInfoMsg, sensor_msgs::CameraInfoPtr rightCamInfoMsg,
                         string leftFrameId, string rightFrameId, bool rawParam = false);

        /* \brief Callback to handle dynamic reconfigure events in ROS
         */
        void dynamicReconfCallback(zed_wrapper::ZedConfig& config, uint32_t level);

        /* \brief Callback to publish IMU raw data with a ROS publisher.
         * \param e : the ros::TimerEvent binded to the callback
         */
        void imuPubCallback(const ros::TimerEvent& e);

        /* \brief Service callback to reset_tracking service
         * Tracking pose is reinitialized to the value available in the ROS Param server
         */
        bool on_reset_tracking(zed_wrapper::reset_tracking::Request&  req,
                               zed_wrapper::reset_tracking::Response& res);

        /* \brief Service callback to reset_odometry service
         *        Odometry is reset to clear drift and odometry frame gets the latest pose
         *        from ZED tracking.
         */
        bool on_reset_odometry(zed_wrapper::reset_odometry::Request&  req,
                               zed_wrapper::reset_odometry::Response& res);

        /* \brief Service callback to set_pose service
         *        Tracking pose is set to the new values
         */
        bool on_set_pose(zed_wrapper::set_initial_pose::Request& req,
                         zed_wrapper::set_initial_pose::Response& res);

        /* \brief Utility to initialize the pose variables
         */
        void set_pose(float xt, float yt, float zt, float rr, float pr, float yr);

        /* \bried Start tracking loading the parameters from param server
         */
        void start_tracking();

        /* \bried Check if FPS and Resolution chosen by user are correct.
         *        Modifies FPS to match correct value.
         */
        void checkResolFps();

      private:
        // SDK version
        int verMajor;
        int verMinor;
        int verSubMinor;

        // ROS
        ros::NodeHandle nh;
        ros::NodeHandle nhNs;
        std::thread devicePollThread;
        std::thread mPcThread; // Point Cloud thread

        bool mStopNode;

        // Publishers
        image_transport::Publisher pubRgb;
        image_transport::Publisher pubRawRgb;
        image_transport::Publisher pubLeft;
        image_transport::Publisher pubRawLeft;
        image_transport::Publisher pubRight;
        image_transport::Publisher pubRawRight;
        image_transport::Publisher pubDepth;
        image_transport::Publisher pubConfImg;
        ros::Publisher pubConfMap;
        ros::Publisher pubDisparity;
        ros::Publisher pubCloud;
        ros::Publisher pubRgbCamInfo;
        ros::Publisher pubLeftCamInfo;
        ros::Publisher pubRightCamInfo;
        ros::Publisher pubRgbCamInfoRaw;
        ros::Publisher pubLeftCamInfoRaw;
        ros::Publisher pubRightCamInfoRaw;
        ros::Publisher pubDepthCamInfo;
        ros::Publisher pubPose;
        ros::Publisher pubOdom;
        ros::Publisher pubImu;
        ros::Publisher pubImuRaw;
        ros::Timer pubImuTimer;

        // Service
        bool trackingActivated;
        ros::ServiceServer srvSetInitPose;
        ros::ServiceServer srvResetOdometry;
        ros::ServiceServer srvResetTracking;

        // Camera info
        sensor_msgs::CameraInfoPtr rgbCamInfoMsg;
        sensor_msgs::CameraInfoPtr leftCamInfoMsg;
        sensor_msgs::CameraInfoPtr rightCamInfoMsg;
        sensor_msgs::CameraInfoPtr rgbCamInfoRawMsg;
        sensor_msgs::CameraInfoPtr leftCamInfoRawMsg;
        sensor_msgs::CameraInfoPtr rightCamInfoRawMsg;
        sensor_msgs::CameraInfoPtr depthCamInfoMsg;

        // tf
        tf2_ros::TransformBroadcaster transformPoseBroadcaster;
        tf2_ros::TransformBroadcaster transformOdomBroadcaster;
        tf2_ros::TransformBroadcaster transformImuBroadcaster;

        std::string rgbFrameId;
        std::string rgbOptFrameId;

        std::string depthFrameId;
        std::string depthOptFrameId;

        std::string disparityFrameId;
        std::string disparityOptFrameId;

        std::string confidenceFrameId;
        std::string confidenceOptFrameId;

        std::string cloudFrameId;

        std::string mapFrameId;
        std::string odometryFrameId;
        std::string baseFrameId;
        std::string rightCamFrameId;
        std::string rightCamOptFrameId;
        std::string leftCamFrameId;
        std::string leftCamOptFrameId;
        std::string imuFrameId;

        // initialization Transform listener
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        bool publishTf;
        bool cameraFlip;

        // Launch file parameters
        int resolution;
        int frameRate;
        int quality;
        int sensingMode;
        int gpuId;
        int zedId;
        int depthStabilization;
        std::string odometryDb;
        std::string svoFilepath;
        double imuPubRate;
        bool verbose;

        // IMU time
        ros::Time imuTime;

        bool poseSmoothing;
        bool spatialMemory;
        bool initOdomWithPose;

        //Tracking variables
        sl::Transform initialPoseSl;
        std::vector<float> initialTrackPose;

        tf2::Transform odomToMapTransform;
        tf2::Transform baseToOdomTransform;

        // zed object
        sl::InitParameters param;
        sl::Camera zed;
        unsigned int serial_number;
        int userCamModel; // Camera model set by ROS Param
        sl::MODEL realCamModel; // Camera model requested to SDK

        // flags
        double matResizeFactor;
        int confidence;
        int exposure;
        int gain;
        bool autoExposure;
        bool triggerAutoExposure;
        bool computeDepth;
        bool grabbing = false;
        int openniDepthMode = 0; // 16 bit UC data in mm else 32F in m, for more info http://www.ros.org/reps/rep-0118.html

        // Frame and Mat
        int camWidth;
        int camHeight;
        int matWidth;
        int matHeight;

        // Mutex
        std::mutex dataMutex;
        std::mutex mPcMutex;
        std::condition_variable mPcDataReadyCondVar;
        bool mPcDataReady;

        // Point cloud variables
        sl::Mat cloud;
        sensor_msgs::PointCloud2 mPointcloudMsg;
        string pointCloudFrameId = "";
        ros::Time pointCloudTime;

        // Dynamic reconfigure
        boost::shared_ptr<dynamic_reconfigure::Server<zed_wrapper::ZedConfig>> server;

        // Coordinate Changing indices and signs
        unsigned int xIdx, yIdx, zIdx;
        int xSign, ySign, zSign;

    }; // class ZEDROSWrapperNodelet
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_wrapper::ZEDWrapperNodelet, nodelet::Nodelet)

#endif // ZED_WRAPPER_NODELET_H
