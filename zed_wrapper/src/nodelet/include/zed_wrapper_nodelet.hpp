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
#include "sl_tools.h"

#include <sl/Camera.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <zed_wrapper/ZedConfig.h>
#include <zed_wrapper/reset_tracking.h>
#include <zed_wrapper/set_initial_pose.h>
#include <zed_wrapper/reset_odometry.h>

#include <memory>
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

      private:
        /* \brief Initialization function called by the Nodelet base class
         */
        virtual void onInit();

        /* \brief ZED camera polling thread function
         */
        void device_poll_thread_func();

        /* \brief Pointcloud publishing function
         */
        void pointcloud_thread_func();

      protected:

        /* \brief Publish the pose of the camera in "Map" frame with a ros Publisher
         * \param t : the ros::Time to stamp the image
         */
        void publishPose(ros::Time t);

        /* \brief Publish the pose of the camera in "Odom" frame with a ros Publisher
         * \param base2odomTransf : Transformation representing the camera pose
         * from base frame to odom frame
         * \param slPose : latest odom pose from ZED SDK
         * \param t : the ros::Time to stamp the image
         */
        void publishOdom(tf2::Transform odom2baseTransf, sl::Pose& slPose, ros::Time t);

        /* \brief Publish the pose of the camera in "Map" frame as a transformation
         * \param baseTransform : Transformation representing the camera pose from
         * odom frame to map frame
         * \param t : the ros::Time to stamp the image
         */
        void publishPoseFrame(tf2::Transform baseTransform, ros::Time t);

        /* \brief Publish the odometry of the camera in "Odom" frame as a
         * transformation
         * \param odomTransf : Transformation representing the camera pose from
         * base frame to odom frame
         * \param t : the ros::Time to stamp the image
         */
        void publishOdomFrame(tf2::Transform odomTransf, ros::Time t);

        /* \brief Publish the pose of the imu in "Odom" frame as a transformation
         * \param imuTransform : Transformation representing the imu pose from base
         * frame to odom framevoid
         * \param t : the ros::Time to stamp the image
         */
        void publishImuFrame(tf2::Transform imuTransform, ros::Time t);

        /* \brief Publish a sl::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use (different image publishers
         * exist)
         * \param img_frame_id : the id of the reference frame of the image (different
         * image frames exist)
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
         */
        void publishPointCloud();

        /* \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param t : the ros::Time to stamp the message
         */
        void publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg,
                            ros::Publisher pubCamInfo, ros::Time t);

        /* \brief Publish a sl::Mat disparity image with a ros Publisher
         * \param disparity : the disparity image to publish
         * \param t : the ros::Time to stamp the depth image
         */
        void publishDisparity(sl::Mat disparity, ros::Time t);

        /* \brief Get the information of the ZED cameras and store them in an
         * information message
         * \param zed : the sl::zed::Camera* pointer to an instance
         * \param left_cam_info_msg : the information message to fill with the left
         * camera informations
         * \param right_cam_info_msg : the information message to fill with the right
         * camera informations
         * \param left_frame_id : the id of the reference frame of the left camera
         * \param right_frame_id : the id of the reference frame of the right camera
         */
        void fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr leftCamInfoMsg,
                         sensor_msgs::CameraInfoPtr rightCamInfoMsg,
                         string leftFrameId, string rightFrameId,
                         bool rawParam = false);

        /* \bried Check if FPS and Resolution chosen by user are correct.
         *        Modifies FPS to match correct value.
         */
        void checkResolFps();

        /* \brief Callback to handle dynamic reconfigure events in ROS
         */
        void dynamicReconfCallback(zed_wrapper::ZedConfig& config, uint32_t level);

        /* \brief Callback to publish Path data with a ROS publisher.
         * \param e : the ros::TimerEvent binded to the callback
         */
        void pathPubCallback(const ros::TimerEvent& e);
        
        /* \brief Callback to publish IMU raw data with a ROS publisher.
         * \param e : the ros::TimerEvent binded to the callback
         */
        void imuPubCallback(const ros::TimerEvent& e);

        /* \brief Callback to update node diagnostic status
         * \param stat : node status
         */
        void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

        /* \brief Service callback to reset_tracking service
         * Tracking pose is reinitialized to the value available in the ROS Param
         * server
         */
        bool on_reset_tracking(zed_wrapper::reset_tracking::Request&  req,
                               zed_wrapper::reset_tracking::Response& res);

        /* \brief Service callback to reset_odometry service
         *        Odometry is reset to clear drift and odometry frame gets the latest
         * pose
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

        /* \brief Utility to initialize the most used transforms
         */
        void initTransforms();

        /* \bried Start tracking loading the parameters from param server
         */
        void start_tracking();



      private:
        // SDK version
        int mVerMajor;
        int mVerMinor;
        int mVerSubMinor;

        // ROS
        ros::NodeHandle mNh;
        ros::NodeHandle mNhNs;
        std::thread mDevicePollThread;
        std::thread mPcThread; // Point Cloud thread

        bool mStopNode;

        // Publishers
        image_transport::Publisher mPubRgb; //
        image_transport::Publisher mPubRawRgb; //
        image_transport::Publisher mPubLeft; //
        image_transport::Publisher mPubRawLeft; //
        image_transport::Publisher mPubRight; //
        image_transport::Publisher mPubRawRight; //
        image_transport::Publisher mPubDepth; //
        image_transport::Publisher mPubConfImg; //

        ros::Publisher mPubConfMap; //
        ros::Publisher mPubDisparity; //
        ros::Publisher mPubCloud;
        ros::Publisher mPubRgbCamInfo; //
        ros::Publisher mPubLeftCamInfo; //
        ros::Publisher mPubRightCamInfo; //
        ros::Publisher mPubRgbCamInfoRaw; //
        ros::Publisher mPubLeftCamInfoRaw; //
        ros::Publisher mPubRightCamInfoRaw; //
        ros::Publisher mPubDepthCamInfo; //
        ros::Publisher mPubPose;
        ros::Publisher mPubPoseCov;
        ros::Publisher mPubOdom;
        ros::Publisher mPubOdomPath;
        ros::Publisher mPubMapPath;
        ros::Publisher mPubImu;
        ros::Publisher mPubImuRaw;
        //ros::Publisher mPubClock;

        // Timers
        ros::Timer mImuTimer;
        ros::Timer mPathTimer;

        // Services
        ros::ServiceServer mSrvSetInitPose;
        ros::ServiceServer mSrvResetOdometry;
        ros::ServiceServer mSrvResetTracking;


        // Camera info
        sensor_msgs::CameraInfoPtr mRgbCamInfoMsg;
        sensor_msgs::CameraInfoPtr mLeftCamInfoMsg;
        sensor_msgs::CameraInfoPtr mRightCamInfoMsg;
        sensor_msgs::CameraInfoPtr mRgbCamInfoRawMsg;
        sensor_msgs::CameraInfoPtr mLeftCamInfoRawMsg;
        sensor_msgs::CameraInfoPtr mRightCamInfoRawMsg;
        sensor_msgs::CameraInfoPtr mDepthCamInfoMsg;

        // ROS TF
        tf2_ros::TransformBroadcaster mTransformPoseBroadcaster;
        tf2_ros::TransformBroadcaster mTransformOdomBroadcaster;
        tf2_ros::TransformBroadcaster mTransformImuBroadcaster;

        std::string mRgbFrameId;
        std::string mRgbOptFrameId;

        std::string mDepthFrameId;
        std::string mDepthOptFrameId;

        std::string mDisparityFrameId;
        std::string mDisparityOptFrameId;

        std::string mConfidenceFrameId;
        std::string mConfidenceOptFrameId;

        std::string mCloudFrameId;
        std::string mPointCloudFrameId;

        std::string mMapFrameId;
        std::string mOdometryFrameId;
        std::string mBaseFrameId;
        std::string mCameraFrameId;

        std::string mRightCamFrameId;
        std::string mRightCamOptFrameId;
        std::string mLeftCamFrameId;
        std::string mLeftCamOptFrameId;
        std::string mImuFrameId;

        // initialization Transform listener
        boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> mTfListener;

        bool mPublishTf;
        bool mPublishMapTf;
        bool mCameraFlip;

        // Launch file parameters
        int mCamResol;
        int mCamFrameRate;
        int mCamQuality;
        int mCamSensingMode;
        int mGpuId;
        int mZedId;
        int mDepthStabilization;
        std::string mOdometryDb;
        std::string mSvoFilepath;
        double mImuPubRate;
        bool mImuTimestampSync;
        double mPathPubRate;
        int mPathMaxCount;
        bool mVerbose;
        bool mSvoMode = false;

        bool mTrackingActivated;
        bool mTrackingReady;
        bool mFloorAlignment = false;
        bool mGrabActive = false; // Indicate if camera grabbing is active (at least one topic subscribed)
        sl::ERROR_CODE mConnStatus;
        sl::ERROR_CODE mGrabStatus;
        sl::TRACKING_STATE mTrackingStatus;
        bool mImuPublishing = false;
        bool mPcPublishing = false;


        // Last frame time
        ros::Time mPrevFrameTimestamp;
        ros::Time mFrameTimestamp;

        //Tracking variables
        sl::Pose mLastZedPose; // Sensor to Map transform
        sl::Transform mInitialPoseSl;
        std::vector<float> mInitialTrackPose;
        std::vector<geometry_msgs::PoseStamped> mOdomPath;
        std::vector<geometry_msgs::PoseStamped> mMapPath;

        // TF Transforms
        tf2::Transform mMap2OdomTransf;     // Coordinates of the odometry frame in map frame
        tf2::Transform mOdom2BaseTransf;    // Coordinates of the base in odometry frame
        tf2::Transform mMap2BaseTransf;     // Coordinates of the base in base frame
        tf2::Transform mSensor2BaseTransf;  // Coordinates of the base frame in sensor frame

        // Zed object
        sl::InitParameters mZedParams;
        sl::Camera mZed;
        unsigned int mZedSerialNumber;
        int mZedUserCamModel;       // Camera model set by ROS Param
        sl::MODEL mZedRealCamModel; // Camera model requested to SDK

        // Dynamic Parameters
        int mCamConfidence;
        int mCamExposure;
        int mCamGain;
        double mCamMatResizeFactor;
        double mCamMaxDepth;
        bool mCamAutoExposure;

        // flags
        bool mTriggerAutoExposure;
        bool mComputeDepth;
        bool mOpenniDepthMode; // 16 bit UC data in mm else 32F in m, for more info -> http://www.ros.org/reps/rep-0118.html
        bool mPoseSmoothing;
        bool mSpatialMemory;
        bool mInitOdomWithPose;
        bool mResetOdom = false;
        bool mPublishPoseCovariance = true;

        // Mat
        int mCamWidth;
        int mCamHeight;
        int mMatWidth;
        int mMatHeight;

        // Thread Sync
        std::mutex mCloseZedMutex;
        std::mutex mCamDataMutex;
        std::mutex mPcMutex;
        std::condition_variable mPcDataReadyCondVar;
        bool mPcDataReady;

        // Point cloud variables
        sl::Mat mCloud;
        sensor_msgs::PointCloud2Ptr mPointcloudMsg;
        ros::Time mPointCloudTime;

        // Dynamic reconfigure
        boost::shared_ptr<dynamic_reconfigure::Server<zed_wrapper::ZedConfig>> mDynRecServer;

        // Coordinate Changing indices and signs
        int mIdxX, mIdxY, mIdxZ;
        int mSignX, mSignY, mSignZ;

        // Diagnostic
        std::unique_ptr<sl_tools::CSmartMean> mElabPeriodMean_sec;
        std::unique_ptr<sl_tools::CSmartMean> mGrabPeriodMean_usec;
        std::unique_ptr<sl_tools::CSmartMean> mPcPeriodMean_usec;
        std::unique_ptr<sl_tools::CSmartMean> mImuPeriodMean_usec;

        diagnostic_updater::Updater mDiagUpdater; // Diagnostic Updater

    }; // class ZEDROSWrapperNodelet
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_wrapper::ZEDWrapperNodelet, nodelet::Nodelet)

#endif // ZED_WRAPPER_NODELET_H
