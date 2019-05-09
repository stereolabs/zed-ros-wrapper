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

// Dynamic reconfiguration
#include <zed_wrapper/ZedConfig.h>

// Services
#include <zed_wrapper/reset_tracking.h>
#include <zed_wrapper/set_pose.h>
#include <zed_wrapper/reset_odometry.h>
#include <zed_wrapper/start_svo_recording.h>
#include <zed_wrapper/stop_svo_recording.h>
#include <zed_wrapper/start_remote_stream.h>
#include <zed_wrapper/stop_remote_stream.h>
#include <zed_wrapper/set_led_status.h>
#include <zed_wrapper/toggle_led.h>

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

      protected:
        /* \brief Initialization function called by the Nodelet base class
         */
        virtual void onInit();

        /* \brief Reads parameters from the param server
         */
        void readParameters();

        /* \brief ZED camera polling thread function
         */
        void device_poll_thread_func();

        /* \brief Pointcloud publishing function
         */
        void pointcloud_thread_func();

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
         * \param pubImg : the publisher object to use (different image publishers
         * exist)
         * \param camInfoMsg : the camera_info to be published with image
         * \param imgFrameId : the id of the reference frame of the image (different
         * image frames exist)
         * \param t : the ros::Time to stamp the image
         */
        void publishImage(sl::Mat img, image_transport::CameraPublisher& pubImg, sensor_msgs::CameraInfoPtr camInfoMsg,
                          string imgFrameId, ros::Time t);

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

        /* \brief Publish a single pointCloud with a ros Publisher
         */
        void publishPointCloud();

        /* \brief Publish a fused pointCloud with a ros Publisher
         */
        void pubFusedPointCloudCallback(const ros::TimerEvent& e);

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
        bool on_set_pose(zed_wrapper::set_pose::Request& req,
                         zed_wrapper::set_pose::Response& res);

        /* \brief Service callback to start_svo_recording service
         */
        bool on_start_svo_recording(zed_wrapper::start_svo_recording::Request& req,
                                    zed_wrapper::start_svo_recording::Response& res);

        /* \brief Service callback to stop_svo_recording service
         */
        bool on_stop_svo_recording(zed_wrapper::stop_svo_recording::Request& req,
                                   zed_wrapper::stop_svo_recording::Response& res);

        /* \brief Service callback to start_remote_stream service
         */
        bool on_start_remote_stream(zed_wrapper::start_remote_stream::Request& req,
                                    zed_wrapper::start_remote_stream::Response& res);

        /* \brief Service callback to stop_remote_stream service
         */
        bool on_stop_remote_stream(zed_wrapper::stop_remote_stream::Request& req,
                                   zed_wrapper::stop_remote_stream::Response& res);

        /* \brief Service callback to set_led_status service
         */
        bool on_set_led_status(zed_wrapper::set_led_status::Request& req,
                               zed_wrapper::set_led_status::Response& res);

        /* \brief Service callback to toggle_led service
         */
        bool on_toggle_led(zed_wrapper::toggle_led::Request& req,
                           zed_wrapper::toggle_led::Response& res);

        /* \brief Utility to initialize the pose variables
         */
        bool set_pose(float xt, float yt, float zt, float rr, float pr, float yr);

        /* \brief Utility to initialize the most used transforms
         */
        void initTransforms();

        /* \brief Utility to initialize the static transform from Sensor to Base
         */
        bool getSens2BaseTransform();

        /* \brief Utility to initialize the static transform from Sensor to Camera
         */
        bool getSens2CameraTransform();

        /* \brief Utility to initialize the static transform from Camera to Base
         */
        bool getCamera2BaseTransform();

        /* \bried Start tracking
         */
        void start_tracking();

        /* \bried Start spatial mapping
         */
        void start_mapping();

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
        image_transport::CameraPublisher mPubRgb; //
        image_transport::CameraPublisher mPubRawRgb; //
        image_transport::CameraPublisher mPubLeft; //
        image_transport::CameraPublisher mPubRawLeft; //
        image_transport::CameraPublisher mPubRight; //
        image_transport::CameraPublisher mPubRawRight; //
        image_transport::CameraPublisher mPubDepth; //
        image_transport::CameraPublisher mPubConfImg; //
        image_transport::Publisher mPubStereo;
        image_transport::Publisher mPubRawStereo;


        ros::Publisher mPubConfMap; //
        ros::Publisher mPubDisparity; //
        ros::Publisher mPubCloud;
        ros::Publisher mPubFusedCloud;
        ros::Publisher mPubPose;
        ros::Publisher mPubPoseCov;
        ros::Publisher mPubOdom;
        ros::Publisher mPubOdomPath;
        ros::Publisher mPubMapPath;
        ros::Publisher mPubImu;
        ros::Publisher mPubImuRaw;

        // Timers
        ros::Timer mImuTimer;
        ros::Timer mPathTimer;
        ros::Timer mFusedPcTimer;

        // Services
        ros::ServiceServer mSrvSetInitPose;
        ros::ServiceServer mSrvResetOdometry;
        ros::ServiceServer mSrvResetTracking;
        ros::ServiceServer mSrvSvoStartRecording;
        ros::ServiceServer mSrvSvoStopRecording;
        ros::ServiceServer mSrvSvoStartStream;
        ros::ServiceServer mSrvSvoStopStream;
        ros::ServiceServer mSrvSetLedStatus;
        ros::ServiceServer mSrvToggleLed;

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

        std::string mWorldFrameId;
        std::string mMapFrameId;
        std::string mOdometryFrameId;
        std::string mBaseFrameId;
        std::string mCameraFrameId;

        std::string mRightCamFrameId;
        std::string mRightCamOptFrameId;
        std::string mLeftCamFrameId;
        std::string mLeftCamOptFrameId;
        std::string mImuFrameId;

        bool mPublishTf;
        bool mPublishMapTf;
        bool mCameraFlip;
        bool mCameraSelfCalib;

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
        std::string mRemoteStreamAddr;
        double mImuPubRate;
        bool mImuTimestampSync;
        double mPathPubRate;
        int mPathMaxCount;
        bool mVerbose;
        bool mSvoMode = false;
        double mCamMinDepth;

        bool mTrackingActivated;
        bool mMappingEnabled;
        bool mMappingActivated;
        bool mTrackingReady;
        bool mTwoDMode = false;
        double mFixedZValue = 0.0;
        bool mFixedCov = true;
        double mFixedCovValue = 1e-6;
        bool mFloorAlignment = false;
        bool mImuFusion = true;
        bool mGrabActive = false; // Indicate if camera grabbing is active (at least one topic subscribed)
        bool mColorEnhancement = true;
        sl::ERROR_CODE mConnStatus;
        sl::ERROR_CODE mGrabStatus;
        sl::TRACKING_STATE mTrackingStatus;
        bool mImuPublishing = false;
        bool mPcPublishing = false;

        int mMappingRes = 0;
        double mFusedPcPubFreq = 2.0;

        // Topic names
        std::string mRgbTopicRoot;
        std::string mRightTopicRoot;
        std::string mLeftTopicRoot;
        std::string mDepthTopicRoot;
        std::string mDisparityTopic;
        std::string mPointCloudTopicRoot;
        std::string mConfImgRoot;
        std::string mPoseTopic;
        std::string mOdometryTopic;
        std::string mImuTopicRoot;
        std::string mStereoTopicRoot;

        // Last frame time
        ros::Time mPrevFrameTimestamp;
        ros::Time mFrameTimestamp;

        //Tracking variables
        sl::Pose mLastZedPose; // Sensor to Map transform
        sl::Transform mInitialPoseSl;
        std::vector<float> mInitialBasePose;
        std::vector<geometry_msgs::PoseStamped> mOdomPath;
        std::vector<geometry_msgs::PoseStamped> mMapPath;

        // TF Transforms
        tf2::Transform mMap2OdomTransf;         // Coordinates of the odometry frame in map frame
        tf2::Transform mOdom2BaseTransf;        // Coordinates of the base in odometry frame
        tf2::Transform mMap2BaseTransf;         // Coordinates of the base in map frame
        tf2::Transform mMap2CameraTransf;       // Coordinates of the camera in base frame
        tf2::Transform mSensor2BaseTransf;      // Coordinates of the base frame in sensor frame
        tf2::Transform mSensor2CameraTransf;    // Coordinates of the camera frame in sensor frame
        tf2::Transform mCamera2BaseTransf;      // Coordinates of the base frame in camera frame

        bool mSensor2BaseTransfValid = false;
        bool mSensor2CameraTransfValid = false;
        bool mCamera2BaseTransfValid = false;

        // initialization Transform listener
        boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> mTfListener;

        // Zed object
        sl::InitParameters mZedParams;
        sl::Camera mZed;
        unsigned int mZedSerialNumber;
        int mZedUserCamModel;       // Camera model set by ROS Param
        sl::MODEL mZedRealCamModel; // Camera model requested to SDK
        unsigned int mFwVersion;

        // Dynamic Parameters
        int mCamConfidence;
        int mCamExposure;
        int mCamGain;
        double mCamMatResizeFactor;
        double mCamMaxDepth;
        bool mCamAutoExposure;
        double mPointCloudFreq;

        // flags
        bool mTriggerAutoExposure;
        bool mComputeDepth;
        bool mOpenniDepthMode; // 16 bit UC data in mm else 32F in m, for more info -> http://www.ros.org/reps/rep-0118.html
        bool mPoseSmoothing = false; // Always disabled. Enable only for AR/VR applications
        bool mSpatialMemory;
        bool mInitOdomWithPose;
        bool mResetOdom = false;
        bool mPublishPoseCovariance = true;

        // SVO recording
        bool mRecording = false;
        sl::RecordingState mRecState;
        sl::SVO_COMPRESSION_MODE mSvoComprMode;

        // Streaming
        bool mStreaming = false;

        // Mat
        int mCamWidth;
        int mCamHeight;
        int mMatWidth;
        int mMatHeight;

        // Thread Sync
        std::mutex mCloseZedMutex;
        std::mutex mCamDataMutex;
        std::mutex mPcMutex;
        std::mutex mRecMutex;
        std::mutex mPosTrkMutex;
        std::condition_variable mPcDataReadyCondVar;
        bool mPcDataReady;

        // Point cloud variables
        sl::Mat mCloud;
        sensor_msgs::PointCloud2Ptr mPointcloudMsg;
#if ((ZED_SDK_MAJOR_VERSION>2) || (ZED_SDK_MAJOR_VERSION==2 && ZED_SDK_MINOR_VERSION>=8) )
        sl::FusedPointCloud mFusedPC;
        sensor_msgs::PointCloud2Ptr mPointcloudFusedMsg;
#endif
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
