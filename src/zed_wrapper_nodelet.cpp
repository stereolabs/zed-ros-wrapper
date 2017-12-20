///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
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

#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
#include <thread>
#include <chrono>
#include <memory>
#include <sys/stat.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <zed_wrapper/ZedConfig.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <boost/make_shared.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sl/Camera.hpp>

using namespace std;

namespace zed_wrapper {

    int checkCameraReady(unsigned int serial_number) {
        int id = -1;
        auto f = sl::Camera::getDeviceList();
        for (auto &it : f)
            if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
                id = it.id;
        return id;
    }

    sl::DeviceProperties getZEDFromSN(unsigned int serial_number) {
        sl::DeviceProperties prop;
        auto f = sl::Camera::getDeviceList();
        for (auto &it : f) {
            if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
                prop = it;
        }
        return prop;
    }

    class ZEDWrapperNodelet : public nodelet::Nodelet {
        ros::NodeHandle nh;
        ros::NodeHandle nh_ns;
        boost::shared_ptr<boost::thread> device_poll_thread;
        image_transport::Publisher pub_rgb;
        image_transport::Publisher pub_raw_rgb;
        image_transport::Publisher pub_left;
        image_transport::Publisher pub_raw_left;
        image_transport::Publisher pub_right;
        image_transport::Publisher pub_raw_right;
        image_transport::Publisher pub_depth;
        ros::Publisher pub_cloud;
        ros::Publisher pub_rgb_cam_info;
        ros::Publisher pub_left_cam_info;
        ros::Publisher pub_right_cam_info;
        ros::Publisher pub_rgb_cam_info_raw;
        ros::Publisher pub_left_cam_info_raw;
        ros::Publisher pub_right_cam_info_raw;
        ros::Publisher pub_depth_cam_info;
        ros::Publisher pub_odom;

        // tf
        tf2_ros::TransformBroadcaster transform_odom_broadcaster;
        std::string left_frame_id;
        std::string right_frame_id;
        std::string rgb_frame_id;
        std::string depth_frame_id;
        std::string cloud_frame_id;
        std::string odometry_frame_id;
        std::string base_frame_id;
        std::string camera_frame_id;
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

        //Tracking variables
        sl::Pose pose;
        tf2::Transform base_transform;

        // zed object
        sl::InitParameters param;
        sl::Camera zed;
        unsigned int serial_number;

        // flags
        int confidence;
        bool computeDepth;
        bool grabbing = false;
        int openniDepthMode = 0; // 16 bit UC data in mm else 32F in m, for more info http://www.ros.org/reps/rep-0118.html

        // Point cloud variables
        sl::Mat cloud;
        string point_cloud_frame_id = "";
        ros::Time point_cloud_time;

        ~ZEDWrapperNodelet() {
            device_poll_thread.get()->join();
        }

        /* \brief Convert an sl:Mat to a cv::Mat
         * \param mat : the sl::Mat to convert
         */
        cv::Mat toCVMat(sl::Mat &mat) {
            if (mat.getMemoryType() == sl::MEM_GPU)
                mat.updateCPUfromGPU();

            int cvType;
            switch (mat.getDataType()) {
                case sl::MAT_TYPE_32F_C1:
                    cvType = CV_32FC1;
                    break;
                case sl::MAT_TYPE_32F_C2:
                    cvType = CV_32FC2;
                    break;
                case sl::MAT_TYPE_32F_C3:
                    cvType = CV_32FC3;
                    break;
                case sl::MAT_TYPE_32F_C4:
                    cvType = CV_32FC4;
                    break;
                case sl::MAT_TYPE_8U_C1:
                    cvType = CV_8UC1;
                    break;
                case sl::MAT_TYPE_8U_C2:
                    cvType = CV_8UC2;
                    break;
                case sl::MAT_TYPE_8U_C3:
                    cvType = CV_8UC3;
                    break;
                case sl::MAT_TYPE_8U_C4:
                    cvType = CV_8UC4;
                    break;
            }
            return cv::Mat((int) mat.getHeight(), (int) mat.getWidth(), cvType, mat.getPtr<sl::uchar1>(sl::MEM_CPU), mat.getStepBytes(sl::MEM_CPU));
        }

        cv::Mat convertRodrigues(sl::float3 r) {
            double theta = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
            cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

            if (theta < DBL_EPSILON) {
                return R;
            } else {
                double c = cos(theta);
                double s = sin(theta);
                double c1 = 1. - c;
                double itheta = theta ? 1. / theta : 0.;

                r *= itheta;

                cv::Mat rrt = cv::Mat::eye(3, 3, CV_32F);
                float* p = (float*) rrt.data;
                p[0] = r.x * r.x;
                p[1] = r.x * r.y;
                p[2] = r.x * r.z;
                p[3] = r.x * r.y;
                p[4] = r.y * r.y;
                p[5] = r.y * r.z;
                p[6] = r.x * r.z;
                p[7] = r.y * r.z;
                p[8] = r.z * r.z;

                cv::Mat r_x = cv::Mat::eye(3, 3, CV_32F);
                p = (float*) r_x.data;
                p[0] = 0;
                p[1] = -r.z;
                p[2] = r.y;
                p[3] = r.z;
                p[4] = 0;
                p[5] = -r.x;
                p[6] = -r.y;
                p[7] = r.x;
                p[8] = 0;

                // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
                R = c * cv::Mat::eye(3, 3, CV_32F) + c1 * rrt + s*r_x;
            }
            return R;
        }

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
        sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t) {
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
                for (unsigned int i = 0; i < img.rows; i++) {
                    memcpy(rosData, opencvData, imgMessage.step);
                    rosData += imgMessage.step;
                    opencvData += img.step;
                }
            }
            return ptr;
        }

        /* \brief Publish the pose of the camera with a ros Publisher
         * \param base_transform : Transformation representing the camera pose from base frame
         * \param pub_odom : the publisher object to use
         * \param odom_frame_id : the id of the reference frame of the pose
         * \param t : the ros::Time to stamp the image
         */
        void publishOdom(tf2::Transform base_transform, ros::Publisher &pub_odom, string odom_frame_id, ros::Time t) {
            nav_msgs::Odometry odom;
            odom.header.stamp = t;
            odom.header.frame_id = odom_frame_id; // odom_frame
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

        /* \brief Publish the pose of the camera as a transformation
         * \param base_transform : Transformation representing the camera pose from base frame
         * \param trans_br : the TransformBroadcaster object to use
         * \param odometry_transform_frame_id : the id of the transformation
         * \param t : the ros::Time to stamp the image
         */
        void publishTrackedFrame(tf2::Transform base_transform, tf2_ros::TransformBroadcaster &trans_br, string odometry_transform_frame_id, ros::Time t) {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = odometry_frame_id;
            transformStamped.child_frame_id = odometry_transform_frame_id;
            // conversion from Tranform to message
            transformStamped.transform = tf2::toMsg(base_transform);
            // Publish transformation
            trans_br.sendTransform(transformStamped);
        }

        /* \brief Publish a cv::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use
         * \param img_frame_id : the id of the reference frame of the image
         * \param t : the ros::Time to stamp the image
         */
        void publishImage(cv::Mat img, image_transport::Publisher &pub_img, string img_frame_id, ros::Time t) {
            pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::BGR8, img_frame_id, t));
        }

        /* \brief Publish a cv::Mat depth image with a ros Publisher
         * \param depth : the depth image to publish
         * \param pub_depth : the publisher object to use
         * \param depth_frame_id : the id of the reference frame of the depth image
         * \param t : the ros::Time to stamp the depth image
         */
        void publishDepth(cv::Mat depth, image_transport::Publisher &pub_depth, string depth_frame_id, ros::Time t) {
            string encoding;
            if (openniDepthMode) {
                depth *= 1000.0f;
                depth.convertTo(depth, CV_16UC1); // in mm, rounded
                encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            } else {
                encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            }
            pub_depth.publish(imageToROSmsg(depth, encoding, depth_frame_id, t));
        }

        /* \brief Publish a pointCloud with a ros Publisher
         * \param width : the width of the point cloud
         * \param height : the height of the point cloud
         * \param pub_cloud : the publisher object to use
         */
        void publishPointCloud(int width, int height, ros::Publisher &pub_cloud) {
            pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
            point_cloud.width = width;
            point_cloud.height = height;
            int size = width*height;
            point_cloud.points.resize(size);

            sl::Vector4<float>* cpu_cloud = cloud.getPtr<sl::float4>();
            for (int i = 0; i < size; i++) {
                point_cloud.points[i].x = cpu_cloud[i][2];
                point_cloud.points[i].y = -cpu_cloud[i][0];
                point_cloud.points[i].z = -cpu_cloud[i][1];
                point_cloud.points[i].rgb = cpu_cloud[i][3];
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
        void fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,
                string left_frame_id, string right_frame_id, bool raw_param = false) {

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
                cv::Mat R_ = convertRodrigues(zedParam.R);
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

        void callback(zed_wrapper::ZedConfig &config, uint32_t level) {
            NODELET_INFO("Reconfigure confidence : %d", config.confidence);
            confidence = config.confidence;
        }

        void device_poll() {
            ros::Rate loop_rate(rate);
            ros::Time old_t = ros::Time::now();
            sl::ERROR_CODE grab_status;
            bool tracking_activated = false;

            // Get the parameters of the ZED images
            int width = zed.getResolution().width;
            int height = zed.getResolution().height;
            NODELET_DEBUG_STREAM("Image size : " << width << "x" << height);

            cv::Size cvSize(width, height);
            cv::Mat leftImRGB(cvSize, CV_8UC3);
            cv::Mat rightImRGB(cvSize, CV_8UC3);

            // Create and fill the camera information messages
            sensor_msgs::CameraInfoPtr rgb_cam_info_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr rgb_cam_info_raw_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr left_cam_info_raw_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr right_cam_info_raw_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
            fillCamInfo(zed, left_cam_info_msg, right_cam_info_msg, left_frame_id, right_frame_id);
            fillCamInfo(zed, left_cam_info_raw_msg, right_cam_info_raw_msg, left_frame_id, right_frame_id, true);
            rgb_cam_info_msg = depth_cam_info_msg = left_cam_info_msg; // the reference camera is the Left one (next to the ZED logo)
            rgb_cam_info_raw_msg = left_cam_info_raw_msg;


            sl::RuntimeParameters runParams;
            runParams.sensing_mode = static_cast<sl::SENSING_MODE> (sensing_mode);

            sl::TrackingParameters trackParams;
            trackParams.area_file_path = odometry_DB.c_str();


            sl::Mat leftZEDMat, rightZEDMat, depthZEDMat;
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
                int cloud_SubNumber = pub_cloud.getNumSubscribers();
                int odom_SubNumber = pub_odom.getNumSubscribers();
                bool runLoop = (rgb_SubNumber + rgb_raw_SubNumber + left_SubNumber + left_raw_SubNumber + right_SubNumber + right_raw_SubNumber + depth_SubNumber + cloud_SubNumber + odom_SubNumber) > 0;

                runParams.enable_point_cloud = false;
                if (cloud_SubNumber > 0)
                    runParams.enable_point_cloud = true;
                // Run the loop only if there is some subscribers
                if (runLoop) {
                    if ((depth_stabilization || odom_SubNumber > 0) && !tracking_activated) { //Start the tracking
                        if (odometry_DB != "" && !file_exist(odometry_DB)) {
                            odometry_DB = "";
                            NODELET_WARN("odometry_DB path doesn't exist or is unreachable.");
                        }
                        zed.enableTracking(trackParams);
                        tracking_activated = true;
                    } else if (!depth_stabilization && odom_SubNumber == 0 && tracking_activated) { //Stop the tracking
                        zed.disableTracking();
                        tracking_activated = false;
                    }
                    computeDepth = (depth_SubNumber + cloud_SubNumber + odom_SubNumber) > 0; // Detect if one of the subscriber need to have the depth information
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
                        } else NODELET_INFO_ONCE(toString(grab_status));

                        std::this_thread::sleep_for(std::chrono::milliseconds(2));

                        if ((t - old_t).toSec() > 5) {
                            zed.close();

                            NODELET_INFO("Re-opening the ZED");
                            sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
                            while (err != sl::SUCCESS) {
                                int id = checkCameraReady(serial_number);
                                if (id > 0) {
                                    param.camera_linux_id = id;
                                    err = zed.open(param); // Try to initialize the ZED
                                    NODELET_INFO_STREAM(toString(err));
                                } else NODELET_INFO("Waiting for the ZED to be re-connected");
                                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                            }
                            tracking_activated = false;
                            if (depth_stabilization || odom_SubNumber > 0) { //Start the tracking
                                if (odometry_DB != "" && !file_exist(odometry_DB)) {
                                    odometry_DB = "";
                                    NODELET_WARN("odometry_DB path doesn't exist or is unreachable.");
                                }
                                zed.enableTracking(trackParams);
                                tracking_activated = true;
                            }
                        }
                        continue;
                    }

                    old_t = ros::Time::now();

                    // Publish the left == rgb image if someone has subscribed to
                    if (left_SubNumber > 0 || rgb_SubNumber > 0) {
                        // Retrieve RGBA Left image
                        zed.retrieveImage(leftZEDMat, sl::VIEW_LEFT);
                        cv::cvtColor(toCVMat(leftZEDMat), leftImRGB, CV_RGBA2RGB);
                        if (left_SubNumber > 0) {
                            publishCamInfo(left_cam_info_msg, pub_left_cam_info, t);
                            publishImage(leftImRGB, pub_left, left_frame_id, t);
                        }
                        if (rgb_SubNumber > 0) {
                            publishCamInfo(rgb_cam_info_msg, pub_rgb_cam_info, t);
                            publishImage(leftImRGB, pub_rgb, rgb_frame_id, t); // rgb is the left image
                        }
                    }

                    // Publish the left_raw == rgb_raw image if someone has subscribed to
                    if (left_raw_SubNumber > 0 || rgb_raw_SubNumber > 0) {
                        // Retrieve RGBA Left image
                        zed.retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED);
                        cv::cvtColor(toCVMat(leftZEDMat), leftImRGB, CV_RGBA2RGB);
                        if (left_raw_SubNumber > 0) {
                            publishCamInfo(left_cam_info_raw_msg, pub_left_cam_info_raw, t);
                            publishImage(leftImRGB, pub_raw_left, left_frame_id, t);
                        }
                        if (rgb_raw_SubNumber > 0) {
                            publishCamInfo(rgb_cam_info_raw_msg, pub_rgb_cam_info_raw, t);
                            publishImage(leftImRGB, pub_raw_rgb, rgb_frame_id, t);
                        }
                    }

                    // Publish the right image if someone has subscribed to
                    if (right_SubNumber > 0) {
                        // Retrieve RGBA Right image
                        zed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT);
                        cv::cvtColor(toCVMat(rightZEDMat), rightImRGB, CV_RGBA2RGB);
                        publishCamInfo(right_cam_info_msg, pub_right_cam_info, t);
                        publishImage(rightImRGB, pub_right, right_frame_id, t);
                    }

                    // Publish the right image if someone has subscribed to
                    if (right_raw_SubNumber > 0) {
                        // Retrieve RGBA Right image
                        zed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED);
                        cv::cvtColor(toCVMat(rightZEDMat), rightImRGB, CV_RGBA2RGB);
                        publishCamInfo(right_cam_info_raw_msg, pub_right_cam_info_raw, t);
                        publishImage(rightImRGB, pub_raw_right, right_frame_id, t);
                    }

                    // Publish the depth image if someone has subscribed to
                    if (depth_SubNumber > 0) {
                        zed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH);
                        publishCamInfo(depth_cam_info_msg, pub_depth_cam_info, t);
                        publishDepth(toCVMat(depthZEDMat), pub_depth, depth_frame_id, t); // in meters
                    }

                    // Publish the point cloud if someone has subscribed to
                    if (cloud_SubNumber > 0) {
                        // Run the point cloud conversion asynchronously to avoid slowing down all the program
                        // Retrieve raw pointCloud data
                        zed.retrieveMeasure(cloud, sl::MEASURE_XYZBGRA);
                        point_cloud_frame_id = cloud_frame_id;
                        point_cloud_time = t;
                        publishPointCloud(width, height, pub_cloud);
                    }

                    // Transform from base to sensor
                    tf2::Transform base_to_sensor;
                    // Look up the transformation from base frame to camera link
                    try {
                        // Save the transformation from base to frame
                        geometry_msgs::TransformStamped b2s = tfBuffer->lookupTransform(base_frame_id, camera_frame_id, t);
                        // Get the TF2 transformation
                        tf2::fromMsg(b2s.transform, base_to_sensor);

                    } catch (tf2::TransformException &ex) {
                        ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                                "will assume it as identity!",
                                base_frame_id.c_str(),
                                camera_frame_id.c_str());
                        ROS_DEBUG("Transform error: %s", ex.what());
                        base_to_sensor.setIdentity();
                    }

                    // Publish the odometry if someone has subscribed to
                    if (odom_SubNumber > 0) {
                        zed.getPosition(pose);
                        // Transform ZED pose in TF2 Transformation
                        tf2::Transform camera_transform;
                        geometry_msgs::Transform c2s;
                        sl::Translation translation = pose.getTranslation();
                        c2s.translation.x = translation(2);
                        c2s.translation.y = -translation(0);
                        c2s.translation.z = -translation(1);
                        sl::Orientation quat = pose.getOrientation();
                        c2s.rotation.x = quat(2);
                        c2s.rotation.y = -quat(0);
                        c2s.rotation.z = -quat(1);
                        c2s.rotation.w = quat(3);
                        tf2::fromMsg(c2s, camera_transform);
                        // Transformation from camera sensor to base frame
                        base_transform = base_to_sensor * camera_transform * base_to_sensor.inverse();
                        // Publish odometry message
                        publishOdom(base_transform, pub_odom, odometry_frame_id, t);
                    }

                    // Publish odometry tf only if enabled
                    if (publish_tf) {
                        //Note, the frame is published, but its values will only change if someone has subscribed to odom
                        publishTrackedFrame(base_transform, transform_odom_broadcaster, base_frame_id, t); //publish the tracked Frame
                    }

                    loop_rate.sleep();
                } else {
                    // Publish odometry tf only if enabled
                    if (publish_tf) {
                        publishTrackedFrame(base_transform, transform_odom_broadcaster, base_frame_id, ros::Time::now()); //publish the tracked Frame before the sleep
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // No subscribers, we just wait
                }
            } // while loop
            zed.close();
        }

        boost::shared_ptr<dynamic_reconfigure::Server<zed_wrapper::ZedConfig>> server;

        void onInit() {
            // Launch file parameters
            resolution = sl::RESOLUTION_HD720;
            quality = sl::DEPTH_MODE_PERFORMANCE;
            sensing_mode = sl::SENSING_MODE_STANDARD;
            rate = 30;
            gpu_id = -1;
            zed_id = 0;
            serial_number = 0;
            odometry_DB = "";

            nh = getMTNodeHandle();
            nh_ns = getMTPrivateNodeHandle();

            // Set  default coordinate frames
            // If unknown left and right frames are set in the same camera coordinate frame
            nh_ns.param<std::string>("odometry_frame", odometry_frame_id, "odometry_frame");
            nh_ns.param<std::string>("base_frame", base_frame_id, "base_frame");
            nh_ns.param<std::string>("camera_frame", camera_frame_id, "camera_frame");
            nh_ns.param<std::string>("depth_frame", depth_frame_id, "depth_frame");

            // Get parameters from launch file
            nh_ns.getParam("resolution", resolution);
            nh_ns.getParam("quality", quality);
            nh_ns.getParam("sensing_mode", sensing_mode);
            nh_ns.getParam("frame_rate", rate);
            nh_ns.getParam("odometry_DB", odometry_DB);
            nh_ns.getParam("openni_depth_mode", openniDepthMode);
            nh_ns.getParam("gpu_id", gpu_id);
            nh_ns.getParam("zed_id", zed_id);
            nh_ns.getParam("depth_stabilization", depth_stabilization);
            int tmp_sn = 0;
            nh_ns.getParam("serial_number", tmp_sn);
            if (tmp_sn > 0) serial_number = tmp_sn;

            // Publish odometry tf
            nh_ns.param<bool>("publish_tf", publish_tf, true);

            if (serial_number > 0)
                ROS_INFO_STREAM("SN : " << serial_number);

            // Print order frames
            ROS_INFO_STREAM("odometry_frame: " << odometry_frame_id);
            ROS_INFO_STREAM("base_frame: " << base_frame_id);
            ROS_INFO_STREAM("camera_frame: " << camera_frame_id);
            ROS_INFO_STREAM("depth_frame: " << depth_frame_id);
            // Status of odometry TF
            ROS_INFO_STREAM("Publish " << odometry_frame_id << " [" << (publish_tf ? "TRUE" : "FALSE") << "]");

            std::string img_topic = "image_rect_color";
            std::string img_raw_topic = "image_raw_color";

            // Set the default topic names
            string left_topic = "left/" + img_topic;
            string left_raw_topic = "left/" + img_raw_topic;
            string left_cam_info_topic = "left/camera_info";
            string left_cam_info_raw_topic = "left/camera_info_raw";
            left_frame_id = camera_frame_id;

            string right_topic = "right/" + img_topic;
            string right_raw_topic = "right/" + img_raw_topic;
            string right_cam_info_topic = "right/camera_info";
            string right_cam_info_raw_topic = "right/camera_info_raw";
            right_frame_id = camera_frame_id;

            string rgb_topic = "rgb/" + img_topic;
            string rgb_raw_topic = "rgb/" + img_raw_topic;
            string rgb_cam_info_topic = "rgb/camera_info";
            string rgb_cam_info_raw_topic = "rgb/camera_info_raw";
            rgb_frame_id = depth_frame_id;

            string depth_topic = "depth/";
            if (openniDepthMode) {
                NODELET_INFO_STREAM("Openni depth mode activated");
                depth_topic += "depth_raw_registered";
            } else {
                depth_topic += "depth_registered";
            }

            string depth_cam_info_topic = "depth/camera_info";

            string point_cloud_topic = "point_cloud/cloud_registered";
            cloud_frame_id = camera_frame_id;

            string odometry_topic = "odom";

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

            nh_ns.getParam("point_cloud_topic", point_cloud_topic);

            nh_ns.getParam("odometry_topic", odometry_topic);

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
                        sl::DeviceProperties prop = getZEDFromSN(serial_number);
                        if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                            std::string msg = "ZED SN" + to_string(serial_number) + " not detected ! Please connect this ZED";
                            NODELET_INFO(msg.c_str());
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        } else {
                            waiting_for_camera = false;
                            param.camera_linux_id = prop.id;
                        }
                    }
                }
            }

            param.coordinate_units = sl::UNIT_METER;
            param.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
            param.depth_mode = static_cast<sl::DEPTH_MODE> (quality);
            param.sdk_verbose = true;
            param.sdk_gpu_id = gpu_id;
            param.depth_stabilization = depth_stabilization;

            sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
            while (err != sl::SUCCESS) {
                err = zed.open(param);
                NODELET_INFO_STREAM(toString(err));
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }

            serial_number = zed.getCameraInformation().serial_number;

            //Reconfigure confidence
            server = boost::make_shared<dynamic_reconfigure::Server < zed_wrapper::ZedConfig >> ();
            dynamic_reconfigure::Server<zed_wrapper::ZedConfig>::CallbackType f;
            f = boost::bind(&ZEDWrapperNodelet::callback, this, _1, _2);
            server->setCallback(f);

            nh_ns.getParam("confidence", confidence);


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

            device_poll_thread = boost::shared_ptr<boost::thread> (new boost::thread(boost::bind(&ZEDWrapperNodelet::device_poll, this)));
        }
    }; // class ZEDROSWrapperNodelet
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_wrapper::ZEDWrapperNodelet, nodelet::Nodelet);
