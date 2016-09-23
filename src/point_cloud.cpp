#include "point_cloud.h"

// ZW_CLOUD_ALGO=1:
//  - Use a background thread to publish the PointCloud2 message
//  - Call retrieveMeasure to get the point cloud in CPU memory
//  - Convert the sl::zed::Mat to pcl::PointCloud
//  - Convert from pcl::PointCloud to the ROS message
// ZW_CLOUD_ALGO=2:
//  - No background thread
//  - Call retrieveMeasure_gpu to get the point cloud in a CUDA buffer
//  - Copy the buffer data straight into the ROS message buffer
#define ZW_CLOUD_ALGO 2

#if ZW_CLOUD_ALGO == 1

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

//PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//ZED Includes
#include <zed/Camera.hpp>

using namespace std;

namespace zw_cloud {

std::mutex mut;
std::condition_variable cv;
std::unique_ptr<std::thread> th;
bool thread_running = true;
bool data_available = false;

pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
string frame_id = "";
ros::Time frame_time;

void publish(ros::Publisher &pub_cloud);

void start(ros::Publisher &pub_cloud) {
	if (th) return;
	
	thread_running = true;
	data_available = false;
	th.reset(new std::thread(&publish, std::ref(pub_cloud)));
}

void quit() {
	if (!th) return;

	{
		lock_guard<mutex> lock(mut);
		thread_running = false;
		cv.notify_all();
	}
	th->join();
	th.reset();
}

bool store(sl::zed::Camera& zed, const std::string& cloud_frame_id, ros::Time cloud_time) {
	// Check if the point cloud thread is busy
	unique_lock<mutex> lock(mut, try_to_lock);
	if (!lock.owns_lock())
		return false;
	// Check if the point cloud thread already has a queued message
	if (data_available)
		return false;
	
	// Retrieve raw pointCloud data
	sl::zed::Mat measure = zed.retrieveMeasure(sl::zed::MEASURE::XYZBGRA);
	int size = measure.width * measure.height;
	
	point_cloud.is_dense = false;
	point_cloud.width = measure.width;
	point_cloud.height = measure.height;
	point_cloud.points.resize(size);
	
	int data_width = measure.getWidthByte();
	const sl::uchar* data_ptr = measure.data;
	pcl::PointXYZRGB* dest_ptr = point_cloud.points.data();
	
	int transp_count = 0;
	for (int row = 0; row < measure.height; ++row, data_ptr += data_width) {
		const float* row_data = reinterpret_cast<const float*>(data_ptr);
		for (int col = 0; col < measure.width; ++col, ++dest_ptr) {
			dest_ptr->y   = -row_data[4*col+0];
			dest_ptr->z   =  row_data[4*col+1];
			dest_ptr->x   = -row_data[4*col+2];
			dest_ptr->rgb =  row_data[4*col+3];
			uint8_t alpha = reinterpret_cast<uint8_t*>(&dest_ptr->rgb)[3];
			if (alpha != 255)
				++transp_count;
		}
	}
	
	ROS_INFO_STREAM("Points: " << size << ", transparent: " << transp_count);
	
	frame_id = cloud_frame_id;
	frame_time = cloud_time;

	data_available = true;
	cv.notify_all();
}

/* \brief Publish a pointCloud with a ros Publisher
 * \param pub_cloud : the publisher object to use
 * \param cloud_frame_id : the id of the reference frame of the point cloud
 * \param t : the ros::Time to stamp the point cloud
 */
void publish(ros::Publisher &pub_cloud) {
	unique_lock<mutex> lock(mut);
	for (;;) {
		while (thread_running && !data_available)
			cv.wait(lock);
		// check if the thread has to continue
		if (!thread_running)
			break;

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(point_cloud, output); // Convert the point cloud to a ROS message
        output.header.frame_id = frame_id; // Set the header values of the ROS message
        output.header.stamp = frame_time;
        ROS_INFO_STREAM("Publishing message: " <<
			output.width << "x" << output.height << "=" << output.data.size() << " bytes" <<
			" point_step=" << output.point_step << " row_step=" << output.row_step);

        pub_cloud.publish(output);
        data_available = false;
    }
}

}
#endif

#if ZW_CLOUD_ALGO == 2

#include <string>

// ROS includes
#include <ros/console.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// ZED includes
#include <zed/Camera.hpp>

// CUDA includes
#include <cuda.h>
#include <cuda_runtime_api.h>

using namespace std;

namespace zw_cloud {

ros::Publisher* pub_cloud = nullptr;

void start(ros::Publisher &publisher) {
	pub_cloud = &publisher;
}

void quit() {
	pub_cloud = nullptr;
}

bool store(sl::zed::Camera& zed, const std::string& cloud_frame_id, ros::Time cloud_time) {
	// Get the point cloud data in a GPU buffer
	sl::zed::Mat measure = zed.retrieveMeasure_gpu(sl::zed::MEASURE::XYZBGRA);
	
	// Build the message
	sensor_msgs::PointCloud2 msg;
	msg.header.stamp = cloud_time;
	msg.header.frame_id = cloud_frame_id;
	
	msg.height = measure.height;
	msg.width = measure.width;
	msg.is_bigendian = false;
	msg.is_dense = false;
	msg.point_step = measure.channels * measure.getDataSize();
	msg.row_step = msg.point_step * msg.width;
	
	sensor_msgs::PointCloud2Modifier modifier(msg);
	modifier.setPointCloud2Fields(4,
		"y", 1, sensor_msgs::PointField::FLOAT32,
		"z", 1, sensor_msgs::PointField::FLOAT32,
		"x", 1, sensor_msgs::PointField::FLOAT32,
		"rgba", 1, sensor_msgs::PointField::FLOAT32);
	
	cuCtxSetCurrent(zed.getCUDAContext());
	cudaError_t err = cudaMemcpy2D(
		msg.data.data(), msg.row_step, measure.data, measure.getWidthByte(),
		msg.row_step, msg.height, cudaMemcpyDeviceToHost);
	
	if (err != cudaSuccess) {
		ROS_WARN_STREAM("Error: cudaMemcpy2D failed with error " << err << ": " << cudaGetErrorString(err));
	} else {
		pub_cloud->publish(msg);
	}
}

}
#endif
