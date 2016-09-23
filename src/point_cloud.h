#pragma once

#include <string>

namespace ros {
	class Publisher;
	class Time;
}
namespace sl {
	namespace zed {
		class Camera;
	}
}

namespace zw_cloud {
	
	void start(ros::Publisher &pub_cloud);
	void quit();
	bool store(sl::zed::Camera& zed, const std::string& cloud_frame_id, ros::Time cloud_time);

}
