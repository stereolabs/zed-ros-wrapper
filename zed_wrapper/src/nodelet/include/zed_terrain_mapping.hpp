#ifndef ZED_TERRAIN_MAPPING_H
#define ZED_TERRAIN_MAPPING_H
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

#include "sl_tools.h"

#ifdef TERRAIN_MAPPING

#include <sl/Camera.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <mutex>
#include <thread>

namespace zed_wrapper {

    class ZEDTerrainMapping {
      public:
        /* \brief Default constructor
         * \param nh : node handler
         * \param nhNs : private node handler
         * \param zed : pointer to a ZED camera object
         */
        ZEDTerrainMapping(ros::NodeHandle nh, ros::NodeHandle nhNs, sl::Camera* zed = nullptr);

        /* \bried Start mapping loading the parameters from param server
         * \note Terrain Mapping is available since SDK v2.7
         */
        bool startTerrainMapping(sl::Transform initialPoseSl);

        /* \brief Set local map type
         * \ param circular : if True map is radial, otherwise is square
         */
        void setLocalMapType(bool circular);

        /* \brief Set the size of the local map
         * \ param size : size of the local map in meters (diameter if circular)
         */
        void setLocalMapSize(double size);

      protected:
        bool init();

        /* \brief Callback to handle async terrain MAPPING to generate high frequency local maps
         * \param e : the ros::TimerEvent binded to the callback
         */
        void localTerrainCallback(const ros::TimerEvent& e);

        /* \brief Callback to handle async terrain MAPPING to generate low frequency global maps
         * \param e : the ros::TimerEvent binded to the callback
         */
        void globalTerrainCallback(const ros::TimerEvent& e);

        /* \brief Initialize the ROS Map messages
         * \param map_W_m : width of the map in meters
         * \param map_H_m : height of the map in meters
         */
        void initGlobalMapMsgs(double map_W_m, double map_H_m);

        /* \brief Publish local height and cost maps from updated Terrain Chunks
         * \param minX : minimum X coordinate of the map in meters
         * \param minY : minimum Y coordinate of the map in meters
         * \param maxX : maximum X coordinate of the map in meters
         * \param maxY : maximum Y coordinate of the map in meters
         * \param chunks : updated chunks from terrain mapping
         * \param heightSub : Height map subscribers count
         * \param costSub : Cost map subscribers count
         * \param cloudSub : Height cloud subscribers count
         * \param mrkSub : Height markers array subscribers count
         * \param t : timestamp
         */
        void publishLocalMaps(float camX, float camY, float minX, float minY, float maxX,
                              float maxY, std::vector<sl::HashKey>& chunks,
                              ros::Time t);

        /* \brief Publish global height and cost maps from updated Terrain Chunks
         * \param chunks : updated chunks from terrain mapping
         * \param heightSub : Height map subscribers count
         * \param costSub : Cost map subscribers count
         * \param cloudSub : Height cloud subscribers count
         * \param mrkSub : Height markers array subscribers count
         * \param heightUpdSub : Height map updates subscribers count
         * \param costUpdSub : Cost map updates subscribers count
         * \param t : timestamp
         */
        void publishGlobalMaps(std::vector<sl::HashKey>& chunks,  ros::Time t);

        /* \brief Callback to handle new global maps subscription.
         * \param e : the ros::TimerEvent binded to the callback
         */
        void globalMapSubscribeCallback(const ros::SingleSubscriberPublisher& pub);

        /* \brief Service callback to GetMap service
         * server to simulate the Navigation stack `map_server` functionality
         */
        bool on_get_static_map(nav_msgs::GetMap::Request&  req,
                               nav_msgs::GetMap::Response& res);

        /* \brief Service callback to GetMap service
         * server to request Local Height Map
         */
        bool on_get_loc_height_map(nav_msgs::GetMap::Request&  req,
                                   nav_msgs::GetMap::Response& res);

        /* \brief Service callback to GetMap service
         * server to request Local Cost Map
         */
        bool on_get_loc_cost_map(nav_msgs::GetMap::Request&  req,
                                 nav_msgs::GetMap::Response& res);

        /* \brief Service callback to GetMap service
         * server to request Local Occupancy Map
         */
        bool on_get_loc_occupancy(nav_msgs::GetMap::Request&  req,
                                  nav_msgs::GetMap::Response& res);

        /* \brief Service callback to GetMap service
         * server to request Global Height Map
         */
        bool on_get_glob_height_map(nav_msgs::GetMap::Request&  req,
                                    nav_msgs::GetMap::Response& res);

        /* \brief Service callback to GetMap service
         * server to request Global Cost Map
         */
        bool on_get_glob_cost_map(nav_msgs::GetMap::Request&  req,
                                  nav_msgs::GetMap::Response& res);

        /* \brief Service callback to GetMap service
         * server to request Global Occupancy Map
         */
        bool on_get_glob_occupancy(nav_msgs::GetMap::Request&  req,
                                   nav_msgs::GetMap::Response& res);

      private:
        // ROS
        ros::NodeHandle mNh;
        ros::NodeHandle mNhNs;

        // Publishers
        ros::Publisher mPubLocalHeightMap;          // Local Height Map publisher
        ros::Publisher mPubLocalHeightCloud;        // Local Height Pointcloud publisher
        ros::Publisher mPubLocalHeightMrk;          // Local Height Cubes publisher
        ros::Publisher mPubLocalCostMap;            // Local Costmap publisher
        ros::Publisher mPubLocalInflatedOccupGrid;  // Local Inflated occupancy grid publisher (thresholded costmap)
        ros::Publisher mPubLocalDynObstaclesMap;       // Local Dynamic Obstacles

        ros::Publisher mPubGlobalHeightMap;         // Global Height Map publisher

        ros::Publisher mPubGlobalHeightCloud;       // Global Height Pointcloud publisher
        ros::Publisher mPubGlobalHeightMrk;         // Global Height Cubes publisher
        ros::Publisher mPubGlobalCostMap;           // Global Costmap publisher
        ros::Publisher mPubGlobalInflatedOccupGrid; // Global Inflated occupancy grid publisher (thresholded costmap)

        //ros::Publisher mPubGlobalHeightMapUpd;
        //ros::Publisher mPubGlobalCostMapUpd;
        //ros::Publisher mPubGlobalOccGridUpd;

        ros::Publisher mPubGlobalHeightMapImg;      // Global Height Map Image publisher
        ros::Publisher mPubGlobalColorMapImg;       // Global Height Map Image publisher
        ros::Publisher mPubGlobalCostMapImg;        // Global Height Map Image publisher


        // Frames
        std::string mMapFrameId;
        std::string mOdometryFrameId;
        std::string mBaseFrameId;
        std::string mCameraFrameId;

        // Transform listener
        boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> mTfListener;

        // ZED
        sl::Camera* mZed;

        // Timers
        ros::Timer mLocalTerrainTimer;
        ros::Timer mGlobalTerrainTimer;

        // Terrain MApping
        sl::Terrain mTerrain;
        sl::Transform mInitialPoseSl;

        // Flags
        bool mInitialized;
        bool mMappingReady;
        bool mGlobMapWholeUpdate;

        // Messages
        sensor_msgs::PointCloud2 mLocalHeightPointcloudMsg;
        sensor_msgs::PointCloud2 mGlobalHeightPointcloudMsg;
        nav_msgs::OccupancyGrid mLocHeightMapMsg;
        nav_msgs::OccupancyGrid mLocInflatedOccupGridMsg;
        nav_msgs::OccupancyGrid mLocCostMapMsg;
        nav_msgs::OccupancyGrid mLocDynObstaclesMsg;
        nav_msgs::OccupancyGrid mGlobHeightMapMsg;
        nav_msgs::OccupancyGrid mGlobCostMapMsg;
        nav_msgs::OccupancyGrid mGlobInflatedOccupGridMsg;

        // Services
        ros::ServiceServer mSrvGetStaticMap;
        ros::ServiceServer mSrvGetGlobHeightMap;
        ros::ServiceServer mSrvGetGlobCostMap;
        ros::ServiceServer mSrvGetGlobOccupancy;
        ros::ServiceServer mSrvGetLocHeightMap;
        ros::ServiceServer mSrvGetLocCostMap;
        ros::ServiceServer mSrvGetLocOccupancy;

        sl::timeStamp mLastGlobMapTimestamp;

        // Terrain Mapping Params
        double mLocalTerrainPubRate;
        double mGlobalTerrainPubRate;
        float mMapAgentStep = 0.05f;
        float mMapAgentSlope = 20.f/*degrees*/;
        float mMapAgentRadius = 0.18f;
        float mMapAgentHeight = 0.8f;
        float mMapAgentRoughness = 0.05f;
        float mMapMaxDepth = 3.5f;
        float mMapMaxHeight = 0.5f;
        float mMapHeightResol = .025f;
        bool mMapLocalCircular = false;
        float mMapLocalSize = 3.0f;
        int mMapResolIdx = 1;
        double mTerrainMapRes;

        // Mutex
        std::mutex mTerrainMutex;
        std::mutex mLocMapMutex;
        std::mutex mGlobMapMutex;
    };
}

#endif // TERRAIN_MAPPING

#endif // ZED_TERRAIN_MAPPING_H
