/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"  // pcl::SAC_SAMPLE_SIZE is protected since PCL 1.8.0
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

//#define COLOR_OCTOMAP_SERVER // switch color here - easier maintenance, only maintain OctomapServer. Two targets are defined in the cmake, octomap_server_color and octomap_server. One has this defined, and the other doesn't

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif

#include <octomap_server/types.h>
#include <octomap_server/PointCloudSynchronizer.h>
#include <octomap_server/OcTreeStampedWithExpiry.h>
#include <octomap_server/SensorUpdateKeyMap.h>

namespace octomap_server {
class OctomapServer {

public:
  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  OctomapServer(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle());
  virtual ~OctomapServer();
  virtual bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  virtual bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
  bool eraseBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  virtual void insertSegmentedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ground_cloud,
                                            const sensor_msgs::PointCloud2::ConstPtr& nonground_cloud,
                                            const sensor_msgs::PointCloud2::ConstPtr& nonclearing_nonground_cloud,
                                            const sensor_msgs::PointCloud2::ConstPtr& nonmarking_nonground_cloud,
                                            const std::string& sensor_origin_frame_id,
                                            unsigned int callback_id);
  virtual void insertSegmentedCloudCallback(PointCloudSynchronizer::MsgTopicVectorConstPtr msgs_and_topics,
                                            const std::string& sensor_origin_frame_id,
                                            unsigned int callback_id);
  virtual bool openFile(const std::string& filename);

  void startTrackingBounds(std::string name);
  void stopTrackingBounds(std::string name);
  void getTrackingBounds(std::string name, boost::shared_ptr<OcTreeT> delta_tree, boost::shared_ptr<octomap::OcTree> bounds_tree);
  void resetTrackingBounds(std::string name);

protected:
  // Add an input point cloud topic
  inline void addCloudTopic(const std::string &topic) {
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > pointCloudSub(
      new message_filters::Subscriber<sensor_msgs::PointCloud2> (
        m_nh,
        topic,
        5
      )
    );
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > tfPointCloudSub(
      new tf::MessageFilter<sensor_msgs::PointCloud2> (
        *pointCloudSub,
        m_tfListener,
        m_worldFrameId,
        5
      )
    );
    tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, _1));
    m_pointCloudSubs.push_back(pointCloudSub);
    m_tfPointCloudSubs.push_back(tfPointCloudSub);
  }

  // Add an input pre-segmented point cloud topic
  inline void addSegmentedCloudTopic(const std::string &ground_topic,
                                     const std::string &nonground_topic,
                                     const std::string &nonclearing_nonground_topic,
                                     const std::string &nonmarking_nonground_topic,
                                     const std::string &sensor_origin_frame_id) {
    if (ground_topic.empty() &&
        nonground_topic.empty() &&
        nonclearing_nonground_topic.empty() &&
        nonmarking_nonground_topic.empty())
    {
      ROS_WARN("addSegmentedCloudTopic passed no topic names");
      return;
    }

    boost::shared_ptr<PointCloudSynchronizer> sync(new PointCloudSynchronizer(m_nh, m_tfListener, 5));

    int callback_id = m_callbackCounts.size();
    m_callbackCounts.push_back(0);
    std::vector<std::string> callback_topics;
    callback_topics.resize(4);
    callback_topics[0] = ground_topic;
    callback_topics[1] = nonground_topic;
    callback_topics[2] = nonclearing_nonground_topic;
    callback_topics[3] = nonmarking_nonground_topic;
    m_callbackTopics.push_back(callback_topics);

    if (!ground_topic.empty())
      sync->addTopic(ground_topic, m_worldFrameId);
    if (!nonground_topic.empty())
      sync->addTopic(nonground_topic, m_worldFrameId);
    if (!nonclearing_nonground_topic.empty())
      sync->addTopic(nonclearing_nonground_topic, m_worldFrameId);
    if (!nonmarking_nonground_topic.empty())
      sync->addTopic(nonmarking_nonground_topic, m_worldFrameId);

    sync->registerCallback(boost::bind(&OctomapServer::insertSegmentedCloudCallback, this, _1,
                                       sensor_origin_frame_id, callback_id));
    m_syncs.push_back(sync);
  }


  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0] + voxelWidth >= m_updateBBXMin[0]
            && key[1] + voxelWidth >= m_updateBBXMin[1]
            && key[0] <= m_updateBBXMax[0]
            && key[1] <= m_updateBBXMax[1]);
  }

  void reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level);
  void onNewBinaryMapSubscription(const ros::SingleSubscriberPublisher& pub);
  void onNewBinaryMapUpdateSubscription(const ros::SingleSubscriberPublisher& pub);
  void publishBinaryOctoMapUpdate(const ros::Time& rostime = ros::Time::now(),
      const ros::SingleSubscriberPublisher* pub =  nullptr);
  void onNewFullMapSubscription(const ros::SingleSubscriberPublisher& pub);
  void onNewFullMapUpdateSubscription(const ros::SingleSubscriberPublisher& pub);
  void publishFullOctoMapUpdate(const ros::Time& rostime = ros::Time::now(),
      const ros::SingleSubscriberPublisher* pub = nullptr);
  virtual void publishAll(const ros::Time& rostime = ros::Time::now());

  /**
  * @brief update occupancy map with a scan labeled as ground and nonground.
  * The scans should be in the global map frame.
  *
  * @param sensorOrigin origin of the measurements for raycasting
  * @param ground scan endpoints on the ground plane (only clear space)
  * @param nonground endpoints (clear up to occupied endpoint)
  * @param nonclearing, nonground endpoints (do not clear up to occupied endpoint)
  */
  virtual void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground,
                          const PCLPointCloud& nonclearing_nonground = PCLPointCloud(),
                          const PCLPointCloud& nonmarking_nonground = PCLPointCloud());

  void handleRayPoint(SensorUpdateKeyMap* update_cells,
                      const octomap::point3d& sensor_origin,
                      const octomap::point3d& point,
                      bool free,
                      bool occupied,
                      bool skip_tracing);

  void applyUpdate();

  /// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
  void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const;

  /**
  * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
  * @param key
  * @return
  */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  /// hook that is called before traversing all nodes
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode(const OcTreeT::iterator& it) {};

  /// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX(const OcTreeT::iterator& it) {};

  /// hook that is called when traversing occupied nodes of the updated Octree
  virtual void handleOccupiedNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing occupied nodes in the updated area (updates 2D map projection here)
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes of the updated Octree
  virtual void handleFreeNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes in the updated area (updates 2D map projection here)
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const {
    return m_gridmap.info.width * j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
    return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                  (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

  }

  /**
   * Adjust data of map due to a change in its info properties (origin or size,
   * resolution needs to stay fixed). map already contains the new map info,
   * but the data is stored according to oldMapInfo.
   */

  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;

  inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
    return (    oldMapInfo.height != newMapInfo.height
                || oldMapInfo.width != newMapInfo.width
                || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }

  void enableChangeCallback();
  void disableChangeCallback();
  void touchKeyAtDepth(const octomap::OcTreeKey& key, unsigned int depth = std::numeric_limits<unsigned int>::max());
  void touchKey(const octomap::OcTreeKey& key);
  void valueChangeCallback(const octomap::OcTreeKey& key, unsigned int depth, const bool node_just_created,
      const float prev_full_val, const bool prev_binary_val,
      const float curr_full_val, const bool curr_binary_val);

  static std_msgs::ColorRGBA heightMapColor(double h);
  ros::NodeHandle m_nh;
  ros::NodeHandle m_nh_private;
  ros::Publisher  m_markerPub, m_binaryMapPub, m_binaryMapUpdatePub, m_fullMapPub, m_fullMapUpdatePub, m_pointCloudPub, m_collisionObjectPub, m_mapPub, m_cmapPub, m_fmapPub, m_fmarkerPub;
  std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > > m_pointCloudSubs;
  std::vector<boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > > m_tfPointCloudSubs;
  std::vector<boost::shared_ptr<PointCloudSynchronizer>> m_syncs;
  ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_clearBBXService, m_eraseBBXService, m_resetService;
  tf::TransformListener m_tfListener;
  boost::recursive_mutex m_config_mutex;
  dynamic_reconfigure::Server<OctomapServerConfig> m_reconfigureServer;

  OcTreeT* m_octree;
  OcTreeT* m_universe;
  OcTreeT* m_octree_deltaBB_;
  OcTreeT* m_octree_binary_deltaBB_;
  octomap::KeyRay m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  double m_maxRange;
  std::string m_worldFrameId; // the map frame
  std::string m_baseFrameId; // base of the robot for ground plane filtering and distance limit
  bool m_useHeightMap;
  bool m_useTimedMap;
  std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;
  double m_colorFactor;

  bool m_latchedTopics;
  int m_callbackSkipCount;
  std::vector<unsigned int> m_callbackCounts;
  std::vector<std::vector<std::string>> m_callbackTopics;
  bool m_trackFreeSpace;
  bool m_publishFreeSpace;
  bool m_newFullSub;
  bool m_newBinarySub;
  uint32_t m_binarySeq;
  uint32_t m_fullSeq;
  double m_publish3DMapPeriod;
  ros::Time m_publish3DMapLastTime;
  double m_publish3DMapUpdatePeriod;
  ros::Time m_publish3DMapUpdateLastTime;
  double m_publish2DPeriod;
  ros::Time m_publish2DLastTime;

  double m_res;
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;

  double m_pointcloudMinX;
  double m_pointcloudMaxX;
  double m_pointcloudMinY;
  double m_pointcloudMaxY;
  double m_pointcloudMinZ;
  double m_pointcloudMaxZ;
  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;
  bool m_filterSpeckles;

  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;

  bool m_compressMap;
  double m_compressPeriod;
  ros::Time m_compressLastTime;

  bool m_initConfig;

  // downprojected 2D map:
  bool m_incrementalUpdate;
  nav_msgs::OccupancyGrid m_gridmap;
  bool m_publish2DMap;
  bool m_mapOriginChanged;
  octomap::OcTreeKey m_paddedMinKey;
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
  bool m_useColoredMap;

  // time-based degrading
  double m_expirePeriod;
  ros::Time m_expireLastTime;

  // efficient sensor update tracking
  SensorUpdateKeyMap m_voxelFilter;
  SensorUpdateKeyMap m_updateCells;
  bool m_deferUpdateToPublish;

  // base distance-based deletion (<= 0.0 is disabled)
  double m_baseDistanceLimitPeriod;
  ros::Time m_baseDistanceLimitLastTime;
  double m_base2DDistanceLimit;
  double m_baseHeightLimit;
  double m_baseDepthLimit;
  double m_update2DDistanceLimit;
  double m_updateHeightLimit;
  double m_updateDepthLimit;
  bool m_baseToWorldValid;
  tf::StampedTransform m_baseToWorldTf;

};
}

#endif
