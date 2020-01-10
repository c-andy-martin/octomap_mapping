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

#include <limits>
#include <octomap_server/OctomapServer.h>
#include <octomap_server/SensorUpdateKeyMap.h>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace octomap_server{

OctomapServer::OctomapServer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_reconfigureServer(m_config_mutex, private_nh_),
  m_octree(NULL),
  m_octree_deltaBB_(NULL),
  m_octree_binary_deltaBB_(NULL),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_useHeightMap(true),
  m_useTimedMap(false),
  m_useColoredMap(false),
  m_colorFactor(0.8),
  m_latchedTopics(true),
  m_callbackSkipCount(0),
  m_trackFreeSpace(true),
  m_publishFreeSpace(false),
  m_publish3DMapPeriod(0.0),
  m_publish2DPeriod(0.0),
  m_publish3DMapUpdatePeriod(0.0),
  m_publish3DMapLastTime(ros::Time::now()),
  m_publish2DLastTime(ros::Time::now()),
  m_publish3DMapUpdateLastTime(ros::Time::now()),
  m_newFullSub(false),
  m_newBinarySub(false),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_pointcloudMinX(-std::numeric_limits<double>::max()),
  m_pointcloudMaxX(std::numeric_limits<double>::max()),
  m_pointcloudMinY(-std::numeric_limits<double>::max()),
  m_pointcloudMaxY(std::numeric_limits<double>::max()),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_minSizeX(0.0), m_minSizeY(0.0),
  m_filterSpeckles(false), m_filterGroundPlane(false),
  m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
  m_compressMap(true),
  m_compressPeriod(0.0),
  m_compressLastTime(ros::Time::now()),
  m_incrementalUpdate(false),
  m_initConfig(true),
  m_expirePeriod(0.0),
  m_expireLastTime(ros::Time::now()),
  m_deferUpdateToPublish(false),
  m_baseDistanceLimitPeriod(0.0),
  m_baseDistanceLimitLastTime(ros::Time::now()),
  m_base2DDistanceLimit(std::numeric_limits<double>::max()),
  m_baseHeightLimit(std::numeric_limits<double>::max()),
  m_baseDepthLimit(std::numeric_limits<double>::max()),
  m_update2DDistanceLimit(std::numeric_limits<double>::max()),
  m_updateHeightLimit(std::numeric_limits<double>::max()),
  m_updateDepthLimit(std::numeric_limits<double>::max()),
  m_baseToWorldValid(false)
{
  double probHit, probMiss, thresMin, thresMax;

  m_nh_private.param("frame_id", m_worldFrameId, m_worldFrameId);
  m_nh_private.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  m_nh_private.param("height_map", m_useHeightMap, m_useHeightMap);
  m_nh_private.param("timed_map", m_useTimedMap, m_useTimedMap);
  m_nh_private.param("colored_map", m_useColoredMap, m_useColoredMap);
  m_nh_private.param("color_factor", m_colorFactor, m_colorFactor);

  m_nh_private.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  m_nh_private.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  m_nh_private.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  m_nh_private.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  m_nh_private.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  m_nh_private.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  m_nh_private.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  m_nh_private.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  m_nh_private.param("min_x_size", m_minSizeX,m_minSizeX);
  m_nh_private.param("min_y_size", m_minSizeY,m_minSizeY);

  std::vector<std::string> cloud_topics;
  m_nh_private.getParam("cloud_topics", cloud_topics);

  XmlRpc::XmlRpcValue segmented_topics;
  m_nh_private.getParam("segmented_topics", segmented_topics);

  m_nh_private.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  m_nh_private.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  // distance of points from plane for RANSAC
  m_nh_private.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  // angular derivation of found plane:
  m_nh_private.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  m_nh_private.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

  m_nh_private.param("sensor_model/max_range", m_maxRange, m_maxRange);

  m_nh_private.param("resolution", m_res, m_res);
  m_nh_private.param("sensor_model/hit", probHit, 0.7);
  m_nh_private.param("sensor_model/miss", probMiss, 0.4);
  m_nh_private.param("sensor_model/min", thresMin, 0.12);
  m_nh_private.param("sensor_model/max", thresMax, 0.97);
  m_nh_private.param("compress_map", m_compressMap, m_compressMap);
  m_nh_private.param("compress_period", m_compressPeriod, m_compressPeriod);
  m_nh_private.param("incremental_2D_projection", m_incrementalUpdate, m_incrementalUpdate);

  // only enabled when expireTimeDelta is positive
  m_nh_private.param("expire_time_delta", m_expirePeriod, m_expirePeriod);

  m_nh_private.param("defer_update_to_publish", m_deferUpdateToPublish, m_deferUpdateToPublish);
  m_nh_private.param("skip_count", m_callbackSkipCount, m_callbackSkipCount);

  m_nh_private.param("base_distance_limit_time_delta", m_baseDistanceLimitPeriod, m_baseDistanceLimitPeriod);
  m_nh_private.param("base_2d_distance_limit", m_base2DDistanceLimit, m_base2DDistanceLimit);
  m_nh_private.param("base_height_limit", m_baseHeightLimit, m_baseHeightLimit);
  m_nh_private.param("base_depth_limit", m_baseDepthLimit, m_baseDepthLimit);
  m_nh_private.param("update_2d_distance_limit", m_update2DDistanceLimit, m_update2DDistanceLimit);
  m_nh_private.param("update_height_limit", m_updateHeightLimit, m_updateHeightLimit);
  m_nh_private.param("update_depth_limit", m_updateDepthLimit, m_updateDepthLimit);
  // It makes no sense for the sensor update limits to be bigger than the
  // robot base limits. Clip the update limits to the base ones.
  if (m_base2DDistanceLimit < m_update2DDistanceLimit)
    m_update2DDistanceLimit = m_base2DDistanceLimit;
  if (m_baseHeightLimit < m_updateHeightLimit)
    m_updateHeightLimit = m_baseHeightLimit;
  if (m_baseDepthLimit < m_updateDepthLimit)
    m_updateDepthLimit = m_baseDepthLimit;

  if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)){
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<m_pointcloudMinZ <<", "<< m_pointcloudMaxZ << "], excluding the ground level z=0. "
              << "This will not work.");
  }

  if (m_useHeightMap && m_useColoredMap) {
    ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_INFO_STREAM("Using RGB color registration (if information available)");
#else
    ROS_ERROR_STREAM("Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
  }

  if (m_useHeightMap && m_useTimedMap) {
    ROS_WARN_STREAM("You enabled both height map and timed map. This is contradictory. Defaulting to height map.");
    m_useTimedMap = false;
  }

  if (m_useColoredMap && m_useTimedMap) {
    ROS_WARN_STREAM("You enabled both colored map and timed map. This is contradictory. Defaulting to colored map.");
    m_useTimedMap = false;
  }

  // initialize octomap object & params
  m_octree = new OcTreeT(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  enableChangeCallback();

  // make the universe and delta trees "binary" in nature by making the
  // probability of hit/miss much bigger than the clamping thresholds
  m_universe = new OcTreeT(m_res);
  m_universe->setProbHit(.99);
  m_universe->setProbMiss(.01);
  m_universe->setClampingThresMin(.1);
  m_universe->setClampingThresMax(.9);
  m_octree_deltaBB_ = new OcTreeT(*m_universe);
  m_octree_binary_deltaBB_ = new OcTreeT(*m_universe);
  // set the universe tree to occupied everywhere
  m_universe->setNodeValueAtDepth(OcTreeKey(), 0, m_universe->getClampingThresMaxLog());

  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_gridmap.info.resolution = m_res;

  double a_coeff, c_coeff, quadratic_start, c_coeff_free;
  m_nh_private.param("expiry/a_coeff", a_coeff, 1.0 / 25.0);
  m_nh_private.param("expiry/c_coeff", c_coeff, 2.0);
  m_nh_private.param("expiry/quadratic_start", quadratic_start, 30.0);
  m_nh_private.param("expiry/c_coeff_free", c_coeff_free, 60.0 * 60.0 * 18.0);
  m_octree->setQuadraticParameters(a_coeff, c_coeff, quadratic_start, c_coeff_free);
  // get expiration time setup
  m_octree->expireNodes();

  double r, g, b, a;
  m_nh_private.param("color/r", r, 0.0);
  m_nh_private.param("color/g", g, 0.0);
  m_nh_private.param("color/b", b, 1.0);
  m_nh_private.param("color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  m_nh_private.param("color_free/r", r, 0.0);
  m_nh_private.param("color_free/g", g, 1.0);
  m_nh_private.param("color_free/b", b, 0.0);
  m_nh_private.param("color_free/a", a, 1.0);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  m_nh_private.param("track_free_space", m_trackFreeSpace, m_trackFreeSpace);
  m_octree->setDeleteMinimum(!m_trackFreeSpace);

  m_nh_private.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);
  m_nh_private.param("publish_3d_map_period", m_publish3DMapPeriod, m_publish3DMapPeriod);
  m_nh_private.param("publish_3d_map_update_period", m_publish3DMapUpdatePeriod, m_publish3DMapUpdatePeriod);
  m_nh_private.param("publish_2d_period", m_publish2DPeriod, m_publish2DPeriod);

  m_nh_private.param("latch", m_latchedTopics, m_latchedTopics);
  if (m_latchedTopics){
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  } else
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");

  m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, m_latchedTopics);
  m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, boost::bind(&OctomapServer::onNewBinaryMapSubscription, this, _1));
  m_binaryMapUpdatePub = m_nh.advertise<octomap_msgs::OctomapUpdate>("octomap_binary_updates", 10, boost::bind(&OctomapServer::onNewBinaryMapUpdateSubscription, this, _1));
  m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, boost::bind(&OctomapServer::onNewFullMapSubscription, this, _1));
  m_fullMapUpdatePub = m_nh.advertise<octomap_msgs::OctomapUpdate>("octomap_full_updates", 10, boost::bind(&OctomapServer::onNewFullMapUpdateSubscription, this, _1));
  m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
  m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, m_latchedTopics);
  m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, m_latchedTopics);

  // Already segmented topics
  if (segmented_topics.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int32_t i = 0; i < segmented_topics.size(); ++i) {
      std::string ground_topic = "";
      std::string nonground_topic = "";
      std::string nonclearing_nonground_topic = "";
      std::string nonmarking_nonground_topic = "";
      std::string sensor_origin_frame_id = "";
      XmlRpc::XmlRpcValue& segmented_topic(segmented_topics[i]);
      if (segmented_topic.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        std::string member;
        member = "ground_topic";
        if (segmented_topic.hasMember(member)) {
          XmlRpc::XmlRpcValue& v(segmented_topic[member]);
          if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
            ground_topic = static_cast<std::string>(v);
          }
        }
        member = "nonground_topic";
        if (segmented_topic.hasMember(member)) {
          XmlRpc::XmlRpcValue& v(segmented_topic[member]);
          if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
            nonground_topic = static_cast<std::string>(v);
          }
        }
        member = "nonclearing_nonground_topic";
        if (segmented_topic.hasMember(member)) {
          XmlRpc::XmlRpcValue& v(segmented_topic[member]);
          if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
            nonclearing_nonground_topic = static_cast<std::string>(v);
          }
        }
        member = "nonmarking_nonground_topic";
        if (segmented_topic.hasMember(member)) {
          XmlRpc::XmlRpcValue& v(segmented_topic[member]);
          if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
            nonmarking_nonground_topic = static_cast<std::string>(v);
          }
        }
        member = "sensor_origin_frame_id";
        if (segmented_topic.hasMember(member)) {
          XmlRpc::XmlRpcValue& v(segmented_topic[member]);
          if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
            sensor_origin_frame_id = static_cast<std::string>(v);
          }
        }
      }
      addSegmentedCloudTopic(ground_topic, nonground_topic,
                             nonclearing_nonground_topic, nonmarking_nonground_topic,
                             sensor_origin_frame_id);
    }
  }

  for (unsigned i=0; i < cloud_topics.size(); i++) {
    addCloudTopic(cloud_topics[i]);
  }

  // If we have not subscribed to any topics, subscribe to default "cloud_in"
  if (m_pointCloudSubs.empty()) {
    addCloudTopic("cloud_in");
  }

  m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &OctomapServer::octomapBinarySrv, this);
  m_octomapFullService = m_nh.advertiseService("octomap_full", &OctomapServer::octomapFullSrv, this);
  m_clearBBXService = m_nh_private.advertiseService("clear_bbx", &OctomapServer::clearBBXSrv, this);
  m_eraseBBXService = m_nh_private.advertiseService("erase_bbx", &OctomapServer::eraseBBXSrv, this);
  m_resetService = m_nh_private.advertiseService("reset", &OctomapServer::resetSrv, this);

  dynamic_reconfigure::Server<OctomapServerConfig>::CallbackType f;
  f = boost::bind(&OctomapServer::reconfigureCallback, this, _1, _2);
  m_reconfigureServer.setCallback(f);
}

OctomapServer::~OctomapServer(){
  // Because time synchronizers reference the TF listeners, delete them first
  m_syncs.clear();
  // Because TF message filters reference the subscriber, deleted them next
  m_tfPointCloudSubs.clear();
  m_pointCloudSubs.clear();
  m_callbackCounts.clear();

  delete m_universe;
  m_universe = NULL;

  delete m_octree;
  m_octree = NULL;

  delete m_octree_deltaBB_;
  m_octree_deltaBB_ = NULL;
}

bool OctomapServer::openFile(const std::string& filename){
  if (filename.length() <= 3)
    return false;

  std::string suffix = filename.substr(filename.length()-3, 3);
  if (suffix== ".bt"){
    if (!m_octree->readBinary(filename)){
      return false;
    }
  } else if (suffix == ".ot"){
    AbstractOcTree* tree = AbstractOcTree::read(filename);
    if (!tree){
      return false;
    }
    if (m_octree){
      delete m_octree;
      m_octree = NULL;
    }
    m_octree = dynamic_cast<OcTreeT*>(tree);
    if (!m_octree){
      ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }

  } else{
    return false;
  }

  ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octree->size());

  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_res = m_octree->getResolution();
  m_gridmap.info.resolution = m_res;
  double minX, minY, minZ;
  double maxX, maxY, maxZ;
  m_octree->getMetricMin(minX, minY, minZ);
  m_octree->getMetricMax(maxX, maxY, maxZ);

  m_updateBBXMin[0] = m_octree->coordToKey(minX);
  m_updateBBXMin[1] = m_octree->coordToKey(minY);
  m_updateBBXMin[2] = m_octree->coordToKey(minZ);

  m_updateBBXMax[0] = m_octree->coordToKey(maxX);
  m_updateBBXMax[1] = m_octree->coordToKey(maxY);
  m_updateBBXMax[2] = m_octree->coordToKey(maxZ);

  publishAll();

  return true;

}

void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  ros::WallTime startTime = ros::WallTime::now();


  //
  // ground filtering in base frame
  //
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


  // set up filter for height range, also removes NANs:
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  if (m_filterGroundPlane || m_baseDistanceLimitPeriod > 0.0){
    try{
      m_tfListener.waitForTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, m_baseToWorldTf);
      m_baseToWorldValid = true;
    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM("Transform error when finding base to world transform: " << ex.what());
    }
  }

  if (m_filterGroundPlane){
    tf::StampedTransform sensorToBaseTf;
    try{
      m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                        "You need to set the base_frame_id or disable filter_ground.");
    }


    Eigen::Matrix4f sensorToBase, baseToWorld;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl_ros::transformAsMatrix(m_baseToWorldTf, baseToWorld);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToBase);
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);
    filterGroundPlane(pc, pc_ground, pc_nonground);

    // transform clouds to world frame for insertion
    pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
    pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
  } else {
    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }


  insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

  publishAll(cloud->header.stamp);
}

void OctomapServer::insertSegmentedCloudCallback(
    PointCloudSynchronizer::MsgTopicVectorConstPtr msgs_and_topics,
    const std::string& sensor_origin_frame_id,
    unsigned int callback_id)
{
  sensor_msgs::PointCloud2::ConstPtr ground_cloud;
  sensor_msgs::PointCloud2::ConstPtr nonground_cloud;
  sensor_msgs::PointCloud2::ConstPtr nonclearing_nonground_cloud;
  sensor_msgs::PointCloud2::ConstPtr nonmarking_nonground_cloud;

  PointCloudSynchronizer::MsgTopicConstVector::const_iterator it;
  for (it = msgs_and_topics->cbegin(); it != msgs_and_topics->cend(); ++it)
  {
    if (*((*it)->second) == m_callbackTopics[callback_id][0])
    {
      ground_cloud = (*it)->first;
    }
    else if (*((*it)->second) == m_callbackTopics[callback_id][1])
    {
      nonground_cloud = (*it)->first;
    }
    else if (*((*it)->second) == m_callbackTopics[callback_id][2])
    {
      nonclearing_nonground_cloud = (*it)->first;
    }
    else if (*((*it)->second) == m_callbackTopics[callback_id][3])
    {
      nonmarking_nonground_cloud = (*it)->first;
    }
  }

  this->insertSegmentedCloudCallback(
      ground_cloud,
      nonground_cloud,
      nonclearing_nonground_cloud,
      nonmarking_nonground_cloud,
      sensor_origin_frame_id,
      callback_id);
}

void OctomapServer::insertSegmentedCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& ground_cloud,
    const sensor_msgs::PointCloud2::ConstPtr& nonground_cloud,
    const sensor_msgs::PointCloud2::ConstPtr& nonclearing_nonground_cloud,
    const sensor_msgs::PointCloud2::ConstPtr& nonmarking_nonground_cloud,
    const std::string& sensor_origin_frame_id,
    unsigned int callback_id)
{
  if (m_callbackCounts[callback_id] < m_callbackSkipCount)
  {
    // skip the callback until we are at the skip count
    ++m_callbackCounts[callback_id];
    return;
  }
  m_callbackCounts[callback_id] = 0;

  ros::WallTime startTime = ros::WallTime::now();

  PCLPointCloud pc_ground;
  PCLPointCloud pc_nonground;
  PCLPointCloud pc_nonclearing_nonground;
  PCLPointCloud pc_nonmarking_nonground;
  bool first_cloud = true;
  ros::Time first_cloud_stamp;
  std::string first_cloud_frame;
  if (ground_cloud)
  {
    pcl::fromROSMsg(*ground_cloud, pc_ground);
    if (first_cloud)
    {
      first_cloud = false;
      first_cloud_stamp = ground_cloud->header.stamp;
      first_cloud_frame = ground_cloud->header.frame_id;
    }
  }
  if (nonground_cloud)
  {
    pcl::fromROSMsg(*nonground_cloud, pc_nonground);
    if (first_cloud)
    {
      first_cloud = false;
      first_cloud_stamp = nonground_cloud->header.stamp;
      first_cloud_frame = nonground_cloud->header.frame_id;
    }
  }
  if (nonclearing_nonground_cloud)
  {
    pcl::fromROSMsg(*nonclearing_nonground_cloud, pc_nonclearing_nonground);
    if (first_cloud)
    {
      first_cloud = false;
      first_cloud_stamp = nonclearing_nonground_cloud->header.stamp;
      first_cloud_frame = nonclearing_nonground_cloud->header.frame_id;
    }
  }
  if (nonmarking_nonground_cloud)
  {
    pcl::fromROSMsg(*nonmarking_nonground_cloud, pc_nonmarking_nonground);
    if (first_cloud)
    {
      first_cloud = false;
      first_cloud_stamp = nonmarking_nonground_cloud->header.stamp;
      first_cloud_frame = nonmarking_nonground_cloud->header.frame_id;
    }
  }

  if (m_baseDistanceLimitPeriod > 0.0){
    try{
      m_tfListener.waitForTransform(m_worldFrameId, m_baseFrameId, first_cloud_stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, first_cloud_stamp, m_baseToWorldTf);
      m_baseToWorldValid = true;
    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM("Transform error when finding base to world transform: " << ex.what());
    }
  }

  tf::StampedTransform sensorToWorldTf;
  tf::StampedTransform sensorOriginTf;
  // Assume exact time-synchronization, only lookup one TF
  try {
    m_tfListener.lookupTransform(m_worldFrameId, first_cloud_frame, first_cloud_stamp, sensorToWorldTf);
    std::string sensor_origin_frame = sensor_origin_frame_id.size() ? sensor_origin_frame_id : first_cloud_frame;
    m_tfListener.lookupTransform(m_worldFrameId, sensor_origin_frame, first_cloud_stamp, sensorOriginTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  bool filter_x = false;
  bool filter_y = false;
  bool filter_z = false;

  if (m_pointcloudMinX > -1.0e37 || m_pointcloudMaxX < 1.0e37)
  {
    filter_x = true;
  }
  if (m_pointcloudMinY > -1.0e37 || m_pointcloudMaxY < 1.0e37)
  {
    filter_y = true;
  }
  if (m_pointcloudMinZ > -1.0e37 || m_pointcloudMaxZ < 1.0e37)
  {
    filter_z = true;
  }

  // directly transform to map frame:
  pcl::transformPointCloud(pc_ground, pc_ground, sensorToWorld);
  pcl::transformPointCloud(pc_nonground, pc_nonground, sensorToWorld);
  pcl::transformPointCloud(pc_nonclearing_nonground, pc_nonclearing_nonground, sensorToWorld);
  pcl::transformPointCloud(pc_nonmarking_nonground, pc_nonmarking_nonground, sensorToWorld);

  if (filter_x)
  {
    pcl::PassThrough<PCLPoint> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pass_x.setInputCloud(pc_ground.makeShared());
    pass_x.filter(pc_ground);
    pass_x.setInputCloud(pc_nonground.makeShared());
    pass_x.filter(pc_nonground);
    pass_x.setInputCloud(pc_nonclearing_nonground.makeShared());
    pass_x.filter(pc_nonclearing_nonground);
    pass_x.setInputCloud(pc_nonmarking_nonground.makeShared());
    pass_x.filter(pc_nonmarking_nonground);
  }
  if (filter_y)
  {
    pcl::PassThrough<PCLPoint> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pass_y.setInputCloud(pc_ground.makeShared());
    pass_y.filter(pc_ground);
    pass_y.setInputCloud(pc_nonground.makeShared());
    pass_y.filter(pc_nonground);
    pass_y.setInputCloud(pc_nonclearing_nonground.makeShared());
    pass_y.filter(pc_nonclearing_nonground);
    pass_y.setInputCloud(pc_nonmarking_nonground.makeShared());
    pass_y.filter(pc_nonmarking_nonground);
  }
  if (filter_z)
  {
    pcl::PassThrough<PCLPoint> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
    pass_z.setInputCloud(pc_ground.makeShared());
    pass_z.filter(pc_ground);
    pass_z.setInputCloud(pc_nonground.makeShared());
    pass_z.filter(pc_nonground);
    pass_z.setInputCloud(pc_nonclearing_nonground.makeShared());
    pass_z.filter(pc_nonclearing_nonground);
    pass_z.setInputCloud(pc_nonmarking_nonground.makeShared());
    pass_z.filter(pc_nonmarking_nonground);
  }

  insertScan(sensorOriginTf.getOrigin(), pc_ground, pc_nonground, pc_nonclearing_nonground, pc_nonmarking_nonground);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

  publishAll(first_cloud_stamp);
}

void OctomapServer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground,
                          const PCLPointCloud& nonground, const PCLPointCloud& nonclearing_nonground,
                          const PCLPointCloud& nonmarking_nonground)
{
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);
  OcTreeKey originKey = m_octree->coordToKey(sensorOrigin);
  point3d originBoundary = m_octree->keyToCoord(originKey);

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
    || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

#ifdef COLOR_OCTOMAP_SERVER
  unsigned char* colors = new unsigned char[3];
#endif

  // instead of direct scan insertion, compute update to filter ground:
  bool floor_truncation_ = true;
  double floor_truncation_z_ = 0.0;
  if (floor_truncation_)
  {
    m_updateCells.setFloorTruncation(m_octree->coordToKey(floor_truncation_z_));
  }
  if (m_baseDistanceLimitPeriod > 0.0)
  {
    tf::Vector3 origin = m_baseToWorldTf.getOrigin();
    octomap::point3d base_position(origin.x(), origin.y(), origin.z());
    octomap::OcTreeKey minKey;
    octomap::OcTreeKey maxKey;
    m_octree->calculateBounds(m_update2DDistanceLimit, m_updateHeightLimit, m_updateDepthLimit, base_position,
                              &minKey, &maxKey);
    m_updateCells.setMinKey(minKey);
    m_updateCells.setMaxKey(maxKey);
  }

  updateMinKey(originKey, m_updateBBXMin);
  updateMaxKey(originKey, m_updateBBXMax);

  // insert non-ground points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it)
  {
    point3d point(it->x, it->y, it->z);
    handleRayPoint(&m_updateCells, sensorOrigin, point, false, true, false);
  }

  // insert ground points only as free, clearing up to them:
  for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it)
  {
    point3d point(it->x, it->y, it->z);
    handleRayPoint(&m_updateCells, sensorOrigin, point, true, false, false);
  }

  // insert non-marking, non-ground points: do not touch endpoint, clear ray
  for (PCLPointCloud::const_iterator it = nonmarking_nonground.begin(); it != nonmarking_nonground.end(); ++it)
  {
    point3d point(it->x, it->y, it->z);
    handleRayPoint(&m_updateCells, sensorOrigin, point, false, false, false);
  }

  // insert non-clearing, non-ground points: occupied only on endpoint:
  for (PCLPointCloud::const_iterator it = nonclearing_nonground.begin(); it != nonclearing_nonground.end(); ++it)
  {
    point3d point(it->x, it->y, it->z);
    handleRayPoint(&m_updateCells, sensorOrigin, point, false, true, true);
  }

  if (!m_deferUpdateToPublish)
  {
    applyUpdate();
  }

  // TODO: eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  //m_octree->updateInnerOccupancy();
  octomap::point3d minPt, maxPt;
  ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
//   if (m_maxTreeDepth < 16)
//   {
//      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
//      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
//      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      m_updateBBXMin = tmpMin;
//      m_updateBBXMax = tmpMax;
//   }

  // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
  minPt = m_octree->keyToCoord(m_updateBBXMin);
  maxPt = m_octree->keyToCoord(m_updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  // Prune map if past period
  bool pruned = false;
  ros::Time now = ros::Time::now();
  if (m_compressMap) {
    if (now >= m_compressLastTime + ros::Duration(m_compressPeriod)) {
      m_compressLastTime = now;
      m_octree->prune();
      pruned = true;
    }
  }

  // Expire if necessary, skip if we just did a pruning cycle
  // (We don't want to do both in one update, as they are both expensive)
  if (!pruned && m_expirePeriod > 0.0) {
    if (now >= m_expireLastTime + ros::Duration(m_expirePeriod)) {
      m_expireLastTime = now;
      m_octree->expireNodes(boost::bind(&OctomapServer::touchKeyAtDepth, this, _1, _2));
    }
  }

  // Delete based on distance periodically.
  if (m_baseDistanceLimitPeriod > 0.0)
  {
    if (m_baseToWorldValid && now >= m_baseDistanceLimitLastTime + ros::Duration(m_baseDistanceLimitPeriod)) {
      m_baseDistanceLimitLastTime = now;
      tf::Vector3 origin = m_baseToWorldTf.getOrigin();
      std::stringstream ss;
      ss << "Limiting ";
      if (m_base2DDistanceLimit < std::numeric_limits<octomap::key_type>::max()) {
        ss << "2D distance to " << m_base2DDistanceLimit;
      }
      if (m_baseHeightLimit < std::numeric_limits<octomap::key_type>::max()) {
        ss << " height to " << m_baseHeightLimit;
      }
      if (m_baseDepthLimit < std::numeric_limits<octomap::key_type>::max()) {
        ss << " depth to " << m_baseDepthLimit;
      }
      ss << " from (" << origin.x() << ", " << origin.y() << ", " << origin.z() << ")";
      ROS_DEBUG_STREAM(ss.str());
      octomap::point3d base_position(origin.x(), origin.y(), origin.z());
      m_octree->outOfBounds(m_base2DDistanceLimit, m_baseHeightLimit, m_baseDepthLimit, base_position,
          boost::bind(&OctomapServer::touchKeyAtDepth, this, _3, _4));
    }
  }


  publishAll(ros::Time::now());
#ifdef COLOR_OCTOMAP_SERVER
  if (colors)
  {
    delete[] colors;
    colors = NULL;
  }
#endif
}

void OctomapServer::handleRayPoint(SensorUpdateKeyMap* update_cells,
                                   const octomap::point3d& sensor_origin,
                                   const octomap::point3d& point,
                                   bool free,
                                   bool occupied,
                                   bool skip_tracing)
{
  // XXX get from params
  bool discrete = true;
  double ray_shrink_cells = 1.0;
  double post_mark_cells = 0.0;
  octomap::OcTreeKey furthest_key;

  // If discrete is enabled, pre-voxel-filter all points using a hash table of
  // octree cells (known as OcTreeKey in octomap). This speeds up performance
  // of fast sensors with normal FoV within the bounds of the update as
  // certain math can be skipped prior to checking the actual ray-trace
  // endpoints (because we trace only to the bounds of the update). This does
  // not speed up sensors as much whose endpoints are often out of the FoV, as
  // the math often has to be done to find the adjusted end point prior to
  // voxel filtering.
  if (discrete)
  {
    octomap::OcTreeKey point_key = m_octree->coordToKey(point);
    octomap::OcTreeKey::KeyHash hasher;
    size_t key_hash = hasher(point_key);
    SensorUpdateKeyMap::iterator it = m_voxelFilter.find(point_key, key_hash);
    if (it != m_voxelFilter.end())
    {
      bool was_occupied = it->value;
      if (was_occupied || !occupied)
      {
        // Discrete is set and we have already processed this point this
        // update cycle and the old point was occupied or the new point is not
        // ocupied. Nothing more to do.
        return;
      }
      else
      {
        // Else, the point was not occupied (yet we have seen it) and the new
        // point is occupied. Change the voxel memory to occupied.
        it->value = true;
      }
    }
    else
    {
      // Memorize this voxel as being used.
      m_voxelFilter.insert(point_key, key_hash, occupied);
    }
  }

  if (free)
  {
    // clear all the way to the end point
    ray_shrink_cells = 0.0;
  }

  if (update_cells->insertRay(*m_octree, sensor_origin, point, discrete,
                              free, occupied, skip_tracing,
                              m_maxRange, ray_shrink_cells, post_mark_cells, &furthest_key))
  {
    updateMinKey(furthest_key, m_updateBBXMin);
    updateMaxKey(furthest_key, m_updateBBXMax);
  }
}

void OctomapServer::applyUpdate()
{
  // update all cells per the accumulated update
  for (SensorUpdateKeyMap::iterator it = m_updateCells.begin(), end=m_updateCells.end(); it!= end; it++) {
    m_octree->updateNode(it->key, it->value);
  }
  // clear stored update
  m_updateCells.clear();
  // clear stored voxel filter
  m_voxelFilter.clear();
}

void OctomapServer::publishAll(const ros::Time& rostime){

  // Figure out which category to publish based on rate (if enabled)
  bool publish_updates = true;
  bool publish_3d = true;
  bool publish_2d = true;
  if (m_publish3DMapPeriod > 0.0 && rostime < m_publish3DMapLastTime + ros::Duration(m_publish3DMapPeriod)) {
    publish_3d = false;
  }
  if (m_publish3DMapUpdatePeriod > 0.0 && rostime < m_publish3DMapUpdateLastTime + ros::Duration(m_publish3DMapUpdatePeriod)) {
    publish_updates = false;
  }
  if (m_publish2DPeriod > 0.0 && rostime < m_publish2DLastTime + ros::Duration(m_publish2DPeriod)) {
    publish_2d = false;
  }
  if (!publish_2d && !publish_3d && !publish_updates)
  {
    // there is nothing to do.
    return;
  }

  if (m_deferUpdateToPublish)
  {
    applyUpdate();
  }

  if (publish_3d) {
    m_publish3DMapLastTime = rostime;
  }
  if (publish_updates) {
    m_publish3DMapUpdateLastTime = rostime;
  }
  if (publish_2d) {
    m_publish2DLastTime = rostime;
  }

  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
  bool publishBinaryMapUpdate = (m_latchedTopics || m_binaryMapUpdatePub.getNumSubscribers() > 0);
  bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
  bool publishFullMapUpdate = (m_latchedTopics || m_fullMapUpdatePub.getNumSubscribers() > 0);
  m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

  // Update above based on publish period booleans set above.
  if (!publish_3d)
  {
    publishFreeMarkerArray = false;
    publishMarkerArray = false;
    publishPointCloud = false;
    publishBinaryMap = false;
    publishFullMap = false;
  }
  if (!publish_updates)
  {
    publishFullMapUpdate = false;
    publishBinaryMapUpdate = false;
  }
  if (!publish_2d)
  {
    m_publish2DMap = false;
  }

  if (publishFullMapUpdate)
  {
    publishFullOctoMapUpdate(rostime);
  }

  if (publishBinaryMapUpdate)
  {
    publishBinaryOctoMapUpdate(rostime);
  }

  // XXX need to publish full/binary non-update maps

  if (!publishFreeMarkerArray &&
      !publishMarkerArray &&
      !publishPointCloud &&
      !publishBinaryMap &&
      !publishFullMap &&
      !m_publish2DMap)
  {
    // There is nothing else left to do.
    return;
  }

  if (m_useTimedMap && publishMarkerArray)
  {
    // Update the 'expiry' field for any nodes that have deferred expiration
    // to get correct color in the marker array
    m_octree->expireNodes(NodeChangeNotification(), false);
  }

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
      end = m_octree->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (m_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
          ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);


        //create marker:
        if (publishMarkerArray){
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          if (m_useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }

          if (m_useTimedMap){
            time_t expiry = it->getExpiry();
            time_t max_expiry_delta = m_octree->getMaxExpiryDelta();
            time_t now = m_octree->getLastUpdateTime();
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 0.0;
            if ( expiry < now ) {
              // doesn't make sense, so highlight pale yellow.
              color.r = 1.0;
              color.g = 1.0;
              color.b = 0.7;
            } else {
              double d;
              double d_max = static_cast<double>(max_expiry_delta);
              d = (expiry - now);
              if(d <= 60.0) {
                d = sqrt(d) / sqrt(60.0);
                color.r = 1.0;
                color.g = d;
              } else if(d <= 3600.0) {
                d = sqrt(d-60.0) / sqrt(3600.0-60.0);
                color.r = 1.0 - d;
                color.g = 1.0;
              } else if(d <= 4.0 * 3600.0) {
                d = sqrt(d-3600.0) / sqrt(3.0 * 3600.0);
                color.g = 1.0;
                color.b = d;
              } else if(d <= 16.0 * 3600.0) {
                d = sqrt(d-4.0*3600.0) / sqrt(12.0 * 3600.0);
                color.g = 1.0 - d;
                color.b = 1.0;
              } else if(d <= d_max) {
                double d_max = static_cast<double>(max_expiry_delta);
                d -= 16.0*3600.0;
                d_max -= 16.0*3600.0;
                color.b = 1.0;
                color.r = d / d_max;
              } else {
                // doesn't make sense, highlight lilac
                color.r = 1.0;
                color.g = 0.7;
                color.b = 1.0;
              }

//              d /= static_cast<double>(max_expiry_delta);
//              d = sqrt(d);
//              d *= m_colorFactor;
            }
            // use the same color maping as the height map using our
            // normalized and linearized expiration scale
//            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(d));
            occupiedNodesVis.markers[idx].colors.push_back(color);
          }

#ifdef COLOR_OCTOMAP_SERVER
          if (m_useColoredMap) {
            std_msgs::ColorRGBA _color; _color.r = (r / 255.); _color.g = (g / 255.); _color.b = (b / 255.); _color.a = 1.0; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
            occupiedNodesVis.markers[idx].colors.push_back(_color);
          }
#endif
        }

        // insert into pointcloud:
        if (publishPointCloud) {
#ifdef COLOR_OCTOMAP_SERVER
          PCLPoint _point = PCLPoint();
          _point.x = x; _point.y = y; _point.z = z;
          _point.r = r; _point.g = g; _point.b = b;
          pclCloud.push_back(_point);
#else
          pclCloud.push_back(PCLPoint(x, y, z));
#endif
        }

      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
      {
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);

        if (m_publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          //create marker for free space:
          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }

      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      if (!m_useColoredMap)
        occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);
  }


  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    m_fmarkerPub.publish(freeNodesVis);
  }


  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);
}


bool OctomapServer::octomapBinarySrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map))
    return false;

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool OctomapServer::octomapFullSrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();


  if (!octomap_msgs::fullMapToMsg(*m_octree, res.map))
    return false;

  return true;
}

bool OctomapServer::clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp){
  point3d min = pointMsgToOctomap(req.min);
  point3d max = pointMsgToOctomap(req.max);

  double thresMin = m_octree->getClampingThresMin();
  for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max),
      end=m_octree->end_leafs_bbx(); it!= end; ++it){

    it->setLogOdds(octomap::logodds(thresMin));
    //			m_octree->updateNode(it.getKey(), -6.0f);
  }
  // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
  m_octree->updateInnerOccupancy();

  publishAll(ros::Time::now());

  return true;
}

bool OctomapServer::eraseBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp){
  point3d min = pointMsgToOctomap(req.min);
  point3d max = pointMsgToOctomap(req.max);

  m_octree->deleteAABB(min, max, false,
                       std::bind(&OctomapServer::touchKeyAtDepth, this, std::placeholders::_3, std::placeholders::_4));

  publishAll(ros::Time::now());

  return true;
}

bool OctomapServer::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth +1);
  ros::Time rostime = ros::Time::now();
  m_octree->clear();
  // clear 2D map:
  m_gridmap.data.clear();
  m_gridmap.info.height = 0.0;
  m_gridmap.info.width = 0.0;
  m_gridmap.info.resolution = 0.0;
  m_gridmap.info.origin.position.x = 0.0;
  m_gridmap.info.origin.position.y = 0.0;

  ROS_INFO("Cleared octomap");
  publishAll(rostime);

  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  m_markerPub.publish(occupiedNodesVis);

  visualization_msgs::MarkerArray freeNodesVis;
  freeNodesVis.markers.resize(m_treeDepth +1);

  for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){

    freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
    freeNodesVis.markers[i].header.stamp = rostime;
    freeNodesVis.markers[i].ns = "map";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_fmarkerPub.publish(freeNodesVis);

  // Force full publish
  touchKeyAtDepth(octomap::OcTreeKey(), 0);
  return true;
}

void OctomapServer::onNewFullMapSubscription(const ros::SingleSubscriberPublisher& pub)
{
  publishAll();
  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = ros::Time::now();

  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    pub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void OctomapServer::onNewFullMapUpdateSubscription(const ros::SingleSubscriberPublisher& pub)
{
  publishFullOctoMapUpdate(ros::Time::now(), &pub);
}

void OctomapServer::onNewBinaryMapSubscription(const ros::SingleSubscriberPublisher& pub)
{
  publishAll();
  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = ros::Time::now();

  if (octomap_msgs::binaryMapToMsg(*m_octree, map))
    pub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void OctomapServer::onNewBinaryMapUpdateSubscription(const ros::SingleSubscriberPublisher& pub)
{
  publishBinaryOctoMapUpdate(ros::Time::now(), &pub);
}

void OctomapServer::publishBinaryOctoMapUpdate(const ros::Time& rostime, const ros::SingleSubscriberPublisher* pub /* =  nullptr */) {

  octomap_msgs::OctomapUpdatePtr map_msg_ptr(new octomap_msgs::OctomapUpdate());
  octomap_msgs::OctomapUpdate& map_msg = *map_msg_ptr;
  OcTreeT delta_map(m_res);

  // Set up header info
  map_msg.header.frame_id = m_worldFrameId;
  map_msg.header.stamp = rostime;
  map_msg.octomap_bounds.header.seq = m_binarySeq;
  map_msg.octomap_bounds.header.frame_id = m_worldFrameId;
  map_msg.octomap_bounds.header.stamp = rostime;
  map_msg.octomap_update.header.seq = m_binarySeq;
  map_msg.octomap_update.header.frame_id = m_worldFrameId;
  map_msg.octomap_update.header.stamp = rostime;

  OcTreeT* bounds_map_ptr = m_octree_binary_deltaBB_;
  if (pub)
  {
    // new subscription, force full publish, set seq unequal for sentinel
    bounds_map_ptr = m_universe;
    map_msg.octomap_update.header.seq--;
  }
  else
  {
    // publishing normally, increment our seq
    m_binarySeq++;
  }

  delta_map.setTreeValues(m_octree, bounds_map_ptr, false, false);

  if(   octomap_msgs::binaryMapToMsg(*bounds_map_ptr, map_msg.octomap_bounds)
     && octomap_msgs::binaryMapToMsg(delta_map, map_msg.octomap_update))
  {
    if (pub)
      pub->publish(map_msg_ptr);
    else
      m_binaryMapUpdatePub.publish(map_msg_ptr);
  }
  else
  {
    ROS_ERROR("Error serializing OctoMap Update");
  }

  if (!pub)
  {
    bounds_map_ptr->clear();
  }
}

void OctomapServer::publishFullOctoMapUpdate(const ros::Time& rostime, const ros::SingleSubscriberPublisher* pub /* =  nullptr */) {

  octomap_msgs::OctomapUpdatePtr map_msg_ptr(new octomap_msgs::OctomapUpdate());
  octomap_msgs::OctomapUpdate& map_msg = *map_msg_ptr;
  OcTreeT delta_map(m_res);

  // Set up header info
  map_msg.header.frame_id = m_worldFrameId;
  map_msg.header.stamp = rostime;
  map_msg.octomap_bounds.header.seq = m_fullSeq;
  map_msg.octomap_bounds.header.frame_id = m_worldFrameId;
  map_msg.octomap_bounds.header.stamp = rostime;
  map_msg.octomap_update.header.seq = m_fullSeq;
  map_msg.octomap_update.header.frame_id = m_worldFrameId;
  map_msg.octomap_update.header.stamp = rostime;

  OcTreeT* bounds_map_ptr = m_octree_deltaBB_;
  if (pub)
  {
    // new subscription, force full publish, set seq unequal for sentinel
    bounds_map_ptr = m_universe;
    map_msg.octomap_update.header.seq--;
  }
  else
  {
    // publishing normally, increment our seq
    m_fullSeq++;
  }

  delta_map.setTreeValues(m_octree, bounds_map_ptr, false, false);

  if(   octomap_msgs::binaryMapToMsg(*bounds_map_ptr, map_msg.octomap_bounds)
     && octomap_msgs::fullMapToMsg(delta_map, map_msg.octomap_update))
  {
    if(pub)
      pub->publish(map_msg_ptr);
    else
      m_fullMapUpdatePub.publish(map_msg_ptr);
  }
  else
  {
    ROS_ERROR("Error serializing OctoMap Update");
  }

  if (!pub)
  {
    bounds_map_ptr->clear();
  }
}


void OctomapServer::filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const{
  ground.header = pc.header;
  nonground.header = pc.header;

  if (pc.size() < 50){
    ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
    nonground = pc;
  } else {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PCLPoint> seg;
    seg.setOptimizeCoefficients (true);
    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold (m_groundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(m_groundFilterAngle);


    PCLPointCloud cloud_filtered(pc);
    // Create the filtering object
    pcl::ExtractIndices<PCLPoint> extract;
    bool groundPlaneFound = false;

    while(cloud_filtered.size() > 10 && !groundPlaneFound){
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_INFO("PCL segmentation did not find any plane.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative (false);
        extract.filter (ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        groundPlaneFound = true;
      } else{
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        pcl::PointCloud<PCLPoint> cloud_out;
        extract.setNegative (false);
        extract.filter(cloud_out);
        nonground +=cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        } else{
          cloud_filtered.points.clear();
        }
      }

    }
    // TODO: also do this if overall starting pointcloud too small?
    if (!groundPlaneFound){ // no plane found or remaining points too small
      ROS_WARN("No ground plane found in scan");

      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<PCLPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);

      second_pass.setFilterLimitsNegative (true);
      second_pass.filter(nonground);
    }

    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<PCLPoint>("nonground.pcd",pc_nonground, false);

  }


}

void OctomapServer::handlePreNodeTraversal(const ros::Time& rostime){
  if (m_publish2DMap){
    // init projected 2D map:
    m_gridmap.header.frame_id = m_worldFrameId;
    m_gridmap.header.stamp = rostime;
    nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
    octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

    // add padding if requested (= new min/maxPts in x&y):
    double halfPaddedX = 0.5*m_minSizeX;
    double halfPaddedY = 0.5*m_minSizeY;
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    minPt = octomap::point3d(minX, minY, minZ);
    maxPt = octomap::point3d(maxX, maxY, maxZ);

    OcTreeKey paddedMaxKey;
    if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)){
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)){
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
    m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
    m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

    int mapOriginX = minKey[0] - m_paddedMinKey[0];
    int mapOriginY = minKey[1] - m_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
    double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
    m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
    m_gridmap.info.resolution = gridRes;
    m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
    m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
    if (m_maxTreeDepth != m_treeDepth){
      m_gridmap.info.origin.position.x -= m_res/2.0;
      m_gridmap.info.origin.position.y -= m_res/2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (m_maxTreeDepth < m_treeDepth)
      m_projectCompleteMap = true;


    if(m_projectCompleteMap){
      ROS_DEBUG("Rebuilding complete 2D map");
      m_gridmap.data.clear();
      // init to unknown:
      m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

    } else {

       if (mapChanged(oldMapInfo, m_gridmap.info)){
          ROS_DEBUG("2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
          adjustMapData(m_gridmap, oldMapInfo);
       }
       nav_msgs::OccupancyGrid::_data_type::iterator startIt;
       size_t mapUpdateBBXMinX = std::max(0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
       size_t mapUpdateBBXMinY = std::max(0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));
       size_t mapUpdateBBXMaxX = std::min(int(m_gridmap.info.width-1), (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
       size_t mapUpdateBBXMaxY = std::min(int(m_gridmap.info.height-1), (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));

       assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
       assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

       size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

       // test for max idx:
       uint max_idx = m_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
       if (max_idx  >= m_gridmap.data.size())
         ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, m_gridmap.data.size(), m_gridmap.info.width, m_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

       // reset proj. 2D map in bounding box:
       for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
          std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width*j+mapUpdateBBXMinX,
                      numCols, -1);
       }

    }



  }

}

void OctomapServer::handlePostNodeTraversal(const ros::Time& rostime){

  if (m_publish2DMap)
    m_mapPub.publish(m_gridmap);
}

void OctomapServer::handleOccupiedNode(const OcTreeT::iterator& it){

  if (m_publish2DMap && m_projectCompleteMap){
    update2DMap(it, true);
  }
}

void OctomapServer::handleFreeNode(const OcTreeT::iterator& it){

  if (m_publish2DMap && m_projectCompleteMap){
    update2DMap(it, false);
  }
}

void OctomapServer::handleOccupiedNodeInBBX(const OcTreeT::iterator& it){

  if (m_publish2DMap && !m_projectCompleteMap){
    update2DMap(it, true);
  }
}

void OctomapServer::handleFreeNodeInBBX(const OcTreeT::iterator& it){

  if (m_publish2DMap && !m_projectCompleteMap){
    update2DMap(it, false);
  }
}

void OctomapServer::update2DMap(const OcTreeT::iterator& it, bool occupied){

  // update 2D map (occupied always overrides):

  if (it.getDepth() == m_maxTreeDepth){
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
      m_gridmap.data[mapIdx(it.getKey())] = 100;
    else if (m_gridmap.data[idx] == -1){
      m_gridmap.data[idx] = 0;
    }

  } else{
    int intSize = 1 << (m_maxTreeDepth - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);
        if (occupied)
          m_gridmap.data[idx] = 100;
        else if (m_gridmap.data[idx] == -1){
          m_gridmap.data[idx] = 0;
        }
      }
    }
  }


}



bool OctomapServer::isSpeckleNode(const OcTreeKey&nKey) const {
  OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          OcTreeNode* node = m_octree->search(key);
          if (node && m_octree->isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}

void OctomapServer::reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level){
  if (m_maxTreeDepth != unsigned(config.max_depth))
    m_maxTreeDepth = unsigned(config.max_depth);
  else{
    m_pointcloudMinZ            = config.pointcloud_min_z;
    m_pointcloudMaxZ            = config.pointcloud_max_z;
    m_occupancyMinZ             = config.occupancy_min_z;
    m_occupancyMaxZ             = config.occupancy_max_z;
    m_filterSpeckles            = config.filter_speckles;
    m_filterGroundPlane         = config.filter_ground;
    m_compressMap               = config.compress_map;
    m_incrementalUpdate         = config.incremental_2D_projection;

    // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
    // will overwrite them because the server is not able to match parameters' names.
    if (m_initConfig){
		// If parameters do not have the default value, dynamic reconfigure server should be updated.
		if(!is_equal(m_groundFilterDistance, 0.04))
          config.ground_filter_distance = m_groundFilterDistance;
		if(!is_equal(m_groundFilterAngle, 0.15))
          config.ground_filter_angle = m_groundFilterAngle;
	    if(!is_equal( m_groundFilterPlaneDistance, 0.07))
          config.ground_filter_plane_distance = m_groundFilterPlaneDistance;
        if(!is_equal(m_maxRange, -1.0))
          config.sensor_model_max_range = m_maxRange;
        if(!is_equal(m_octree->getProbHit(), 0.7))
          config.sensor_model_hit = m_octree->getProbHit();
	    if(!is_equal(m_octree->getProbMiss(), 0.4))
          config.sensor_model_miss = m_octree->getProbMiss();
		if(!is_equal(m_octree->getClampingThresMin(), 0.12))
          config.sensor_model_min = m_octree->getClampingThresMin();
		if(!is_equal(m_octree->getClampingThresMax(), 0.97))
          config.sensor_model_max = m_octree->getClampingThresMax();
        m_initConfig = false;

	    boost::recursive_mutex::scoped_lock reconf_lock(m_config_mutex);
        m_reconfigureServer.updateConfig(config);
    }
    else{
	  m_groundFilterDistance      = config.ground_filter_distance;
      m_groundFilterAngle         = config.ground_filter_angle;
      m_groundFilterPlaneDistance = config.ground_filter_plane_distance;
      m_maxRange                  = config.sensor_model_max_range;
      m_octree->setClampingThresMin(config.sensor_model_min);
      m_octree->setClampingThresMax(config.sensor_model_max);

     // Checking values that might create unexpected behaviors.
      if (is_equal(config.sensor_model_hit, 1.0))
		config.sensor_model_hit -= 1.0e-6;
      m_octree->setProbHit(config.sensor_model_hit);
	  if (is_equal(config.sensor_model_miss, 0.0))
		config.sensor_model_miss += 1.0e-6;
      m_octree->setProbMiss(config.sensor_model_miss);
	}
  }
  publishAll();
}

void OctomapServer::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const{
  if (map.info.resolution != oldMapInfo.resolution){
    ROS_ERROR("Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
  int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);

  if (i_off < 0 || j_off < 0
      || oldMapInfo.width  + i_off > map.info.width
      || oldMapInfo.height + j_off > map.info.height)
  {
    ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

  for (int j =0; j < int(oldMapInfo.height); ++j ){
    // copy chunks, row by row:
    fromStart = oldMapData.begin() + j*oldMapInfo.width;
    fromEnd = fromStart + oldMapInfo.width;
    toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
    copy(fromStart, fromEnd, toStart);

//    for (int i =0; i < int(oldMapInfo.width); ++i){
//      map.data[m_gridmap.info.width*(j+j_off) +i+i_off] = oldMapData[oldMapInfo.width*j +i];
//    }

  }
}

void OctomapServer::touchKeyAtDepth(const OcTreeKey& key, unsigned int depth /* = MAX_INT */)
{
  m_octree_deltaBB_->setNodeValueAtDepth(key, depth, m_octree_deltaBB_->getClampingThresMaxLog());
  m_octree_binary_deltaBB_->setNodeValueAtDepth(key, depth, m_octree_binary_deltaBB_->getClampingThresMaxLog());
}

// for convenience
void OctomapServer::touchKey(const OcTreeKey& key)
{
  touchKeyAtDepth(key);
}

void OctomapServer::enableChangeCallback()
{
  m_octree->enableChangeDetection(true);
  m_octree->registerValueChangeCallback(boost::bind(&OctomapServer::valueChangeCallback, this,
      _1, _2, _3, _4, _5, _6, _7));
}

void OctomapServer::disableChangeCallback()
{
  m_octree->unregisterValueChangeCallback();
  m_octree->enableChangeDetection(false);
}

void OctomapServer::valueChangeCallback(const OcTreeKey& key, unsigned int depth, const bool node_just_created,
      const float prev_full_val, const bool prev_binary_val,
      const float curr_full_val, const bool curr_binary_val){
  if (prev_binary_val != curr_binary_val || node_just_created)
    m_octree_binary_deltaBB_->setNodeValueAtDepth(key, depth, m_octree_binary_deltaBB_->getClampingThresMaxLog());
  m_octree_deltaBB_->setNodeValueAtDepth(key, depth, m_octree_deltaBB_->getClampingThresMaxLog());
}

std_msgs::ColorRGBA OctomapServer::heightMapColor(double h) {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}
}



