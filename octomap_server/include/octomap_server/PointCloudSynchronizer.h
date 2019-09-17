#ifndef OCTOMAP_SERVER_POINT_CLOUD_SYNCHRONIZER_H
#define OCTOMAP_SERVER_POINT_CLOUD_SYNCHRONIZER_H

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

namespace octomap_server
{

/* Exact time synchronizes any number of point cloud topics.
 * Uses a tf::MessageFilter on the topics.
 * All topics must be point cloud topics.
 * The callback will be passed a MsgTopicVectorConstPtr which contains
 * a vector of pairs of PointCloud2 messsage and the associated topic name.
 * The callback can use the topic name to know which point cloud the topic
 * came from, if that is important.
 */
class PointCloudSynchronizer
{
public:
  typedef sensor_msgs::PointCloud2 Msg;
  typedef boost::shared_ptr<Msg> MsgPtr;
  typedef boost::shared_ptr<const Msg> MsgConstPtr;
  typedef std::string Topic;
  typedef std::string Frame;
  typedef boost::shared_ptr<const Topic> TopicConstPtr;
  typedef boost::shared_ptr<const Frame> FrameConstPtr;
  typedef std::pair<MsgConstPtr, TopicConstPtr> MsgTopicConstPair;
  typedef boost::shared_ptr<const MsgTopicConstPair> MsgTopicConstPtr;
  typedef std::vector<MsgTopicConstPtr> MsgTopicConstVector;
  typedef boost::shared_ptr<MsgTopicConstVector> MsgTopicVectorPtr;
  typedef boost::shared_ptr<const MsgTopicConstVector> MsgTopicVectorConstPtr;
  typedef std::map<ros::Time, MsgTopicVectorPtr> TimeToMsgTopicMap;

  typedef boost::function<void(MsgTopicVectorConstPtr)> Callback;

  PointCloudSynchronizer(ros::NodeHandle nh, tf::Transformer& tf, uint32_t queue_size)
  : nh_(nh), tf_(tf), queue_size_(queue_size) {}

  ~PointCloudSynchronizer()
  {
    // Ensure that TF filters are destroyed before the subscribers.
    tf_filters_.clear();
  }

  // Doesn't subscribe to the topic until registerCallback is called to
  // avoid races.
  void addTopic(const std::string& topic_name, const std::string& target_frame);

  // Adds the callback to call when all topics are received, and then
  // subscribes to all input topics.
  void registerCallback(Callback callback);

private:
  void topicCallback(MsgConstPtr msg_ptr, TopicConstPtr topic_ptr);

  void enforceQueueSize();

private:
  ros::NodeHandle nh_;
  tf::Transformer& tf_;
  uint32_t queue_size_;
  std::vector<std::pair<TopicConstPtr, FrameConstPtr>> topics_and_frames_;
  TimeToMsgTopicMap map_;
  Callback callback_;
  std::vector<boost::shared_ptr<message_filters::Subscriber<Msg>>> subs_;
  std::vector<boost::shared_ptr<tf::MessageFilter<Msg>>> tf_filters_;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SERVER_POINT_CLOUD_SYNCHRONIZER_H

