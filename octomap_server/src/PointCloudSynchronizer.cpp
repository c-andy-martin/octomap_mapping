#include <octomap_server/PointCloudSynchronizer.h>

namespace octomap_server
{

void PointCloudSynchronizer::addTopic(const std::string& topic_name, const std::string& target_frame)
{
  TopicConstPtr topic(new Topic(topic_name));
  FrameConstPtr frame(new Frame(target_frame));
  topics_and_frames_.push_back(std::make_pair(topic, frame));
}

void PointCloudSynchronizer::registerCallback(PointCloudSynchronizer::Callback callback)
{
  callback_ = callback;

  tf_filters_.clear();
  subs_.clear();
  // Subscribe to our input topics and use TF message filters to the target frame.
  std::vector<std::pair<TopicConstPtr, FrameConstPtr>>::const_iterator it;
  for (it = topics_and_frames_.begin(); it != topics_and_frames_.end(); ++it)
  {
    boost::shared_ptr<message_filters::Subscriber<Msg>> sub(
      new message_filters::Subscriber<Msg> (
        nh_,
        *(it->first),
        queue_size_
      )
    );
    subs_.push_back(sub);
    boost::shared_ptr<tf::MessageFilter<Msg>> tf_filter(
      new tf::MessageFilter<Msg> (
        *sub,
        tf_,
        *(it->second),
        queue_size_,
        nh_
      )
    );
    tf_filters_.push_back(tf_filter);
    tf_filter->registerCallback(boost::bind(&PointCloudSynchronizer::topicCallback, this, _1, it->first));
  }
}

void PointCloudSynchronizer::topicCallback(PointCloudSynchronizer::MsgConstPtr msg_ptr,
                                           PointCloudSynchronizer::TopicConstPtr topic_ptr)
{
  const ros::Time& stamp(msg_ptr->header.stamp);
  MsgTopicConstPtr msg_topic(new MsgTopicConstPair(msg_ptr, topic_ptr));
  TimeToMsgTopicMap::const_iterator it = map_.find(stamp);
  if (it != map_.cend())
  {
    it->second->push_back(msg_topic);
  }
  else
  {
    MsgTopicVectorPtr vec(new MsgTopicConstVector());
    vec->push_back(msg_topic);
    it = map_.insert(std::make_pair(stamp, vec)).first;
  }

  if (it->second->size() == topics_and_frames_.size())
  {
    // Received a message on each registered topic
    MsgTopicVectorConstPtr const_vec(it->second);
    callback_(const_vec);
    // Erase this message and all older messages.
    map_.erase(map_.begin(), it);
  }
  enforceQueueSize();
}

void PointCloudSynchronizer::enforceQueueSize()
{
  // If our queue is too big, make space for a new message
  while (map_.size() >= queue_size_)
  {
    map_.erase(map_.begin());
  }
}

}  // namespace octomap_server
