#include <ros/ros.h>
#include <octomap_server/OcTreeStampedWithExpiry.h>

namespace octomap_server {

OcTreeStampedWithExpiry::OcTreeStampedWithExpiry(double resolution)
  : OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry>(resolution)
  , a_coeff_(1.0 / 25.0)
  , c_coeff_(2.0)
  , quadratic_start_(30.0)
  , c_coeff_free_(60.0*60.0*18.0)
  , last_expire_time(0)
{
  ocTreeStampedWithExpiryMemberInit.ensureLinking();
}

void OcTreeStampedWithExpiry::expireNodes(NodeChangeNotification change_notification /* = NodeChangeNotification() */)
{
  octomap::OcTreeKey rootKey(this->tree_max_val, this->tree_max_val, this->tree_max_val);
  last_expire_time = ros::Time::now().sec;

  // pre-compute a_coeff in terms of log-odds instead of number of observations
  a_coeff_log_odds_ = a_coeff_ * (1.0 / prob_hit_log) * (1.0 / prob_hit_log);
  quadratic_start_log_odds_ = quadratic_start_ * prob_hit_log;

  if (root != NULL)
  {
    ROS_INFO("prior to expiry, root expiry was: %ld.", root->getExpiry());
    expire_count = 0;
    if (expireNodeRecurs(root, rootKey, 0,change_notification))
    {
      // The whole tree expired. This is odd but possible if no sensor data
      // has been received. It is odd enough to log this.
      ROS_WARN("Entire octree expired!");
      if(change_notification)
      {
        change_notification(rootKey, 0);
      }
      delete root;
      root = NULL;
      expire_count++;
    }
    ROS_INFO("Expired %d nodes. Last expiry time is %ld.", expire_count, last_expire_time);
    if (root)
    {
      ROS_INFO("Next node expires at %ld.", root->getExpiry());
    }
  }
}

bool OcTreeStampedWithExpiry::expireNodeRecurs(OcTreeNodeStampedWithExpiry* node,
                                               const octomap::OcTreeKey& key,
                                               int depth,
                                               NodeChangeNotification change_notification /* = NodeChangeNotification() */)
{
  octomap::key_type center_offset_key = this->tree_max_val >> (depth + 1);
  // We can prune our search tree using the stored expiry.
  // If we encounter an expiry of zero, that indicates a deferred calculation.
  // Calculate the expiration of such nodes as they are encountered, being
  // sure to update the inner nodes when necessary.

  // For now, only expire occupied nodes
//  if (isNodeOccupied(node))
  {
    time_t expiry = node->getExpiry();
    // For inner nodes, expiry is the minimum of all child nodes expiry's.
    // For nodes which have not yet had expiry calculated, expiry will be zero
    // If this node (or any child) has expired or this node (or any child) has
    // not had expiry set, decend in the search.
    if (expiry <= last_expire_time)
    {
      // This is an inner node
      if (nodeHasChildren(node))
      {
        // Update all children first
        for (unsigned int i=0; i<8; i++)
        {
          if (nodeChildExists(node, i))
          {
            octomap::OcTreeKey child_key;
            computeChildKey(i, center_offset_key, key, child_key);
            if (expireNodeRecurs(getNodeChild(node, i), child_key, depth+1, change_notification))
            {
              // Delete the child node
              deleteNodeChild(node, i);
              expire_count++;
            }
          }
        }
        // If we have no more children left, this inner node has expired too
        if (!nodeHasChildren(node))
        {
          // Curiously, deleteNodeChild does not clean up the dynamic array
          // for the children pointers when the child count drops to zero.
          // pruneNode does this, but is not what we want (we want to drop the
          // expired data, not merge it up the tree!). So prevent leaking
          // memory be ensuring this inner node deletes the children pointer
          // storage before the caller deletes us!
          node->deleteNodeChildren();
          return true;
        }
        // Update the inner node's expiry to track the min of all children
        node->updateOccupancyChildren();
      }
      else
      {
        // Leaf, update expiry if 0
        if (expiry == 0)
        {
          const double value = node->getLogOdds();
          if (value < occ_prob_thres_log)
          {
            // free space
            expiry = node->getTimestamp() + c_coeff_free_;
          }
          else
          {
            // occupied space
            expiry = node->getTimestamp() + c_coeff_;
            const double v = (value - quadratic_start_log_odds_);
            if (v > 0.0)
            {
              expiry += a_coeff_log_odds_ * v * v;
            }
          }
          node->setExpiry(expiry);
          /*
          if (expiry <= last_expire_time)
          {
            ROS_WARN_THROTTLE(1.0, "newly added node immediately expired! (ts: %ld, expiry: %ld, value: %f)", node->getTimestamp(), expiry, value);
          }
          */
        }
        if (expiry <= last_expire_time)
        {
          //ROS_INFO_THROTTLE(1.0, "child node expired: value: %f expiry: %ld ts: %ld", node->getLogOdds(), expiry, node->getTimestamp());
          // We have expired!
          change_notification(key, depth);
          return true;
        }
      }
    }
  }

  return false;
}

void OcTreeStampedWithExpiry::calculateBounds(double xy_distance,
                                              double z_height,
                                              double z_depth,
                                              const octomap::point3d& base_position,
                                              octomap::OcTreeKey* min_key,
                                              octomap::OcTreeKey* max_key)
{
  // Find the (clamped) min and max keys.
  *min_key = this->coordToKeyClamped(base_position.x() - xy_distance,
                                     base_position.y() - xy_distance,
                                     base_position.z() - z_depth);
  *max_key = this->coordToKeyClamped(base_position.x() + xy_distance,
                                     base_position.y() + xy_distance,
                                     base_position.z() + z_height);
}

void OcTreeStampedWithExpiry::outOfBounds(double xy_distance,
                                          double z_height,
                                          double z_depth,
                                          const octomap::point3d& base_position,
                                          DeletionCallback change_notification /* = NodeChangeNotification() */)
{
  octomap::OcTreeKey rootKey(this->tree_max_val, this->tree_max_val, this->tree_max_val);
  octomap::OcTreeKey minKey;
  octomap::OcTreeKey maxKey;
  calculateBounds(xy_distance, z_height, z_depth, base_position, &minKey, &maxKey);
  ROS_INFO_STREAM("Limiting to min key (" << minKey[0] << ", " << minKey[1] << ", " << minKey[2] <<
                    "), max key (" << maxKey[0] << ", " << maxKey[1] << ", " << maxKey[2] << ")");
  if (root != NULL)
  {
    deleteAABB(minKey, maxKey, true, change_notification);
  }
}

OcTreeNodeStampedWithExpiry* OcTreeStampedWithExpiry::updateNode(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval)
{
  // early abort (no change will happen).
  // may cause an overhead in some configuration, but more often helps
  OcTreeNodeStampedWithExpiry* leaf = this->search(key);
  // no change: node already at threshold
  if (leaf
      && ((log_odds_update >= 0 && leaf->getLogOdds() >= this->clamping_thres_max)
      || ( log_odds_update <= 0 && leaf->getLogOdds() <= this->clamping_thres_min))
      && (leaf->getTimestamp() == getLastUpdateTime()))
  {
    return leaf;
  }

  bool createdRoot = false;
  if (this->root == NULL) {
    this->root = new OcTreeNodeStampedWithExpiry();
    this->tree_size++;
    createdRoot = true;
  }

  return OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry>::updateNodeRecurs(this->root, createdRoot, key, 0, log_odds_update, lazy_eval);
}

void OcTreeStampedWithExpiry::updateNodeLogOdds(OcTreeNodeStampedWithExpiry* node, const float& update) const
{
  // Update value based on expiry if present
  // This will rarely be present, only if we haven't seen this node recently
  time_t expiry = node->getExpiry();
  // For now, only decay occupied nodes. Free ones have such a simple timeout
  // mechanism that decaying the timeout prior to a miss is practically
  // useless, so avoid the computational cost for free nodes.
  if (expiry != 0 && isNodeOccupied(node))
  {
    time_t orig_delta_t = expiry - node->getTimestamp();
    time_t curr_delta_t = expiry - last_expire_time;
    if (curr_delta_t <= 0)
    {
      // Its already expired, set it back to the background value prior to update
      node->setLogOdds(occ_prob_thres_log);
    }
    else
    {
      // Decay the value towards the background by an amount proportional to the remaining time
      double decay_factor = ((double)curr_delta_t)/((double)orig_delta_t);
      double logodds_delta = node->getLogOdds() - occ_prob_thres_log;
      node->setLogOdds(occ_prob_thres_log + logodds_delta * decay_factor);
    }
    // The above code should not be able to alter whether the node was occupied
    assert(isNodeOccupied(node));
  }
  OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry>::updateNodeLogOdds(node, update);
  node->setTimestamp(last_expire_time);
  // Because we will very likely observe the same space multiple times, we do
  // not want to update expiry just to have to update it again on the next
  // sensor cycle. Instead, set it to zero and re-calculate it when it is
  // needed.
  node->setExpiry(0);
}

void OcTreeStampedWithExpiry::expandNode(OcTreeStampedWithExpiry::NodeType* node)
{
  bool old_size_changed = size_changed;
  octomap::OccupancyOcTreeBase<NodeType>::expandNode(node);
  // This should really be fixed upstream, if it ever is, we can remove
  if (!old_size_changed && size_changed)
  {
    // override size_changed back to false, expanding can't alter extents
    size_changed = false;
  }
}

bool OcTreeStampedWithExpiry::pruneNode(OcTreeStampedWithExpiry::NodeType* node)
{
  bool old_size_changed = size_changed;
  bool rv = octomap::OccupancyOcTreeBase<NodeType>::pruneNode(node);
  // This should really be fixed upstream, if it ever is, we can remove
  if (!old_size_changed && size_changed)
  {
    // override size_changed back to false, pruning can't alter extents
    size_changed = false;
  }
  return rv;
}

OcTreeStampedWithExpiry::StaticMemberInitializer OcTreeStampedWithExpiry::ocTreeStampedWithExpiryMemberInit;

} // end namespace
