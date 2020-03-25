#include <ros/ros.h>
#include <octomap_server/OcTreeStampedWithExpiry.h>

namespace octomap_server {

OcTreeStampedWithExpiry::OcTreeStampedWithExpiry(double resolution)
  : super(resolution)
  , a_coeff_(1.0 / 25.0)
  , c_coeff_(2.0)
  , quadratic_start_(30.0)
  , c_coeff_free_(60.0*60.0*18.0)
  , last_expire_time(0)
  , delete_minimum(false)
{
  ocTreeStampedWithExpiryMemberInit.ensureLinking();
}

void OcTreeStampedWithExpiry::expireNodes(NodeChangeNotification change_notification /* = NodeChangeNotification() */,
                                          bool delete_expired_nodes /* = true */)
{
  octomap::OcTreeKey rootKey(this->tree_max_val, this->tree_max_val, this->tree_max_val);
  last_expire_time = ros::Time::now().sec;

  // pre-compute a_coeff in terms of log-odds instead of number of observations
  a_coeff_log_odds_ = a_coeff_ * (1.0 / prob_hit_log) * (1.0 / prob_hit_log);
  quadratic_start_log_odds_ = quadratic_start_ * prob_hit_log;

  if (root != NULL)
  {
    ROS_DEBUG("prior to expiry, root expiry was: %ld.", root->getExpiry());
    expire_count = 0;
    if (expireNodeRecurs(root, rootKey, 0, change_notification, delete_expired_nodes))
    {
      // The whole tree expired. This is odd but possible if no sensor data
      // has been received. It is odd enough to log this.
      ROS_WARN("Entire octree expired!");
      if(change_notification)
      {
        change_notification(rootKey, 0);
      }
      this->deleteNodeRecurs(root);
      root = NULL;
      expire_count++;
    }
    ROS_DEBUG("Expired %d nodes. Last expiry time is %ld.", expire_count, last_expire_time);
    if (root)
    {
      ROS_DEBUG("Next node expires at %ld.", root->getExpiry());
    }
  }
}

bool OcTreeStampedWithExpiry::expireNodeRecurs(OcTreeNodeStampedWithExpiry* node,
                                               const octomap::OcTreeKey& key,
                                               int depth,
                                               const NodeChangeNotification& change_notification,
                                               bool delete_expired_nodes /* = true */)
{
  octomap::key_type center_offset_key = octomap::computeCenterOffsetKey(depth, this->tree_max_val);
  // We can prune our search tree using the stored expiry.
  // If we encounter an expiry of zero, that indicates a deferred calculation.
  // Calculate the expiration of such nodes as they are encountered, being
  // sure to update the inner nodes when necessary.

  // For now, only expire occupied nodes
//  if (isNodeOccupied(node))
  {
    time_t expiry = node->getExpiry();
    // If we are not deleting, only update those nodes which have no expiry.
    if (!delete_expired_nodes && expiry != 0)
    {
      return false;
    }
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
            if (expireNodeRecurs(getNodeChild(node, i), child_key, depth+1, change_notification, delete_expired_nodes))
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
        if (delete_expired_nodes && expiry <= last_expire_time)
        {
          // We have expired!
          change_notification(key, depth);
          return true;
        }
      }
    }
  }

  return false;
}

void OcTreeStampedWithExpiry::applyUpdate(const SensorUpdateKeyMap& update)
{
  octomap::OcTreeKey root_key(tree_max_val, tree_max_val, tree_max_val);

  bool created_root = false;
  if (root == nullptr)
  {
    root = allocNode();
    tree_size++;
    created_root = true;
  }
  if (applyUpdateRecurs(update, root, created_root, root_key, 0, tree_max_val >> 1))
  {
    deleteNodeRecurs(root);
    root = nullptr;
  }
}

bool OcTreeStampedWithExpiry::applyUpdateRecurs(
    const SensorUpdateKeyMap& update,
    OcTreeStampedWithExpiry::NodeType* node,
    bool node_just_created,
    const octomap::OcTreeKey& key,
    unsigned int depth,
    octomap::key_type center_offset_key)
{
  assert(node != nullptr);
  assert(depth < tree_depth);
  unsigned int next_depth = depth + 1;

  if (!node_just_created && !nodeHasChildren(node))
  {
    // Expand the node. We will (likely) need to update our children.
    expandNode(node);
  }

  const octomap::key_type next_center_offset_key = center_offset_key >> 1;
  for (unsigned int i=0; i<8; ++i)
  {
    octomap::OcTreeKey child_key;
    octomap::computeChildKey(i, center_offset_key, key, child_key);
    const VoxelState child_voxel_state = update.find(child_key, next_depth);

    if (child_voxel_state == voxel_state::UNKNOWN)
    {
      // There is nothing to update in this direction.
      continue;
    }

    bool voxel_is_leaf = ((child_voxel_state & voxel_state::INNER) == 0);
    bool voxel_free_bit = ((child_voxel_state & voxel_state::FREE) == voxel_state::FREE);
    bool voxel_occupied_bit = ((child_voxel_state & voxel_state::OCCUPIED) == voxel_state::OCCUPIED);

    NodeType* child_node;
    bool child_created = false;
    if (!nodeChildExists(node, i))
    {
      if (delete_minimum && voxel_free_bit && !voxel_occupied_bit && (0.0 + prob_miss_log) <= clamping_thres_min)
      {
        // Delete minimum is set, the node doesn't exist in the tree, the
        // update voxel (and every sub-voxel) is a miss, and the miss
        // probability will put us at or below the minimum.
        // Nothing to do in this direction, as we would simply delete
        // everything we created.
        continue;
      }
      child_created = true;
      child_node = createNodeChild(node, i);
    }
    else
    {
      child_node = getNodeChild(node, i);
    }

    float child_log_odds = child_node->getLogOdds();
    bool leaf = (!child_created && !nodeHasChildren(child_node));
    bool child_at_min = (child_log_odds <= clamping_thres_min);
    bool child_at_max = (child_log_odds >= clamping_thres_max);
    bool stamps_match = (child_node->getTimestamp() == getLastUpdateTime());

    if (leaf && child_at_min && voxel_free_bit && !voxel_occupied_bit && stamps_match)
    {
      // This leaf is at min and the voxel (and any sub voxels) are free (the
      // child voxel can be inner or a leaf in this case).
      // The time stamps match.
      // Nothing more to do in this direction.
      continue;
    }

    if (leaf && child_at_max && voxel_is_leaf && voxel_occupied_bit && stamps_match)
    {
      // This leaf is at max, the voxel state is a leaf and occupied, and the
      // time stamps match.
      // Nothing more to do in this direction.
      continue;
    }

    if (delete_minimum && leaf && voxel_is_leaf && voxel_free_bit &&
        (child_log_odds + prob_miss_log <= clamping_thres_min))
    {
      // Deleting minimum, at a leaf and the update will put us below the min.
      // The child node should e deleted.
      // Lie and say the child was just created. This way the notification
      // function will always think there is a change (as we are about to
      // delete this child key).
      valueChangeCallbackWrapper(child_key,
                                 next_depth,
                                 true,
                                 child_log_odds,
                                 isNodeOccupied(child_node),
                                 clamping_thres_min,
                                 false);
      deleteNodeChild(node, i);
      continue;
    }

    if ((leaf || child_created) && voxel_is_leaf)
    {
      // Both the child node and child voxel are (or will be) leaves. Update the child node.
      float log_odds_update = voxel_free_bit ? prob_miss_log : prob_hit_log;

      // Update this leaf
      updateNodeLogOddsAndTrackChanges(child_node,
                                       log_odds_update,
                                       child_created,
                                       child_key,
                                       next_depth);
      // Continue on to next child
      continue;
    }

    // Need to go deeper in the tree. Recurs!
    if (applyUpdateRecurs(update, child_node, child_created, child_key, next_depth, next_center_offset_key))
    {
      deleteNodeChild(node, i);
    }
  }

  // Handle case that the node ended up with no children.
  // This can happen if the update had nothing in it, or if we are deleting
  // minimum and all the children were deleted.
  if (!nodeHasChildren(node))
  {
    return true;
  }

  if (!pruneNode(node))
  {
    // XXX check and make sure this will call the change call back when
    // appropriate
    node->updateOccupancyChildren();
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
  this->coordToKeyClamped(base_position.x() - xy_distance,
                          base_position.y() - xy_distance,
                          base_position.z() - z_depth,
                          *min_key);
  this->coordToKeyClamped(base_position.x() + xy_distance,
                          base_position.y() + xy_distance,
                          base_position.z() + z_height,
                          *max_key);
}

void OcTreeStampedWithExpiry::outOfBounds(double xy_distance,
                                          double z_height,
                                          double z_depth,
                                          const octomap::point3d& base_position,
                                          DeletionCallback change_notification /* = NodeChangeNotification() */)
{
  octomap::OcTreeKey minKey;
  octomap::OcTreeKey maxKey;
  calculateBounds(xy_distance, z_height, z_depth, base_position, &minKey, &maxKey);
  ROS_DEBUG_STREAM("Limiting to min key (" << minKey[0] << ", " << minKey[1] << ", " << minKey[2] <<
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
  // NOTE: if the tree ever has a default value, that should be considered
  // here. We assume that the default construction value of the node is 0.0
  // log odds. As long as the update would make the node the minimum, the
  // node is never stored. If this is the desired behavior, the user of the
  // tree should ensure it is setup that way for maximum performance gain.
  if (!leaf && delete_minimum && log_odds_update <= 0 && (0.0 + log_odds_update) <= this->clamping_thres_min)
  {
    return NULL;
  }
  if (leaf && delete_minimum && leaf->getLogOdds() + log_odds_update <= this->clamping_thres_min)
  {
    // Lie and say the node was just created. This way the notification
    // function will always think there is a change (as we are about to delete
    // this key).
    this->valueChangeCallbackWrapper(key, this->tree_depth, true,
                                     leaf->getLogOdds(), this->isNodeOccupied(leaf),
                                     this->clamping_thres_min, false);
    deleteNode(key);
    return NULL;
  }
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
    this->root = this->allocNode();
    this->tree_size++;
    createdRoot = true;
  }

  return super::updateNodeRecurs(this->root, createdRoot, key, 0, log_odds_update, lazy_eval);
}

void OcTreeStampedWithExpiry::updateNodeLogOdds(OcTreeNodeStampedWithExpiry* node, const float& update) const
{
  super::updateNodeLogOdds(node, update);
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
  super::expandNode(node);
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
  bool rv = super::pruneNode(node);
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
