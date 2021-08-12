#ifndef OCTOMAP_OCTREE_STAMPED_NONLINEAR_DECAY_H
#define OCTOMAP_OCTREE_STAMPED_NONLINEAR_DECAY_H

#include <ctime>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <stdlib.h>
#include <ros/ros.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap_server/SensorUpdateKeyMap.h>

namespace octomap_server {
using NodeChangeNotification = std::function<void(const octomap::OcTreeKey&, unsigned int)>;

// Node declaration and definition
template <typename OcTreeNodeBase>
class OcTreeNodeStampedWithExpiry : public OcTreeNodeBase
{
    using super = OcTreeNodeBase;
  public:
    // Class-wide Parameters.

    OcTreeNodeStampedWithExpiry() : super(), stamp(0), expiry(0) {}

    bool operator==(const OcTreeNodeStampedWithExpiry<OcTreeNodeBase>& rhs) const
    {
      // No need to compare expiry, as it is a function of stamp and value
      return rhs.stamp == stamp && static_cast<const super&>(*this) == static_cast<const super&>(rhs);
    }

    void copyData(const OcTreeNodeStampedWithExpiry<OcTreeNodeBase>& from)
    {
      super::copyData(from);
      stamp = from.stamp;
      expiry = from.expiry;
    }

    // timestamp
    inline time_t getTimestamp() const { return stamp; }
    inline void setTimestamp(time_t new_stamp) { stamp = new_stamp; }

    // expiry
    inline time_t getExpiry() const { return expiry; }
    inline void setExpiry(time_t new_expiry) { expiry = new_expiry; }

    // update occupancy and timesteps of inner nodes
    inline void updateOccupancyChildren()
    {
      super::updateOccupancyChildren();

      time_t min_stamp = std::numeric_limits<time_t>::max();
      time_t min_expiry = std::numeric_limits<time_t>::max();
      bool have_children = false;

      if (this->children != NULL) {
        for (unsigned int i=0; i<8; i++) {
          if (this->children[i] != NULL) {
            OcTreeNodeStampedWithExpiry<OcTreeNodeBase> *node = static_cast<OcTreeNodeStampedWithExpiry<OcTreeNodeBase>*>(this->children[i]);
            min_stamp = std::min(min_stamp, node->stamp);
            min_expiry = std::min(min_expiry, node->expiry);
            have_children = true;
          }
        }
      }

      // only update if we have children
      if (have_children) {
        stamp = min_stamp;
        expiry = min_expiry;
      }
    }

  protected:
    // There is no need for more than second accuracy, so only store the
    // seconds component of the timestamp.
    time_t stamp, expiry;
};


// Tree declaration
template <typename OcTreeNodeBase>
class OcTreeStampedWithExpiry : public octomap::OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry<OcTreeNodeBase>>
{
    using super = octomap::OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry<OcTreeNodeBase>>;
  public:
    using typename super::NodeType;
    // Default constructor, sets resolution.
    // Be sure to call expireNodes() after construction to initialize the
    // expiration time. This can not be done in the default constructor
    // because it is called before ros::Time::now() can be accessed.
    OcTreeStampedWithExpiry(double resolution);

    // virtual constructor: creates a new object of same type
    // (Covariant return type requires an up-to-date compiler)
    OcTreeStampedWithExpiry<OcTreeNodeBase>* create() const {return new OcTreeStampedWithExpiry<OcTreeNodeBase>(this->resolution); }

    void setQuadraticParameters(double a_coeff, double c_coeff, double quadratic_start, double c_coeff_free, bool log=true)
    {
      a_coeff_ = a_coeff;
      c_coeff_ = c_coeff;
      quadratic_start_ = quadratic_start;
      c_coeff_free_ = c_coeff_free;
      // Set the free space mask to round to the nearest power of 2 of c_coeff_free_/10.0
      uint32_t c_power_2 = (1 << (32 - __builtin_clz(static_cast<uint32_t>(std::floor(c_coeff_free_/10.0)))));
      free_space_stamp_mask_ = ~(c_power_2 - 1);
      if (log)
      {
        ROS_INFO_STREAM("Set quadratic parameters a_coeff: " << a_coeff_ <<
                        " c_coeff: " << c_coeff_ <<
                        " c_coeff_free: " << c_coeff_free_ <<
                        " quadratic_start: " << quadratic_start_ <<
                        " free_space_stamp_mask: " << std::hex << free_space_stamp_mask_);
      }
    }

    // FIXME: until this class is in a library that the octomap_rviz_plugin is
    // using, pretend we are the base tree for serializing so that the
    // octomap_rviz_plugin can visualize this octree. Once the
    // octomap_rviz_plugin can parse and understand this type of tree, change
    // to the correct value and also remove the commented line below commenting
    // out registerTreeType inside of the StaticMemberInitializer.
    std::string getTreeType() const {
      if (std::is_same<octomap::ColorOcTreeNode, OcTreeNodeBase>()) {
        return "ColorOcTree";
      }
      return "OcTree";
    }

    // Time of last update
    time_t getLastUpdateTime() const {return last_expire_time;}

    // Time of last update, masked by free space mask
    time_t getLastUpdateTimeFreeSpace() const {return (last_expire_time & free_space_stamp_mask_);}

    // Remove all expired nodes.
    // This also calculates and stores any missing expiration times in the tree.
    // This function should be called periodically.
    void expireNodes(NodeChangeNotification change_notification = NodeChangeNotification(),
                     bool delete_expired_nodes = true);

    // Calculate min/max octree keys based on given parameters
    void calculateBounds(double xy_distance,
                         double z_height,
                         double z_depth,
                         const octomap::point3d& base_position,
                         octomap::OcTreeKey* min_key,
                         octomap::OcTreeKey* max_key);

    // Delete nodes that are out of bounds
    void outOfBounds(double xy_distance, double z_height, double z_depth, const octomap::point3d& base_position, typename super::DeletionCallback change_notification = typename super::DeletionCallback());

    virtual OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* updateNode(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval = false);
    virtual OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* updateNode(const octomap::OcTreeKey& key, bool occupied, bool lazy_eval = false) {
      return updateNode(key, occupied ? this->prob_hit_log : this->prob_miss_log, lazy_eval);
    }

    virtual void updateNodeLogOdds(OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* node, const float& update) const;

    virtual void expandNode(NodeType* node);
    virtual bool pruneNode(NodeType* node);

    bool getSizeChanged() {return this->size_changed;}
    void setSizeChanged(bool new_value) {this->size_changed = new_value;}

    time_t getMaxExpiryDelta() const {
      return a_coeff_log_odds_* this->clamping_thres_max * this->clamping_thres_max + c_coeff_;
    }

    /// When a node is updated to the minimum, delete it if present and do not store
    void setDeleteMinimum(bool enable) { delete_minimum = enable; }

    /// Get mode of deleting free space
    bool getDeleteMinimum() { return delete_minimum; }

    // Apply a sensor update to our tree efficiently.
    // This method is O(n*log(depth)), where looping over the
    // update and calling updateNode would be O(n*depth) where n is the number
    // of new nodes in the update and depth is the tree_depth.
    void applyUpdate(const SensorUpdateKeyMap& update);

  protected:
    // Returns true if the node should be removed from the tree
    // This might happen if delete_minimum is set.
    bool applyUpdateRecurs(const SensorUpdateKeyMap& update,
                           NodeType* node,
                           bool node_just_created,
                           const octomap::OcTreeKey& key,
                           unsigned int depth,
                           octomap::key_type center_offset_key);
    // Quadratic delta-t expiration coefficients. The input is the number of
    // times a particular mode was marked from the default value (which would
    // be the current logodds divided prob_hit_log).
    double a_coeff_, a_coeff_log_odds_;
    // Assume b_coeff is always zero
    double c_coeff_;
    double quadratic_start_, quadratic_start_log_odds_;
    // Assume free space we just use a flat timeout for
    double c_coeff_free_;
    // Relax time-stamp matching requirements for free-space
    // This allows optimal pruning for free-space as we sense free-space at
    // different time intervals.
    // Free-space time is relaxed according to c_coeff_free. A power of 2 is
    // chosen near 1/10th of c_coeff_free.
    time_t free_space_stamp_mask_;
    // Used as the new value for updated nodes.
    // Only updated when calling expireNodes. This keeps our idea of time at
    // the resolution of our expiration rate check, which allows us to easily
    // prune nodes as their timestamps will only change when this time is
    // updated.
    time_t last_expire_time;
    int expire_count;

    bool delete_minimum;

    // Return true if this node has expired.
    // Assume node is valid.
    bool expireNodeRecurs(OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* node,
                          const octomap::OcTreeKey& key,
                          int depth,
                          const NodeChangeNotification& change_notification,
                          bool delete_expired_nodes = true);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTreeStampedWithExpiry<OcTreeNodeBase>* tree = new OcTreeStampedWithExpiry<OcTreeNodeBase>(0.1);
        tree->clearKeyRays();
        //AbstractOcTree::registerTreeType(tree);
      }

      /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
      void ensureLinking() {};
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeStampedWithExpiryMemberInit;
};


// Tree Definitions
template <typename OcTreeNodeBase>
OcTreeStampedWithExpiry<OcTreeNodeBase>::OcTreeStampedWithExpiry(double resolution)
  : super(resolution)
  , last_expire_time(0)
  , delete_minimum(false)
{
  // Set the occupancy threshold to log odds zero.
  this->occ_prob_thres_log = 0.0;
  ocTreeStampedWithExpiryMemberInit.ensureLinking();
  // Set the quadratic parameters to some defaults
  setQuadraticParameters(1.0, 3.0, 0.0, 60.0, false);
}

template <typename OcTreeNodeBase>
void OcTreeStampedWithExpiry<OcTreeNodeBase>::expireNodes(
    NodeChangeNotification change_notification /* = NodeChangeNotification() */,
    bool delete_expired_nodes /* = true */)
{
  octomap::OcTreeKey rootKey(this->tree_max_val, this->tree_max_val, this->tree_max_val);
  last_expire_time = ros::Time::now().sec;

  // pre-compute a_coeff in terms of log-odds instead of number of observations
  a_coeff_log_odds_ = a_coeff_ * (1.0 / this->prob_hit_log) * (1.0 / this->prob_hit_log);
  quadratic_start_log_odds_ = quadratic_start_ * this->prob_hit_log;

  if (this->root != NULL)
  {
    ROS_DEBUG("prior to expiry, root expiry was: %ld.", this->root->getExpiry());
    expire_count = 0;
    if (expireNodeRecurs(this->root, rootKey, 0, change_notification, delete_expired_nodes))
    {
      // The whole tree expired. This is odd but possible if no sensor data
      // has been received. It is odd enough to log this.
      ROS_WARN("Entire octree expired!");
      if(change_notification)
      {
        change_notification(rootKey, 0);
      }
      this->deleteNodeRecurs(this->root);
      this->root = NULL;
      expire_count++;
    }
    ROS_DEBUG("Expired %d nodes. Last expiry time is %ld.", expire_count, last_expire_time);
    if (this->root)
    {
      ROS_DEBUG("Next node expires at %ld.", this->root->getExpiry());
    }
  }
}

template <typename OcTreeNodeBase>
bool OcTreeStampedWithExpiry<OcTreeNodeBase>::expireNodeRecurs(
    OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* node,
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
    if (this->nodeHasChildren(node))
    {
      // Update all children first
      for (unsigned int i=0; i<8; i++)
      {
        if (this->nodeChildExists(node, i))
        {
          octomap::OcTreeKey child_key;
          computeChildKey(i, center_offset_key, key, child_key);
          if (expireNodeRecurs(this->getNodeChild(node, i), child_key, depth+1, change_notification, delete_expired_nodes))
          {
            // Delete the child node
            this->deleteNodeChild(node, i);
            expire_count++;
          }
        }
      }
      // If we have no more children left, this inner node has expired too
      if (!this->nodeHasChildren(node))
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
        if (value < this->occ_prob_thres_log)
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
      }
      if (delete_expired_nodes && expiry <= last_expire_time)
      {
        // We have expired!
        change_notification(key, depth);
        return true;
      }
    }
  }
  return false;
}

template <typename OcTreeNodeBase>
void OcTreeStampedWithExpiry<OcTreeNodeBase>::applyUpdate(const SensorUpdateKeyMap& update)
{
  octomap::OcTreeKey root_key(this->tree_max_val, this->tree_max_val, this->tree_max_val);

  bool created_root = false;
  if (this->root == nullptr)
  {
    this->root = this->allocNode();
    this->tree_size++;
    created_root = true;
  }
  if (applyUpdateRecurs(update, this->root, created_root, root_key, 0, this->tree_max_val >> 1))
  {
    this->deleteNodeRecurs(this->root);
    this->root = nullptr;
  }
}

template <typename OcTreeNodeBase>
bool OcTreeStampedWithExpiry<OcTreeNodeBase>::applyUpdateRecurs(
    const SensorUpdateKeyMap& update,
    OcTreeStampedWithExpiry<OcTreeNodeBase>::NodeType* node,
    bool node_just_created,
    const octomap::OcTreeKey& key,
    unsigned int depth,
    octomap::key_type center_offset_key)
{
  assert(node != nullptr);
  assert(depth < this->tree_depth);
  unsigned int next_depth = depth + 1;

  if (!node_just_created && !this->nodeHasChildren(node))
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
    if (!this->nodeChildExists(node, i))
    {
      if (delete_minimum && voxel_free_bit && !voxel_occupied_bit && (0.0 + this->prob_miss_log) <= this->clamping_thres_min)
      {
        // Delete minimum is set, the node doesn't exist in the tree, the
        // update voxel (and every sub-voxel) is a miss, and the miss
        // probability will put us at or below the minimum.
        // Nothing to do in this direction, as we would simply delete
        // everything we created.
        continue;
      }
      child_created = true;
      child_node = this->createNodeChild(node, i);
    }
    else
    {
      child_node = this->getNodeChild(node, i);
    }

    float child_log_odds = child_node->getLogOdds();
    bool leaf = (!child_created && !this->nodeHasChildren(child_node));
    bool child_at_min = (child_log_odds <= this->clamping_thres_min);
    bool child_at_max = (child_log_odds >= this->clamping_thres_max);

    if (leaf && child_at_min && voxel_free_bit && !voxel_occupied_bit &&
        child_node->getTimestamp() == getLastUpdateTimeFreeSpace())
    {
      // This leaf is at min and the voxel (and any sub voxels) are free (the
      // child voxel can be inner or a leaf in this case).
      // The time stamps match.
      // Nothing more to do in this direction.
      continue;
    }

    if (leaf && child_at_max && voxel_is_leaf && voxel_occupied_bit &&
        child_node->getTimestamp() == getLastUpdateTime())
    {
      // This leaf is at max, the voxel state is a leaf and occupied, and the
      // time stamps match.
      // Nothing more to do in this direction.
      continue;
    }

    if (delete_minimum && leaf && voxel_is_leaf && voxel_free_bit &&
        (child_log_odds + this->prob_miss_log <= this->clamping_thres_min))
    {
      // Deleting minimum, at a leaf and the update will put us below the min.
      // The child node should e deleted.
      // Lie and say the child was just created. This way the notification
      // function will always think there is a change (as we are about to
      // delete this child key).
      this->valueChangeCallbackWrapper(
          child_key,
          next_depth,
          true,
          child_log_odds,
          this->isNodeOccupied(child_node),
          this->clamping_thres_min,
          false);
      this->deleteNodeChild(node, i);
      continue;
    }

    if ((leaf || child_created) && voxel_is_leaf)
    {
      // Both the child node and child voxel are (or will be) leaves. Update the child node.
      float log_odds_update = voxel_free_bit ? this->prob_miss_log : this->prob_hit_log;

      // Update this leaf
      this->updateNodeLogOddsAndTrackChanges(
          child_node,
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
      this->deleteNodeChild(node, i);
    }
  }

  // Handle case that the node ended up with no children.
  // This can happen if the update had nothing in it, or if we are deleting
  // minimum and all the children were deleted.
  if (!this->nodeHasChildren(node))
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

template <typename OcTreeNodeBase>
void OcTreeStampedWithExpiry<OcTreeNodeBase>::calculateBounds(
    double xy_distance,
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

template <typename OcTreeNodeBase>
void OcTreeStampedWithExpiry<OcTreeNodeBase>::outOfBounds(
    double xy_distance,
    double z_height,
    double z_depth,
    const octomap::point3d& base_position,
    typename super::DeletionCallback change_notification /* = NodeChangeNotification() */)
{
  octomap::OcTreeKey minKey;
  octomap::OcTreeKey maxKey;
  calculateBounds(xy_distance, z_height, z_depth, base_position, &minKey, &maxKey);
  ROS_DEBUG_STREAM("Limiting to min key (" << minKey[0] << ", " << minKey[1] << ", " << minKey[2] <<
                   "), max key (" << maxKey[0] << ", " << maxKey[1] << ", " << maxKey[2] << ")");
  if (this->root != NULL)
  {
    this->deleteAABB(minKey, maxKey, true, change_notification);
  }
}

template <typename OcTreeNodeBase>
OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* OcTreeStampedWithExpiry<OcTreeNodeBase>::updateNode(
    const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval)
{
  // early abort (no change will happen).
  // may cause an overhead in some configuration, but more often helps
  OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* leaf = this->search(key);
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
    this->deleteNode(key);
    return NULL;
  }
  // no change: node already at threshold
  if (leaf
      && (log_odds_update >= 0 && leaf->getLogOdds() >= this->clamping_thres_max)
      && (leaf->getTimestamp() == getLastUpdateTime()))
  {
    return leaf;
  }
  if (leaf
      && (log_odds_update <= 0 && leaf->getLogOdds() <= this->clamping_thres_min)
      && (leaf->getTimestamp() == getLastUpdateTimeFreeSpace()))
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

template <typename OcTreeNodeBase>
void OcTreeStampedWithExpiry<OcTreeNodeBase>::updateNodeLogOdds(
    OcTreeNodeStampedWithExpiry<OcTreeNodeBase>* node, const float& update) const
{
  super::updateNodeLogOdds(node, update);
  time_t new_stamp = last_expire_time;
  if (!this->isNodeOccupied(node))
  {
    new_stamp &= free_space_stamp_mask_;
  }
  node->setTimestamp(new_stamp);
  // Because we will very likely observe the same space multiple times, we do
  // not want to update expiry just to have to update it again on the next
  // sensor cycle. Instead, set it to zero and re-calculate it when it is
  // needed.
  node->setExpiry(0);
}

template <typename OcTreeNodeBase>
void OcTreeStampedWithExpiry<OcTreeNodeBase>::expandNode(OcTreeStampedWithExpiry<OcTreeNodeBase>::NodeType* node)
{
  bool old_size_changed = this->size_changed;
  super::expandNode(node);
  // This should really be fixed upstream, if it ever is, we can remove
  if (!old_size_changed && this->size_changed)
  {
    // override size_changed back to false, expanding can't alter extents
    this->size_changed = false;
  }
}

template <typename OcTreeNodeBase>
bool OcTreeStampedWithExpiry<OcTreeNodeBase>::pruneNode(OcTreeStampedWithExpiry<OcTreeNodeBase>::NodeType* node)
{
  bool old_size_changed = this->size_changed;
  bool rv = super::pruneNode(node);
  // This should really be fixed upstream, if it ever is, we can remove
  if (!old_size_changed && this->size_changed)
  {
    // override size_changed back to false, pruning can't alter extents
    this->size_changed = false;
  }
  return rv;
}

} // end namespace

#endif
