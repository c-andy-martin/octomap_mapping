#ifndef OCTOMAP_SENSOR_UPDATE_KEY_MAP_H
#define OCTOMAP_SENSOR_UPDATE_KEY_MAP_H

#include <memory>
#include <octomap/octomap.h>
#include <octomap_server/SensorUpdateKeyMapImpl.h>

namespace octomap_server {

class SensorUpdateKeyMap
{
public:
  SensorUpdateKeyMap();
  ~SensorUpdateKeyMap();

  //// NOTE: has no effect until setBounds is called
  void setVoxelVolumeArrayThreshold(size_t t) { voxel_volume_array_threshold_ = t; }
  void setFloorTruncation(octomap::key_type floor_z);
  /// NOTE: setBounds will call clear, so no need to clear first
  void setBounds(const octomap::OcTreeKey& min_key,
                 const octomap::OcTreeKey& max_key);
  /// setDepth should be called first before the first call to setBounds
  void setDepth(unsigned int depth)
  {
    depth_ = depth;
    if (depth == 0)
    {
      center_key_ = 0;
    }
    else
    {
      center_key_ = (1 << (depth - 1));
    }
  }

  inline bool isKeyOutOfBounds(const octomap::OcTreeKey& key) const
  {
    return !(min_key_[0] <= key[0] && key[0] <= max_key_[0] &&
        min_key_[1] <= key[1] && key[1] <= max_key_[1] &&
        min_key_[2] <= key[2] && key[2] <= max_key_[2]);
  }

  // Returns true if the key at the given level of the tree is entirely out
  // of bounds. (depth 0 is the root of the tree).
  // TODO: Move this into octomap itself. It has this same logic duplicated
  // inside both the bounding box iterator and deleteAABB.
  inline bool isKeyOutOfBounds(const octomap::OcTreeKey& key, unsigned int depth) const
  {
    if (depth == depth_)
    {
      return isKeyOutOfBounds(key);
    }
    assert(depth < depth_);
    const octomap::key_type offset_down = (center_key_ >> depth);
    const octomap::key_type offset_up = ((center_key_ - 1) >> depth);
    return !((min_key_[0] <= (key[0] + offset_up)) && (max_key_[0] >= (key[0] - offset_down)) &&
             (min_key_[1] <= (key[1] + offset_up)) && (max_key_[1] >= (key[1] - offset_down)) &&
             (min_key_[2] <= (key[2] + offset_up)) && (max_key_[2] >= (key[2] - offset_down)));
  }

  void clampRayToBounds(const octomap::OcTreeSpace& tree, const octomap::point3d& origin, octomap::point3d* end) const;
  bool insertFree(const octomap::OcTreeKey& key) {return insert(key, false);}
  bool insertFreeRay(const octomap::OcTreeSpace& tree, const octomap::point3d& origin, const octomap::point3d& end);
  bool insertOccupied(const octomap::OcTreeKey& key) {return insert(key, true);}
  /** Insert a ray, optionally marking or clearing the end.
   *
   * Does nothing if the origin is out of bounds.
   * Will clamp the end of the ray-tracing to the boundaries of this sensor
   * update. If a ray is clamped, it will not be marked at the end.
   * When in discrete mode, no work is done if the (clamped) end point of the
   * ray has already been traced, with the exception that if the end point is
   * to be marked, but was free, it will be marked.
   *
   * @param tree             octree to use for converting coordinates and keys
   * @param origin           world coordinate of ray origin
   * @param end              world coordinate of ray end
   * @param discrete         if true, do not re-trace a ray when a key already
   *                         exists at the ray-trace end
   * @param end_free         if true, set the end of the ray to free space
   * @param end_occupied     if true, set the end of the ray to occupied space
   * @param skip_tracing     if true, skip ray tracing completely
   * @param max_range        if positive, clamp rays to max_range, don't
   *                         mark/clear past this range
   * @param ray_shrink_cells if positive, shrink the ray to trace by this many
   *                         resolution units of space. this does not change
   *                         the end point for `end_free` or `end_clear`, but does
   *                         alter where the ray is checked for `discrete`.
   * @param post_mark_cells  if positive, mark this many resoultion units of
   *                         space on the ray after the end point.
   * @param furthest_touched_key key furthest from origin updated
   * @return true if any work is done
   */
  bool insertRay(const octomap::OcTreeSpace& tree,
                 const octomap::point3d& origin,
                 octomap::point3d end,
                 bool discrete=false,
                 bool end_free=false,
                 bool end_occupied=false,
                 bool skip_tracing=false,
                 double max_range=-1.0,
                 double ray_shrink_cells=0.0,
                 double post_mark_cells=0.0,
                 octomap::OcTreeKey* furthest_touched_key=NULL);

  // Returns true if the key was inserted, false if the key already existed
  // Unlike std::unordered_map, if the key exists, and value is true, the old
  // value is overwritten with true. This is because we are modeling a sensor
  // and want a voxel with both a ground and non-ground point to be always
  // counted as occupeid.
  bool insert(const octomap::OcTreeKey& key, bool value = false);

  // Insert from any container of OcTreeKey's
  // This will only insert clear space (value is always false)
  template <class InputIt>
  void insert(InputIt first, InputIt last) {
    while (first != last) {
      insert(*first);
      ++first;
    }
  }

  /// Empty the sensor update key map
  void clear() {impl_->clear();}
  /// Return the state of the given voxel
  VoxelState find(const octomap::OcTreeKey& key) const;

  /// Return the state of the given voxel at the given depth in the tree
  /// (depth 0 is the root, opposite of level, coresponding to octomap APIs)
  /// The layers above 0 are only updated when updateLayers is called
  VoxelState find(const octomap::OcTreeKey& key, unsigned int depth) const;

  /// Update the down-sampled layers using the given OcTreeSpace.
  /// Call this after the update is complete prior to using the version of
  /// find() that takes depth.
  void updateLayers(const octomap::OcTreeSpace& tree);

private:
  // If the voxel volume represented by the bounds is less than this
  // threshold, the voxel array implementation will be used. Otherwise, the
  // hash table implementation will be used (which will consume much less
  // memory, but be more computationally expensive).
  size_t voxel_volume_array_threshold_;
  std::vector<std::unique_ptr<SensorUpdateKeyMapImpl>> impls_;
  // convenience pointer
  SensorUpdateKeyMapImpl* impl_;
  std::unique_ptr<octomap::OcTreeKey> free_cells_;
  size_t free_cells_capacity_;

  bool truncate_floor_;
  octomap::key_type truncate_floor_z_;

  octomap::OcTreeKey min_key_;
  octomap::OcTreeKey max_key_;
  unsigned int depth_;
  octomap::key_type center_key_;  // Cache the center key for the given depth
};

}  // namespace octomap_server

#endif  // OCTOMAP_SENSOR_UPDATE_KEY_MAP_H
