#ifndef OCTOMAP_SENSOR_UPDATE_KEY_MAP_IMPL_H
#define OCTOMAP_SENSOR_UPDATE_KEY_MAP_IMPL_H

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_server/VoxelState.h>

namespace octomap_server {

class SensorUpdateKeyMapImpl
{
public:
  virtual ~SensorUpdateKeyMapImpl() {}
  /// Clear the update
  virtual void clear() = 0;
  virtual unsigned int getDepth() const {return depth_;}
  /// NOTE: setDepth will change the interpretation of any data present
  virtual void setDepth(unsigned int depth) {depth_ = depth;}
  virtual unsigned int getLevel() const {return level_;}
  virtual void setLevel(unsigned int level) {level_ = level;}
  /// NOTE: setBounds will call clear, so no need to clear first
  virtual void setBounds(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key) = 0;
  /// Make the output map be a down-sampled depth-1 view of this map.
  /// Calls setBounds from our bounds, thus clearing any data present in the
  /// output map.
  virtual void downSample(const octomap::OcTreeSpace& tree, SensorUpdateKeyMapImpl* output_map) const
  {
    assert(level_ < depth_);
    output_map->setLevel(level_ + 1);
  }
  // Returns true if the key was inserted, false if the key already existed
  // Unlike std::unordered_map, if the key exists, and value is true, the old
  // value is overwritten with true. This is because we are modeling a sensor
  // and want a voxel with both a ground and non-ground point to be always
  // counted as occupeid.
  virtual bool insertFree(const octomap::OcTreeKey& key) = 0;
  // This call only works on level 0 maps, as it is an optimization.
  virtual bool insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count) = 0;
  virtual bool insertOccupied(const octomap::OcTreeKey& key) = 0;
  // Insert an inner voxel at the given key.
  // Does nothing if the voxel already existed.
  virtual void insertInner(const octomap::OcTreeKey& key) = 0;
  // Insert a voxel with the given state at the given key.
  // Does nothing if the voxel already existed.
  virtual void insert(const octomap::OcTreeKey& key, VoxelState state) = 0;
  /// Return the state of the given voxel
  virtual VoxelState find(const octomap::OcTreeKey& key) const = 0;
protected:
  unsigned int depth_;  // Tree depth
  unsigned int level_;  // Level of the tree we are on (level 0 is the bottom)
};

}  // namespace octomap_server

#endif  // OCTOMAP_SENSOR_UPDATE_KEY_MAP_IMPL_H
