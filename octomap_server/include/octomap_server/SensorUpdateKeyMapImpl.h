#ifndef OCTOMAP_SENSOR_UPDATE_KEY_MAP_IMPL_H
#define OCTOMAP_SENSOR_UPDATE_KEY_MAP_IMPL_H

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_server/types.h>
#include <octomap_server/VoxelState.h>

namespace octomap_server {

class SensorUpdateKeyMapImpl
{
public:
  virtual ~SensorUpdateKeyMapImpl() {}
  /// Clear the update
  virtual void clear() = 0;
  /// Apply this update to the tree
  virtual void apply(OcTreeT* tree) const = 0;
  /// NOTE: setBounds will call clear, so no need to clear first
  virtual void setBounds(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key) = 0;
  // Returns true if the key was inserted, false if the key already existed
  // Unlike std::unordered_map, if the key exists, and value is true, the old
  // value is overwritten with true. This is because we are modeling a sensor
  // and want a voxel with both a ground and non-ground point to be always
  // counted as occupeid.
  virtual bool insertFree(const octomap::OcTreeKey& key) = 0;
  virtual bool insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count) = 0;
  virtual bool insertOccupied(const octomap::OcTreeKey& key) = 0;
  /// Return the state of the given voxel
  virtual VoxelState find(const octomap::OcTreeKey& key) const = 0;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SENSOR_UPDATE_KEY_MAP_IMPL_H
