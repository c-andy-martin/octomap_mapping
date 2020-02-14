#ifndef OCTOMAP_SENSOR_UPDATE_KEY_MAP_ARRAY_IMPL_H
#define OCTOMAP_SENSOR_UPDATE_KEY_MAP_ARRAY_IMPL_H

#include <vector>
#include <octomap_server/SensorUpdateKeyMapImpl.h>

namespace octomap_server {

// Implementation of SensorUpdateKeyMap which uses a 3D array.
// This works well when the map is size-limited such that the 3D array isn't
// too large.
// This could be made more memory efficient by only using 2 bits per location
// instead of 8 at the cost of a slight performance hit.
class SensorUpdateKeyMapArrayImpl : public SensorUpdateKeyMapImpl
{
public:
  SensorUpdateKeyMapArrayImpl(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key);
  virtual ~SensorUpdateKeyMapArrayImpl() {}
  virtual void clear();
  virtual void apply(OcTreeT* tree) const;
  virtual void setBounds(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key);
  virtual bool insertFree(const octomap::OcTreeKey& key);
  virtual bool insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count);
  virtual bool insertOccupied(const octomap::OcTreeKey& key);
  virtual VoxelState find(const octomap::OcTreeKey& key) const;
private:
  inline unsigned int calculateIndex(const octomap::OcTreeKey& key) const
  {
    const unsigned int i = key[0] - min_key_[0];
    const unsigned int j = key[1] - min_key_[1];
    const unsigned int k = key[2] - min_key_[2];
    return i + j*dims_[0] + k*skip_;
  }
  // NOTE: does no bounds checking at all. It is assumed that
  // SensorUpdateKeyMap does the bounds checking.
  inline const VoxelState& gridRef(const octomap::OcTreeKey& key) const
  {
    return grid_[calculateIndex(key)];
  }
  inline VoxelState& gridRef(const octomap::OcTreeKey& key)
  {
    return grid_[calculateIndex(key)];
  }
  template <VoxelState mark_value>
  inline bool insertImpl(const octomap::OcTreeKey& key)
  {
    VoxelState& grid_ref = gridRef(key);
    if (grid_ref < mark_value)
    {
      grid_ref = mark_value;
      return true;
    }
    return false;
  }
  octomap::OcTreeKey min_key_;
  octomap::OcTreeKey dims_;
  unsigned int skip_;
  std::vector<VoxelState> grid_;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SENSOR_UPDATE_KEY_MAP_ARRAY_IMPL_H
