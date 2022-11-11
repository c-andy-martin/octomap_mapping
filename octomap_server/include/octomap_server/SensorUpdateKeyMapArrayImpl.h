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
  virtual void setBounds(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key);
  virtual bool insertFree(const octomap::OcTreeKey& key);
  virtual bool insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count);
  virtual bool insertOccupied(const octomap::OcTreeKey& key);
  virtual void insertInner(const octomap::OcTreeKey& key);
  virtual void insert(const octomap::OcTreeKey& key, VoxelState state);
  virtual void downSample(const octomap::OcTreeSpace& tree, SensorUpdateKeyMapImpl* output_map) const;
  virtual VoxelState find(const octomap::OcTreeKey& key) const;
private:
  void downSample(const octomap::OcTreeSpace& tree, SensorUpdateKeyMapArrayImpl* output_array) const;
  inline unsigned int calculateIndex(const octomap::OcTreeKey& key) const
  {
    if (level_ < depth_)
    {
      // Bounds checking should be done by the interface class, not the
      // implementation. Assert that the key is in bounds.
      const unsigned int i = (key[0] - min_key_[0]) >> level_;
      const unsigned int j = (key[1] - min_key_[1]) >> level_;
      const unsigned int k = (key[2] - min_key_[2]) >> level_;
      assert(i < dims_[0]);
      assert(j < dims_[1]);
      assert(k < dims_[2]);
      unsigned int rv = i + j*dims_[0] + k*skip_;
      assert (rv < grid_.size());
      return rv;
    }
    return 0;
  }
  // Very fast implementation when it is known to be at level 0.
  // Static for speed, the caller should sample min_key, dims_[0] and skip_ to
  // locals (this method is mainly useful in tight loops for speed).
  static inline unsigned int calculateIndexLevel0(
      const octomap::OcTreeKey& key,
      const octomap::OcTreeKey& min_key,
      unsigned int width,
      unsigned int skip)
  {
    const unsigned int i = (key[0] - min_key[0]);
    const unsigned int j = (key[1] - min_key[1]);
    const unsigned int k = (key[2] - min_key[2]);
    unsigned int rv = i + j*width + k*skip;
    return rv;
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
  // Note that min/max are not center keys, but index keys!
  octomap::OcTreeKey min_key_;  // index key of minimum corner at level_
  octomap::OcTreeKey max_key_;  // index key of maximum corner at level_ (incl.)
  octomap::OcTreeKey dims_;
  unsigned int skip_;
  std::vector<VoxelState> grid_;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SENSOR_UPDATE_KEY_MAP_ARRAY_IMPL_H
