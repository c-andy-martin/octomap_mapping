#include <octomap_server/SensorUpdateKeyMapArrayImpl.h>
#include <cstring>

namespace octomap_server {

SensorUpdateKeyMapArrayImpl::SensorUpdateKeyMapArrayImpl(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key)
{
  setBounds(min_key, max_key);
}

void SensorUpdateKeyMapArrayImpl::apply(OcTreeT* tree) const
{
  octomap::OcTreeKey index;
  // put min and dims on stack so compiler may optimize w/ register
  const octomap::OcTreeKey min(min_key_);
  const octomap::OcTreeKey dims(dims_);
  const VoxelState* const grid(grid_.data());
  for (index.k[2]=0; index.k[2]<dims.k[2]; ++index.k[2])
  {
    const unsigned int z_skip = index.k[2] * skip_;
    for (index.k[1]=0; index.k[1]<dims.k[1]; ++index.k[1])
    {
      const unsigned int y_skip = z_skip + index.k[1] * dims[0];
      for (index.k[0]=0; index.k[0]<dims.k[0]; ++index.k[0])
      {
        const VoxelState voxel_state = grid[y_skip + index.k[0]];
        if (voxel_state != UNKNOWN)
        {
          octomap::OcTreeKey key;
          key.k[0] = min.k[0] + index.k[0];
          key.k[1] = min.k[1] + index.k[1];
          key.k[2] = min.k[2] + index.k[2];
          tree->updateNode(key, voxel_state == OCCUPIED);
        }
      }
    }
  }
}

void SensorUpdateKeyMapArrayImpl::clear()
{
  std::memset(grid_.data(), UNKNOWN, grid_.size() * sizeof(VoxelState));
}

void SensorUpdateKeyMapArrayImpl::setBounds(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key)
{
  min_key_ = min_key;
  // max is inclusive
  dims_[0] = max_key[0] - min_key[0] + 1;
  dims_[1] = max_key[1] - min_key[1] + 1;
  dims_[2] = max_key[2] - min_key[2] + 1;
  skip_ = dims_[0] * dims_[1];
  grid_.resize(dims_[0] * dims_[1] * dims_[2]);
}

bool SensorUpdateKeyMapArrayImpl::insertFree(const octomap::OcTreeKey& key)
{
  return insertImpl<FREE>(key);
}

bool SensorUpdateKeyMapArrayImpl::insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count)
{
  if (free_cells_count == 0)
    return false;
  VoxelState* grid = grid_.data();
  while (free_cells_count > 0)
  {
    VoxelState* grid_loc = grid + calculateIndex(*free_cells);
    if (*grid_loc == UNKNOWN)
    {
      *grid_loc = FREE;
    }
    ++free_cells;
    --free_cells_count;
  }
  return true;
}

bool SensorUpdateKeyMapArrayImpl::insertOccupied(const octomap::OcTreeKey& key)
{
  return insertImpl<OCCUPIED>(key);
}

VoxelState SensorUpdateKeyMapArrayImpl::find(const octomap::OcTreeKey& key) const
{
  return gridRef(key);
}

} // end namespace octomap_server
