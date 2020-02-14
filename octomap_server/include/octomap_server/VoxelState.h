#ifndef OCTOMAP_VOXEL_STATE_H
#define OCTOMAP_VOXEL_STATE_H

namespace octomap_server {

using VoxelState = uint8_t;
static constexpr VoxelState UNKNOWN = 0;
static constexpr VoxelState FREE = 1;
static constexpr VoxelState OCCUPIED = 2;
static constexpr VoxelState INNER = 3;

}  // namespace octomap_server

#endif  // OCTOMAP_VOXEL_STATE_H
