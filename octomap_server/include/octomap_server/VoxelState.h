#ifndef OCTOMAP_VOXEL_STATE_H
#define OCTOMAP_VOXEL_STATE_H

namespace octomap_server {

using VoxelState = uint8_t;

namespace voxel_state {

// Make the voxel states unique bits for speed of down sampling in the array
// implementation.
static constexpr VoxelState UNKNOWN = 0;
static constexpr VoxelState FREE_SHIFT = 0;
static constexpr VoxelState FREE = (1<<FREE_SHIFT);
static constexpr VoxelState OCCUPIED_SHIFT = 1;
static constexpr VoxelState OCCUPIED = (1<<OCCUPIED_SHIFT);
static constexpr VoxelState INNER_SHIFT = 2;
static constexpr VoxelState INNER = (1<<INNER_SHIFT);
// MAX can be used to size a counter container indexed by VoxelState
// Since all the bits could be set, make max one more than all the bits on
static constexpr VoxelState MAX = (1<<(INNER_SHIFT+1));

}  // namespace VoxelState

}  // namespace octomap_server

#endif  // OCTOMAP_VOXEL_STATE_H
