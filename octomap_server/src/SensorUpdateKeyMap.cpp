#include <octomap_server/SensorUpdateKeyMap.h>

#include <limits>

#include <octomap_server/SensorUpdateKeyMapHashImpl.h>
#include <octomap_server/SensorUpdateKeyMapArrayImpl.h>

namespace octomap_server {

SensorUpdateKeyMap::SensorUpdateKeyMap()
  : free_cells_capacity_(0)
  , truncate_floor_(false)
{
  octomap::OcTreeKey min_key(std::numeric_limits<octomap::key_type>::min(),
                             std::numeric_limits<octomap::key_type>::min(),
                             std::numeric_limits<octomap::key_type>::min());
  octomap::OcTreeKey max_key(std::numeric_limits<octomap::key_type>::max(),
                             std::numeric_limits<octomap::key_type>::max(),
                             std::numeric_limits<octomap::key_type>::max());
  setBounds(min_key, max_key);
}

SensorUpdateKeyMap::~SensorUpdateKeyMap()
{
}

void SensorUpdateKeyMap::clampRayToBounds(const OcTreeT& tree, const octomap::point3d& origin, octomap::point3d* end) const
{
  octomap::OcTreeKey origin_key;
  octomap::OcTreeKey end_key;
  if (!tree.coordToKeyChecked(origin, origin_key))
  {
    // Don't adjust the end if the origin is not in the tree.
    return;
  }

  tree.coordToKeyClamped(*end, end_key);
  if (isKeyOutOfBounds(origin_key) || !isKeyOutOfBounds(end_key))
  {
    // either origin is out of bounds, or end is in bounds, nothing to do.
    return;
  }

  // Find the dimension with the lowest ratio of projected length vs. actual.
  octomap::point3d direction = *end - origin;

  double smallest_ratio = std::numeric_limits<typeof(direction(0))>::max();
  for (int i=0; i<3; i++)
  {
    if (direction(i) != 0.0)
    {
      auto min_pt_i = tree.keyToCoord(min_key_[i]);
      auto max_pt_i = tree.keyToCoord(max_key_[i]);
      auto numer = (direction(i) < 0.0 ? min_pt_i : max_pt_i) - origin(i);
      auto ratio = numer / direction(i);
      if (ratio > 0.0 && ratio < smallest_ratio )
      {
        smallest_ratio = ratio;
      }
    }
  }
  // project ray using smallest ratio
  octomap::point3d new_end = origin + direction * smallest_ratio;
  // round to cell center using tree coord to key operations
  tree.coordToKeyClamped(new_end, end_key);
  assert (!isKeyOutOfBounds(end_key));
  *end = tree.keyToCoord(end_key);
}

void SensorUpdateKeyMap::setFloorTruncation(octomap::key_type floor_z)
{
  truncate_floor_ = true;
  truncate_floor_z_ = floor_z;
}

bool SensorUpdateKeyMap::insertFreeRay(const OcTreeT& tree,
                                        const octomap::point3d& origin,
                                        const octomap::point3d& end)
{
  // a version of computeRayKeys from OcTreeBaseImpl.hxx which adds the ray
  // keys directly to the our SensorUpdateKeyMap

  octomap::OcTreeKey origin_key, end_key;
  if (!tree.coordToKeyChecked(origin, origin_key))
  {
    return false;
  }
  if (!tree.coordToKeyChecked(end, end_key))
  {
    return false;
  }
  if (isKeyOutOfBounds(origin_key))
  {
    return false;
  }
  // Nothing to do if origin and end are in same cell
  if (origin_key == end_key)
  {
    return false;
  }

  // Initialization
  octomap::point3d direction = (end - origin);
  octomap::point3d origin_boundary = tree.keyToCoord(origin_key);
  double resolution = tree.getResolution();
  double length = direction.norm();
  direction /= length; // normalize vector

  int    step[3];
  double tMax[3];
  double tDelta[3];

  octomap::OcTreeKey current_key = origin_key;
  octomap::OcTreeKey justOut;

  size_t max_cells = 0;

  for(unsigned int i=0; i < 3; ++i) {
    // tally up max_cells (maximum ray trace is one cell per delta dimension)
    if (origin_key[i] > end_key[i])
    {
      max_cells += origin_key[i] - end_key[i] + 1;
    }
    else
    {
      max_cells += end_key[i] - origin_key[i] + 1;
    }

    // compute step direction
    if (direction(i) > 0.0) step[i] = 1;
    else if (direction(i) < 0.0) step[i] = -1;
    else step[i] = 0;

    // compute tMax, tDelta, justOut
    if (step[i] != 0) {
      // corner point of voxel (in direction of ray)
      double voxelBorder = origin_boundary(i);
      voxelBorder += (step[i] * resolution * 0.5);

      tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
      tDelta[i] = resolution / fabs( direction(i) );
      justOut[i] = end_key[i] + step[i];
    }
    else {
      tMax[i] =  std::numeric_limits<double>::max( );
      tDelta[i] = std::numeric_limits<double>::max( );
      justOut[i] = std::numeric_limits<octomap::key_type>::max( );
    }
  }

  if (truncate_floor_ && justOut[2] < truncate_floor_z_) {
    // truncation only makes sense if we are going from above to below
    if (direction(2) < 0.0) {
      // set just out just below the floor z value
      justOut[2] = truncate_floor_z_ - 1;
    }
  }

  if (free_cells_capacity_ < max_cells)
  {
    // need more space
    // always over-allocate to leave some room to grow
    free_cells_capacity_ = max_cells * 2;
    free_cells_.reset(new octomap::OcTreeKey[free_cells_capacity_]);
  }

  size_t free_cells_count = 0;
  octomap::OcTreeKey* free_cells = free_cells_.get();

  // Incremental phase
  for (;;)
  {
    // We have moved out-of-bounds, stop tracing
    if (isKeyOutOfBounds(current_key))
      break;

    // add the cell
    free_cells[free_cells_count++] = current_key;

    if (tMax[0] < tMax[1])
    {
      if (tMax[0] < tMax[2])
      {
        current_key[0] += step[0];
        tMax[0] += tDelta[0];
        if (current_key[0] == justOut[0])
        {
          break;
        }
      }
      else
      {
        current_key[2] += step[2];
        tMax[2] += tDelta[2];
        if (current_key[2] == justOut[2])
        {
          break;
        }
      }
    }
    else
    {
      if (tMax[1] < tMax[2])
      {
        current_key[1] += step[1];
        tMax[1] += tDelta[1];
        if (current_key[1] == justOut[1])
        {
          break;
        }
      }
      else
      {
        current_key[2] += step[2];
        tMax[2] += tDelta[2];
        if (current_key[2] == justOut[2])
        {
          break;
        }
      }
    }
  }
  assert(free_cells_count <= free_cells_capacity_);
  return impl_->insertFreeCells(free_cells, free_cells_count);
}

bool SensorUpdateKeyMap::insertRay(const OcTreeT& tree,
                                   const octomap::point3d& origin,
                                   octomap::point3d end,
                                   bool discrete,
                                   bool end_free,
                                   bool end_occupied,
                                   bool skip_tracing,
                                   double max_range,
                                   double ray_shrink_cells,
                                   double post_mark_cells,
                                   octomap::OcTreeKey* furthest_touched_key)
{
  bool cells_added = false;
  octomap::OcTreeKey origin_key, end_key;
  if (!tree.coordToKeyChecked(origin, origin_key))
  {
    // origin key is not in the tree
    return false;
  }
  if (isKeyOutOfBounds(origin_key))
  {
    // origin key is outside of bounds, we do not support starting tracing
    // out-of-bounds.
    return false;
  }

  const octomap::point3d ray = end - origin;
  const octomap::point3d direction = ray.normalized();

  // Find the adjusted ray trace end point.
  {
    octomap::point3d adjusted_end = end;
    octomap::OcTreeKey adjusted_end_key;

    // Adjust for max range
    if (max_range > 0.0 && (adjusted_end - origin).norm() > max_range)
    {
      adjusted_end = origin + direction * max_range;
      if (end_occupied)
      {
        // The original obstacle is out-of-bounds, so pretend this is a
        // clear-ray instead of an occupied one.
        end_free = true;
        end_occupied = false;
      }
    }

    // Adjust the end to the bounds.
    clampRayToBounds(tree, origin, &adjusted_end);

    end_key = tree.coordToKey(end);
    adjusted_end_key = tree.coordToKey(adjusted_end);
    assert(!isKeyOutOfBounds(adjusted_end_key));

    if (end_occupied && isKeyOutOfBounds(end_key))
    {
      // The original end is beyond the bounds.
      if (truncate_floor_ && end_key[2] < truncate_floor_z_ && adjusted_end_key[2] <= truncate_floor_z_)
      {
        // We are truncating the floor, the original end is below the floor and
        // the adjusted end is within one resolution unit of the floor.
        // In such a case, we want to mark the end as a cliff-like obstacle
        // because floor truncation is on, so do not clear end_occupied.
      }
      else
      {
        // The original obstacle is out-of-bounds, so pretend this is a
        // clear-ray instead of an occupied one.
        end_free = true;
        end_occupied = false;
      }
    }

    // If the end voxel key has changed, there is no need to shrink the ray by
    // ray shrink cells. While this may mean we clear a bit extra if we were
    // within one voxel of the boundary, it is not worth the extra calculation
    // to adjust ray shrink cells in the small chance it overlapped. Simply
    // turn it off if the end was moved.
    if (end_key != adjusted_end_key)
    {
      ray_shrink_cells = 0.0;
    }

    // Update the end
    end = adjusted_end;
    end_key = adjusted_end_key;
  }
  assert(!isKeyOutOfBounds(end_key));

  // Check the adjusted end before marking or clearing the end point to correctly apply discrete.
  if (discrete)
  {
    if (impl_->find(end_key) != UNKNOWN)
    {
      // ray tracing endpoint already in update, skip ray-tracing.
      skip_tracing = true;
    }
  }

  if (end_free || end_occupied)
  {
    if (max_range <= 0.0 || ray.norm() <= max_range)
    {
      if (tree.coordToKeyChecked(end, end_key))
      {
        if (!isKeyOutOfBounds(end_key))
        {
          bool inserted = false;
          // call insertOccupied/insertFree so floor truncation works if enabled
          if ((end_occupied && insertOccupied(end_key)) || (!end_occupied && insertFree(end_key)))
          {
            inserted = true;
            cells_added = true;
            if (furthest_touched_key)
            {
              *furthest_touched_key = end_key;
            }
          }
          else
          {
            if (discrete)
            {
              // we have information already on this cell, do not raytrace
              skip_tracing = true;
            }
          }
          if (!discrete || inserted)
          {
            if (end_occupied && post_mark_cells > 0.0)
            {
              // apply any post_mark_cells
              octomap::point3d step_point = end;
              const unsigned int step_cnt = std::round(post_mark_cells * 2);
              const double step_amt = 0.5 * tree.getResolution();
              octomap::point3d step_vector = direction * step_amt;
              for (unsigned int step=0; step<step_cnt; ++step)
              {
                step_point += step_vector;
                octomap::OcTreeKey mark_key;
                if (tree.coordToKeyChecked(step_point, mark_key))
                {
                  if (truncate_floor_ && mark_key[2] < truncate_floor_z_)
                  {
                    break;
                  }
                  if (isKeyOutOfBounds(mark_key))
                  {
                    break;
                  }

                  if (insertOccupied(mark_key))
                  {
                    cells_added = true;
                    if (furthest_touched_key)
                    {
                      *furthest_touched_key = mark_key;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  if (!skip_tracing)
  {
    // Now that marking is complete, adjust the end for ray_shrink_cells
    if (ray_shrink_cells > 0.0)
    {
      const double ray_shrink_length = ray_shrink_cells * tree.getResolution();
      if (ray.norm() <= ray_shrink_length)
      {
        // Our ray will shrink to nothing.
        end = origin;
      }
      else
      {
        end -= direction * ray_shrink_length;
      }
    }

    if (insertFreeRay(tree, origin, end))
    {
      if (!cells_added && furthest_touched_key)
      {
        *furthest_touched_key = end_key;
      }
      cells_added = true;
    }
  }

  return cells_added;
}

// Returns true if a node was inserted, false if the node already existed
bool SensorUpdateKeyMap::insert(const octomap::OcTreeKey& key, bool value)
{
  if (isKeyOutOfBounds(key))
  {
    return false;
  }

  // apply floor truncation to the key first, moving the point to the floor
  // if floor truncation is enabled and the z coord is below the floor
  if (truncate_floor_ && key[2] < truncate_floor_z_)
  {
    octomap::OcTreeKey trunc_key(key);
    trunc_key.k[2] = truncate_floor_z_;
    if (value)
    {
      return impl_->insertOccupied(trunc_key);
    }
    else
    {
      return impl_->insertFree(trunc_key);
    }
  }
  else
  {
    if (value)
    {
      return impl_->insertOccupied(key);
    }
    else
    {
      return impl_->insertFree(key);
    }
  }
}

void SensorUpdateKeyMap::setBounds(const octomap::OcTreeKey& min_key,
                                   const octomap::OcTreeKey& max_key)
{
  bool use_array = false;
  max_key_ = max_key;
  min_key_ = min_key;
  octomap::OcTreeKey key_diff;
  key_diff[0] = max_key[0] - min_key[0];
  key_diff[1] = max_key[1] - min_key[1];
  key_diff[2] = max_key[2] - min_key[2];
  // Check each diff to ensure each individual dimension is below the threshold
  // to prevent overflowing the multiplication.
  if (key_diff[0] < voxel_volume_array_threshold &&
      key_diff[1] < voxel_volume_array_threshold &&
      key_diff[2] < voxel_volume_array_threshold)
  {
    size_t voxel_volume = static_cast<size_t>(key_diff[0]) * key_diff[1] * key_diff[2];
    if (voxel_volume < voxel_volume_array_threshold)
    {
      use_array = true;
    }
  }
  bool impl_is_array = (dynamic_cast<SensorUpdateKeyMapArrayImpl*>(impl_.get()) != nullptr);
  // (re)allocate impl if necessary
  if (impl_.get() == nullptr || impl_is_array != use_array)
  {
    if (use_array)
    {
      impl_.reset(new SensorUpdateKeyMapArrayImpl(min_key, max_key));
    }
    else
    {
      impl_.reset(new SensorUpdateKeyMapHashImpl());
    }
  }
  else
  {
    impl_->clear();
  }
  impl_->setBounds(min_key, max_key);
}

VoxelState SensorUpdateKeyMap::find(const octomap::OcTreeKey& key) const
{
  if (isKeyOutOfBounds(key))
  {
    return UNKNOWN;
  }
  else
  {
    return impl_->find(key);
  }
}

}  // namespace octomap_server
