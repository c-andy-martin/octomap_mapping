#include "octomap_server/SensorUpdateKeyMap.h"

namespace octomap_server {

SensorUpdateKeyMap::iterator SensorUpdateKeyMap::begin()
{
  for (size_t i=0; i<table_.size(); ++i)
  {
    if (table_[i] != NULL)
    {
      return SensorUpdateKeyMap::iterator(*this, i, table_[i]);
    }
  }
  return end();
}

SensorUpdateKeyMap::iterator SensorUpdateKeyMap::end()
{
  return SensorUpdateKeyMap::iterator(*this, table_.size(), NULL);
}

void SensorUpdateKeyMap::clear()
{
  // clear our node pointers out. do not de-allocate the table, we will re-use
  // whatever size it is.
  std::fill(table_.begin(), table_.end(), static_cast<Node*>(NULL));
  // reset our node cache
  resetNodeCache();
}

SensorUpdateKeyMap::SensorUpdateKeyMap(size_t initial_capacity, double max_load_factor)
  : max_load_factor_(max_load_factor)
  , free_cells_(NULL)
  , free_cells_capacity_(0)
  , truncate_floor_(false)
{
  initializeNodeCache(initial_capacity);
  calculateTableCapacity();
  table_.resize(table_capacity_, NULL);
}

SensorUpdateKeyMap::~SensorUpdateKeyMap()
{
  delete [] free_cells_;
  table_.clear();
  destroyNodeCache();
}

SensorUpdateKeyMap::iterator SensorUpdateKeyMap::find(const octomap::OcTreeKey& key)
{
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash % table_.size();
  SensorUpdateKeyMap::Node *node = table_[index];
  while (node) {
    if (node->key == key) {
      return SensorUpdateKeyMap::iterator(*this, index, node);
    }
    node = node->next;
  }
  return end();
}

void SensorUpdateKeyMap::setFloorTruncation(octomap::key_type floor_z)
{
  truncate_floor_ = true;
  truncate_floor_z_ = floor_z;
}

inline bool SensorUpdateKeyMap::insertFreeByIndexImpl(const octomap::OcTreeKey& key, size_t index)
{
  SensorUpdateKeyMap::Node *node = table_[index];
  while (node) {
    if (node->key == key) {
      return false;
    }
    node = node->next;
  }

  // insert a new node at the head of the chain for the bucket
  node = allocNodeFromCache();
  node->key = key;
  node->value = false;
  node->next = table_[index];
  table_[index] = node;
  return true;
}

// Returns true if a node was inserted, false if the node already existed
bool SensorUpdateKeyMap::insertFree(octomap::OcTreeKey& key)
{
  // apply floor truncation to the key first, moving the point to the floor
  // if floor truncation is enabled and the z coord is below the floor
  if (truncate_floor_ && key[2] < truncate_floor_z_) {
    key.k[2] = truncate_floor_z_;
  }
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash % table_.size();
  bool rv = insertFreeByIndexImpl(key, index);
  // if we have just run out of room, this will resize our set
  if (rv) resizeIfNecessary();
  return rv;
}

bool SensorUpdateKeyMap::insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count)
{
  if (free_cells_count == 0) {
    return false;
  }
  while (node_cache_size_ + free_cells_count + 1 >= node_cache_capacity_) {
    doubleCapacity();
  }
  const size_t n = ((free_cells_count-1) & ~7) + 8;
  const size_t tn = table_.size();
  size_t x_keys[n] __attribute__((__aligned__(64)));
  size_t y_keys[n] __attribute__((__aligned__(64)));
  size_t z_keys[n] __attribute__((__aligned__(64)));
  size_t indecies[n] __attribute__((__aligned__(64)));
  unsigned int i=0, j=0;
  for (i=0; i<free_cells_count; ++i) {
    x_keys[i] = free_cells[i][0];
    y_keys[i] = free_cells[i][1];
    z_keys[i] = free_cells[i][2];
  }
  const size_t *x_keys_p = x_keys;
  const size_t *y_keys_p = y_keys;
  const size_t *z_keys_p = z_keys;
  size_t *indecies_p = indecies;
  const uint64_t table_mask = table_mask_;
  for (i=0; i<n; i+=8) {
    // structure this inner loop such that gcc vectorizes using best vector ops
    for (j=0; j<8; ++j) {
      // we can't vectorize modulus, so ensure the table is a power of two and
      // use a bit mask
      *indecies_p = (*x_keys_p + 1447 * *y_keys_p + 345637 * *z_keys_p) & table_mask;
      ++indecies_p;
      ++x_keys_p;
      ++y_keys_p;
      ++z_keys_p;
    }
  }
  for (i=0; i<free_cells_count; ++i)
  {
    insertFreeByIndexImpl(free_cells[i], indecies[i]);
  }
  return true;
}

bool SensorUpdateKeyMap::insertFreeRay(const octomap::point3d& origin, const octomap::point3d& end,
                     const octomap::OcTreeKey& key_origin, const octomap::OcTreeKey& key_end,
                     const octomap::point3d& origin_boundary,
                     double resolution)
{
  // a version of computeRayKeys from OcTreeBaseImpl.hxx which adds the ray
  // keys directly to the our SensorUpdateKeyMap

  // Nothing to do if origin and end are in same cell
  if (key_origin == key_end)
    return false;

  // Initialization
  octomap::point3d direction = (end - origin);
  float length = (float) direction.norm();
  direction /= length; // normalize vector

  int    step[3];
  double tMax[3];
  double tDelta[3];

  octomap::OcTreeKey current_key = key_origin;
  octomap::OcTreeKey justOut;

  size_t max_cells = 0;

  for(unsigned int i=0; i < 3; ++i) {
    // tally up max_cells (maximum ray trace is one cell per delta dimension)
    max_cells += abs(key_origin[i] - key_end[i]) + 1;

    // compute step direction
    if (direction(i) > 0.0) step[i] = 1;
    else if (direction(i) < 0.0) step[i] = -1;
    else step[i] = 0;

    // compute tMax, tDelta, justOut
    if (step[i] != 0) {
      // corner point of voxel (in direction of ray)
      double voxelBorder = origin_boundary(i);
      voxelBorder += (float) (step[i] * resolution * 0.5);

      tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
      tDelta[i] = resolution / fabs( direction(i) );
      justOut[i] = key_end[i] + step[i];
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
    delete [] free_cells_;
    // always over-allocate to leave some room to grow
    free_cells_capacity_ = max_cells * 2;
    free_cells_ = new octomap::OcTreeKey[free_cells_capacity_];
  }

  size_t free_cells_count = 0;

  // Incremental phase
  for (;;) {

    // add the cell
    free_cells_[free_cells_count++] = current_key;

    if (tMax[0] < tMax[1]) {
      if (tMax[0] < tMax[2]) {
        current_key[0] += step[0];
        tMax[0] += tDelta[0];
        if (current_key[0] == justOut[0]) {
          break;
        }
      } else {
        current_key[2] += step[2];
        tMax[2] += tDelta[2];
        if (current_key[2] == justOut[2]) {
          break;
        }
      }
    } else {
      if (tMax[1] < tMax[2]) {
        current_key[1] += step[1];
        tMax[1] += tDelta[1];
        if (current_key[1] == justOut[1]) {
          break;
        }
      } else {
        current_key[2] += step[2];
        tMax[2] += tDelta[2];
        if (current_key[2] == justOut[2]) {
          break;
        }
      }
    }
  }
  return insertFreeCells(free_cells_, free_cells_count);
}

// Returns true if a node was inserted, false if the node already existed
bool SensorUpdateKeyMap::insertOccupied(octomap::OcTreeKey& key)
{
  // apply floor truncation to the key first, moving the point to the floor
  // if floor truncation is enabled and the z coord is below the floor
  if (truncate_floor_ && key[2] < truncate_floor_z_) {
    key.k[2] = truncate_floor_z_;
  }
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash % table_.size();
  SensorUpdateKeyMap::Node *node = table_[index];
  while (node) {
    if (node->key == key) {
      // Ensure that this key maps to occupied (true)
      node->value = true;
      return false;
    }
    node = node->next;
  }

  // insert a new node at the head of the chain for the bucket
  node = allocNodeFromCache();
  node->key = key;
  node->value = true;
  node->next = table_[index];
  table_[index] = node;

  // if we have just run out of room, this will resize our set
  resizeIfNecessary();
  return true;
}

// Returns true if a node was inserted, false if the node already existed
bool SensorUpdateKeyMap::insert(const octomap::OcTreeKey& key, bool value)
{
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash % table_.size();
  SensorUpdateKeyMap::Node *node = table_[index];
  while (node) {
    if (node->key == key) {
      // Change the value in the map, iff it was false.
      // True latches, because for sensor updates we want a cell with both
      // ground and nonground points to be marked as occupied.
      if (value && !node->value)
      {
        node->value = true;
      }
      return false;
    }
    node = node->next;
  }

  // insert a new node at the head of the chain for the bucket
  node = allocNodeFromCache();
  node->key = key;
  node->value = value;
  node->next = table_[index];
  table_[index] = node;

  // if we have just run out of room, this will resize our set
  resizeIfNecessary();
  return true;
}

void SensorUpdateKeyMap::resizeIfNecessary()
{
  assert(node_cache_size_ <= node_cache_capacity_);
  // If we have run out of size in the node cache, double it.
  if (node_cache_size_ == node_cache_capacity_) {
    doubleCapacity();
  }
}

void SensorUpdateKeyMap::initializeNodeCache(size_t capacity)
{
  node_cache_capacity_ = capacity;
  node_cache_ = new unsigned char[sizeof(Node) * node_cache_capacity_];
  node_cache_size_ = 0;
}

void SensorUpdateKeyMap::destroyNodeCache()
{
  resetNodeCache();
  node_cache_capacity_ = 0;
  delete [] node_cache_;
}

// Double the capacity of our set.
// We do this by re-allocating our table to keep the max load factor,
// allocating a new, double-sized node_cache, and then copying the old table
// to the new.
void SensorUpdateKeyMap::doubleCapacity()
{
  // Remember the old node cache information and table
  std::vector<Node*> old_table(table_);
  size_t old_capacity = node_cache_capacity_;
  unsigned char * old_cache = node_cache_;
  // Double the capacity and allocate new cache
  initializeNodeCache(node_cache_capacity_ * 2);
  // resize the hash table to keep the load factor balanced
  calculateTableCapacity();
  table_.resize(table_capacity_);
  // clear all the old entries out
  clear();
  // Deep copy the old set to the newly allocated one.
  for (size_t i=0; i<old_table.size(); ++i) {
    Node* node = old_table[i];
    while (node) {
      insert(node->key, node->value);
      node = node->next;
    }
  }
  // Destroy the old cache
  delete [] old_cache;
}

unsigned char * SensorUpdateKeyMap::getNewNodePtr() {
  assert(node_cache_size_ < node_cache_capacity_);
  unsigned char * new_node_memory = node_cache_ + node_cache_size_ * sizeof(Node);
  node_cache_size_++;
  return new_node_memory;
}

SensorUpdateKeyMap::Node* SensorUpdateKeyMap::allocNodeFromCache()
{
  return new(getNewNodePtr()) Node;
}

void SensorUpdateKeyMap::resetNodeCache()
{
  // We do not want to reclaim any memory, keep the cache around. It can
  // only grow to the size of the largest sensor update, which is what we
  // want.
  // Technically, we should call the destructor of every element here,
  // however for efficiency, leverage the fact that we are storing
  // plain-old-data and do nothing on destruction
  node_cache_size_ = 0;
}

} // end namespace octomap_server
