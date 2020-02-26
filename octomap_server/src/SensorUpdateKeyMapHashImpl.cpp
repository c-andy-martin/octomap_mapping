#include <octomap_server/SensorUpdateKeyMapHashImpl.h>
#include <cstring>

namespace octomap_server {

SensorUpdateKeyMapHashImpl::SensorUpdateKeyMapHashImpl(size_t initial_capacity, double max_load_factor)
  : max_load_factor_(max_load_factor)
{
  initializeNodeCache(initial_capacity);
  calculateTableCapacity();
  table_.resize(table_capacity_, NULL);
}

SensorUpdateKeyMapHashImpl::~SensorUpdateKeyMapHashImpl()
{
  table_.clear();
  destroyNodeCache();
}

void SensorUpdateKeyMapHashImpl::clear()
{
  // clear our node pointers out. do not de-allocate the table, we will re-use
  // whatever size it is.
  std::memset(table_.data(), 0, table_.size() * sizeof(Node*));
  // reset our node cache
  resetNodeCache();
}

VoxelState SensorUpdateKeyMapHashImpl::find(const octomap::OcTreeKey& key) const
{
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  return find(key, hash);
}

VoxelState SensorUpdateKeyMapHashImpl::find(const octomap::OcTreeKey& key, size_t hash) const
{
  size_t index = hash & table_mask_;
  Node *node = table_[index];
  while (node)
  {
    if (node->key == key)
    {
      return node->value;
    }
    node = node->next;
  }
  return voxel_state::UNKNOWN;
}

bool SensorUpdateKeyMapHashImpl::insertFreeByIndexImpl(const octomap::OcTreeKey& key, size_t index, Node** table)
{
  Node *table_node = table[index];
  Node *node = table_node;
  while (node) {
    if (node->key == key) {
      return false;
    }
    node = node->next;
  }

  // insert a new node at the head of the chain for the bucket
  node = allocNodeFromCache();
  node->key = key;
  node->value = voxel_state::FREE;
  node->next = table_node;
  table[index] = node;
  return true;
}

// Returns true if a node was inserted, false if the node already existed
bool SensorUpdateKeyMapHashImpl::insertFree(const octomap::OcTreeKey& key)
{
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash & table_mask_;
  bool rv = insertFreeByIndexImpl(key, index, table_.data());
  // if we have just run out of room, this will resize our set
  if (rv) resizeIfNecessary();
  return rv;
}

bool SensorUpdateKeyMapHashImpl::insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count)
{
  if (free_cells_count == 0) {
    return false;
  }
  while (node_cache_size_ + free_cells_count + 1 >= node_cache_capacity_) {
    doubleCapacity();
  }
  const size_t n = ((free_cells_count-1) & ~7) + 8;
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

  Node** table = table_.data();
  for (i=0; i<free_cells_count; ++i)
  {
    insertFreeByIndexImpl(free_cells[i], indecies[i], table);
  }
  return true;
}

bool SensorUpdateKeyMapHashImpl::insertOccupied(const octomap::OcTreeKey& key)
{
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash & table_mask_;
  Node *node = table_[index];
  while (node)
  {
    if (node->key == key)
    {
      // Ensure that this key maps to occupied
      node->value = voxel_state::OCCUPIED;
      return false;
    }
    node = node->next;
  }

  // insert a new node at the head of the chain for the bucket
  node = allocNodeFromCache();
  node->key = key;
  node->value = voxel_state::OCCUPIED;
  node->next = table_[index];
  table_[index] = node;

  // if we have just run out of room, this will resize our set
  resizeIfNecessary();
  return true;
}

void SensorUpdateKeyMapHashImpl::insertInner(const octomap::OcTreeKey& key)
{
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash & table_mask_;
  Node *node = table_[index];
  while (node)
  {
    if (node->key == key)
    {
      // do not override an existing node
      return;
    }
    node = node->next;
  }

  // insert a new node at the head of the chain for the bucket
  node = allocNodeFromCache();
  node->key = key;
  node->value = voxel_state::INNER;
  node->next = table_[index];
  table_[index] = node;

  // if we have just run out of room, this will resize our set
  resizeIfNecessary();
}

void SensorUpdateKeyMapHashImpl::insert(const octomap::OcTreeKey& key, VoxelState state)
{
  octomap::OcTreeKey::KeyHash hasher;
  size_t hash = hasher(key);
  size_t index = hash & table_mask_;
  Node *node = table_[index];
  while (node)
  {
    if (node->key == key)
    {
      // do not override an existing node
      return;
    }
    node = node->next;
  }

  // insert a new node at the head of the chain for the bucket
  node = allocNodeFromCache();
  node->key = key;
  node->value = state;
  node->next = table_[index];
  table_[index] = node;

  // if we have just run out of room, this will resize our set
  resizeIfNecessary();
}

void SensorUpdateKeyMapHashImpl::downSample(const octomap::OcTreeSpace& tree, SensorUpdateKeyMapImpl* output_map) const
{
  SensorUpdateKeyMapImpl::downSample(tree, output_map);
  unsigned int target_depth = getDepth() - output_map->getLevel();
  unsigned int voxel_state_counts[voxel_state::MAX] = {0};
  // first ensure the output bounds are set correctly
  output_map->setBounds(min_key_, max_key_);

  octomap::key_type center_offset_key = octomap::computeCenterOffsetKey(target_depth, tree.getCenterKey());

  // loop over the hash table
  const unsigned int table_size = table_.size();
  auto table = table_.data();
  for (unsigned int i=0; i<table_size; ++i)
  {
    const Node* node = table_[i];
    while (node)
    {
      octomap::OcTreeKey target_key = tree.adjustKeyAtDepth(node->key, target_depth);
      // if the output map doesn't yet have this key, time to add it.
      if (output_map->find(target_key) == voxel_state::UNKNOWN)
      {
        unsigned int free_count=0;
        unsigned int inner_count=0;
        voxel_state_counts[voxel_state::FREE] = 0;
        voxel_state_counts[voxel_state::OCCUPIED] = 0;
        for (unsigned int i=0; i<8; ++i)
        {
          octomap::OcTreeKey child_key;
          octomap::computeChildKey(i, center_offset_key, target_key, child_key);
          const VoxelState voxel_state = find(child_key);
          ++voxel_state_counts[voxel_state];
        }
        if (voxel_state_counts[voxel_state::FREE] == 8)
        {
          output_map->insertFree(target_key);
        }
        else if (voxel_state_counts[voxel_state::OCCUPIED] == 8)
        {
          output_map->insertOccupied(target_key);
        }
        else
        {
          // There must be at least one child key to get here
          // Mark the free and/or occupied bits to match the children that are
          // present. This allows for implemetations that do not store free
          // space to skip large sections of tree that only have free space
          // under it, for example.
          VoxelState new_state = voxel_state::INNER;
          if (voxel_state_counts[voxel_state::FREE] > 0 ||
              voxel_state_counts[voxel_state::INNER | voxel_state::FREE] > 0)
          {
            new_state |= voxel_state::FREE;
          }
          if (voxel_state_counts[voxel_state::OCCUPIED] > 0 ||
              voxel_state_counts[voxel_state::INNER | voxel_state::OCCUPIED] > 0)
          {
            new_state |= voxel_state::OCCUPIED;
          }
          if (voxel_state_counts[voxel_state::INNER | voxel_state::OCCUPIED | voxel_state::FREE] > 0)
          {
            new_state |= voxel_state::FREE;
            new_state |= voxel_state::OCCUPIED;
          }
          output_map->insert(target_key, new_state);
        }
      }
      node = node->next;
    }
  }
}

void SensorUpdateKeyMapHashImpl::resizeIfNecessary()
{
  assert(node_cache_size_ <= node_cache_capacity_);
  // If we have run out of size in the node cache, double it.
  if (node_cache_size_ == node_cache_capacity_) {
    doubleCapacity();
  }
}

void SensorUpdateKeyMapHashImpl::initializeNodeCache(size_t capacity)
{
  node_cache_capacity_ = capacity;
  node_cache_ = new unsigned char[sizeof(Node) * node_cache_capacity_];
  node_cache_size_ = 0;
}

void SensorUpdateKeyMapHashImpl::destroyNodeCache()
{
  resetNodeCache();
  node_cache_capacity_ = 0;
  delete [] node_cache_;
}

// Double the capacity of our set.
// We do this by re-allocating our table to keep the max load factor,
// allocating a new, double-sized node_cache, and then copying the old table
// to the new.
void SensorUpdateKeyMapHashImpl::doubleCapacity()
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
    while (node)
    {
      if (node->value == voxel_state::FREE)
      {
        insertFree(node->key);
      }
      else if (node->value == voxel_state::OCCUPIED)
      {
        insertOccupied(node->key);
      }
      else if (node->value == voxel_state::INNER)
      {
        insertInner(node->key);
      }
      node = node->next;
    }
  }
  // Destroy the old cache
  delete [] old_cache;
}

unsigned char * SensorUpdateKeyMapHashImpl::getNewNodePtr() {
  assert(node_cache_size_ < node_cache_capacity_);
  unsigned char * new_node_memory = node_cache_ + node_cache_size_ * sizeof(Node);
  node_cache_size_++;
  return new_node_memory;
}

SensorUpdateKeyMapHashImpl::Node* SensorUpdateKeyMapHashImpl::allocNodeFromCache()
{
  return new(getNewNodePtr()) Node;
}

void SensorUpdateKeyMapHashImpl::resetNodeCache()
{
  // We do not want to reclaim any memory, keep the cache around. It can
  // only grow to the size of the largest sensor update, which is what we
  // want.
  // Technically, we should call the destructor of every element here,
  // however for efficiency, leverage the fact that we are storing
  // plain-old-data and do nothing on destruction
  node_cache_size_ = 0;
}

#if 0
void SensorUpdateKeyMapHashImpl::apply(OcTreeT* tree) const
{
  const unsigned int table_size = table_.size();
  auto table = table_.data();
  for (unsigned int i=0; i<table_size; ++i)
  {
    const Node* node = table_[i];
    while (node)
    {
      tree->updateNode(node->key, node->value);
      node = node->next;
    }
  }
}
#endif

} // end namespace octomap_server
