#ifndef OCTOMAP_SENSOR_UPDATE_KEY_MAP_HASH_IMPL_H
#define OCTOMAP_SENSOR_UPDATE_KEY_MAP_HASH_IMPL_H

#include <vector>
#include <octomap_server/SensorUpdateKeyMapImpl.h>

namespace octomap_server {

// A key map implementation using an optimized hash table which uses pre-allocated
// memory for all nodes in the hash table, and stores true for occupied, false
// for not. The class is optimized for storing a periodic sensor update by
// keeping an instance allocated, inserting the readings for an update, and
// then calling clear().
// Memory is not freed by clear() but re-used.
// clear() is optimized by not destroying the individual nodes, just zeroing
// out the base hash table of pointers. This is possible since an OcTreeKey is
// just plain-old-data. The hash table itself is never shrunk. This allows for
// optimal use of a key set during a sensor update by re-using the same
// object for each update. It will grow to match the size of the
// largest update over the life of the OctomapServer, which is bounded in
// space. This way, no allocation/deallocation is happening at all, which is
// the biggest cost of the sensor updates, and hence the biggest cost of
// OctomapServer. Not having to call the underlying memory allocator for every
// node saves considerable CPU time and memory fragementation.
class SensorUpdateKeyMapHashImpl : public SensorUpdateKeyMapImpl
{
public:
  SensorUpdateKeyMapHashImpl(size_t initial_capacity=1024, double max_load_factor=0.5);
  // non-copyable. we could implement a deep copy, but there is no use case
  // for it yet.
  SensorUpdateKeyMapHashImpl(const SensorUpdateKeyMapHashImpl &rhs) = delete;
  virtual ~SensorUpdateKeyMapHashImpl();

  virtual bool insertFree(const octomap::OcTreeKey& key);
  virtual bool insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count);
  virtual bool insertOccupied(const octomap::OcTreeKey& key);
  virtual void clear();
  virtual void setBounds(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key) {}

  virtual void apply(OcTreeT* tree) const;

  virtual VoxelState find(const octomap::OcTreeKey& key) const;

private:
  VoxelState find(const octomap::OcTreeKey& key, size_t hash) const;

  struct Node
  {
    octomap::OcTreeKey key;
    bool value;
    struct Node *next;
  };

  std::vector<Node*> table_;

  Node* allocNodeFromCache();

  void initializeNodeCache(size_t capacity);
  void destroyNodeCache();
  unsigned char * getNewNodePtr();
  void resetNodeCache();
  void doubleCapacity();
  void resizeIfNecessary();

  double max_load_factor_;
  unsigned char *node_cache_;
  size_t node_cache_capacity_;
  size_t node_cache_size_;
  uint64_t table_mask_;
  size_t table_capacity_;
  void calculateTableCapacity() {
    uint32_t capacity = node_cache_capacity_ / max_load_factor_;
    uint32_t table_order_ = 33 -__builtin_clz(capacity-1);
    table_capacity_ = (1<<table_order_);
    table_mask_ = (table_capacity_-1);
  }

  bool insertFreeByIndexImpl(const octomap::OcTreeKey& key, size_t index, Node** table);
};

} // end namespace octomap_server

#endif  // OCTOMAP_SENSOR_UPDATE_KEY_MAP_HASH_IMPL_H
