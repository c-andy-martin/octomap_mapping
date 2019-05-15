#ifndef OCTOMAP_SENSOR_UPDATE_KEY_MAP_HH
#define OCTOMAP_SENSOR_UPDATE_KEY_MAP_HH

#include <limits>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

namespace octomap_server {

struct SensorUpdateKeyMapNode {
  octomap::OcTreeKey key;
  bool value;
  struct SensorUpdateKeyMapNode *next;
};

// An optimized replacement for octomap::KeyMap which uses pre-allocated
// memory for all nodes in the hash table, and stores true for occupied, false
// for not. The class is optimized for storing a periodic sensor update by
// keeping an instance allocated, inserting the readings for an update, and
// then calling clear().
// Memory is not freed by clear() but re-used.
// clear() is optimized by not destroying the individual nodes, just zeroing
// out the base hash table of pointers. This is possible since an OcTreeKey is
// just plain-old-data. The hash table itself is never shrunk. This allows for
// optimal use of a key set during a sensor update by re-using the same
// SensorUpdateKeyMap object for each update. It will grow to match the size of the
// largest update over the life of the OctomapServer, which is bounded in
// space. This way, no allocation/deallocation is happening at all, which is
// the biggest cost of the sensor updates, and hence the biggest cost of
// OctomapServer. Not having to call the underlying memory allocator for every
// node saves considerable CPU time and memory fragementation.
class SensorUpdateKeyMap {

public:
  typedef SensorUpdateKeyMapNode Node;
  SensorUpdateKeyMap(size_t initial_capacity=1024, double max_load_factor=0.5);
  ~SensorUpdateKeyMap();

  void setFloorTruncation(octomap::key_type floor_z);
  void setMinKey(const octomap::OcTreeKey& min_key) {min_key_ = min_key;}
  void setMaxKey(const octomap::OcTreeKey& max_key) {max_key_ = max_key;}
  inline bool isKeyOutOfBounds(const octomap::OcTreeKey& key)
  {
    return !(min_key_[0] <= key[0] && key[0] <= max_key_[0] &&
        min_key_[1] <= key[1] && key[1] <= max_key_[1] &&
        min_key_[2] <= key[2] && key[2] <= max_key_[2]);
  }
  bool insertFree(octomap::OcTreeKey& key);
  bool insertFreeRay(const octomap::point3d& origin, const octomap::point3d& end,
                     const octomap::OcTreeKey& key_origin, const octomap::OcTreeKey& key_end,
                     const octomap::point3d& origin_boundary,
                     double resolution);
  bool insertOccupied(octomap::OcTreeKey& key);

  // Returns true if the key was inserted, false if the key already existed
  // Invalidates all prior iterators if a key is inserted.
  // Unlike std::unordered_map, if the key exists, and value is true, the old
  // value is overwritten with true. This is because we are modeling a sensor
  // and want a voxel with both a ground and non-ground point to be always
  // counted as occupeid.
  bool insert(const octomap::OcTreeKey& key, bool value = false);

  // Insert from any container of OcTreeKey's
  // This will only insert clear space (value is always false)
  template <class InputIt>
  void insert(InputIt first, InputIt last) {
    while (first != last) {
      insert(*first);
      ++first;
    }
  }

  // Empty the sensor update key map
  // Invalidates all iterators.
  void clear();

  class iterator {
    public:
      iterator(const SensorUpdateKeyMap& key_set, size_t table_index, const Node *node)
        : key_map_(key_set), table_index_(table_index), node_(node) {}
      iterator(const iterator& rhs)
        : key_map_(rhs.key_map_), table_index_(rhs.table_index_), node_(rhs.node_) {}
      const Node& operator*() { return *node_; }
      const Node* operator->() { return node_; }
      operator bool() const { return node_ != NULL; }
      bool operator==(const iterator& rhs) { return node_ == rhs.node_; }
      bool operator!=(const iterator& rhs) { return !operator==(rhs); }
      iterator& operator++() {
        if (node_) {
          if (node_->next) {
            // there was a chained node, go to next node in chain
            node_ = node_->next;
          } else {
            // scan down the table for a non-NULL entry
            while (++table_index_ < key_map_.table_.size()) {
              if (key_map_.table_[table_index_] != NULL) {
                node_ = key_map_.table_[table_index_];
                break;
              }
            }
            if (table_index_ == key_map_.table_.size()) {
              // no entries found, set node_ to NULL
              node_ = NULL;
            }
          }
        }
        return *this;
      }
      iterator operator++(int) {
        iterator rv(*this);
        operator++();
        return rv;
      }
    protected:
      const SensorUpdateKeyMap &key_map_;
      size_t table_index_;
      const Node *node_;
  };

  iterator find(const octomap::OcTreeKey& key);
  iterator begin();
  iterator end();

protected:
  std::vector<Node*> table_;

  bool insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count);
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

  octomap::OcTreeKey *free_cells_;
  size_t free_cells_capacity_;

  bool insertFreeByIndexImpl(const octomap::OcTreeKey& key, size_t index);

  bool truncate_floor_;
  octomap::key_type truncate_floor_z_;

  octomap::OcTreeKey min_key_;
  octomap::OcTreeKey max_key_;
private:
  // non-copyable. we could implement a deep copy, but there is no use case
  // for it yet.
  SensorUpdateKeyMap(const SensorUpdateKeyMap &rhs) {}
};

} // end namespace octomap_server

#endif
