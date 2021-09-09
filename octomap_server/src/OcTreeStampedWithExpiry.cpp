#include <octomap/OcTreeNode.h>
#include <octomap/ColorOcTree.h>
#include <octomap_server/OcTreeStampedWithExpiry.h>

namespace octomap_server {

// Provide the storage for the static ocTreeStampedWithExpiryMemberInit for OcTreeNode
template <>
OcTreeStampedWithExpiry<octomap::OcTreeNode>::StaticMemberInitializer OcTreeStampedWithExpiry<octomap::OcTreeNode>::ocTreeStampedWithExpiryMemberInit{};

// Provide the storage for the static ocTreeStampedWithExpiryMemberInit for ColorOcTreeNode
template <>
OcTreeStampedWithExpiry<octomap::ColorOcTreeNode>::StaticMemberInitializer OcTreeStampedWithExpiry<octomap::ColorOcTreeNode>::ocTreeStampedWithExpiryMemberInit{};

} // end namespace
