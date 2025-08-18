// #pragma once
// #include <algorithm>
// #include <cstddef> // for size_t
// #include <limits>
// #include <memory>
// #include <vector>

// struct SphereData {
//     // Explicitly use individual floats instead of vec3
//     float centerX, centerY, centerZ;
//     float radius;
//     float colorR, colorG, colorB;
//     float padding;

//     SphereData() = default;
// };

// // Axis-aligned bounding box
// struct AABB {
//     float minX, minY, minZ;
//     float maxX, maxY, maxZ;

//     AABB()
//         : minX(std::numeric_limits<float>::max()), minY(std::numeric_limits<float>::max()),
//           minZ(std::numeric_limits<float>::max()), maxX(std::numeric_limits<float>::lowest()),
//           maxY(std::numeric_limits<float>::lowest()), maxZ(std::numeric_limits<float>::lowest()) {}

//     AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
//         : minX(minX), minY(minY), minZ(minZ), maxX(maxX), maxY(maxY), maxZ(maxZ) {}

//     // Calculate surface area for SAH
//     float surfaceArea() const {
//         float dx = maxX - minX;
//         float dy = maxY - minY;
//         float dz = maxZ - minZ;
//         return 2.0f * (dx * dy + dx * dz + dy * dz);
//     }

//     // Expand to include another AABB
//     void expand(const AABB& other) {
//         minX = std::min(minX, other.minX);
//         minY = std::min(minY, other.minY);
//         minZ = std::min(minZ, other.minZ);
//         maxX = std::max(maxX, other.maxX);
//         maxY = std::max(maxY, other.maxY);
//         maxZ = std::max(maxZ, other.maxZ);
//     }

//     // Get AABB for a sphere
//     static AABB fromSphere(const SphereData& sphere) {
//         return AABB(
//             sphere.centerX - sphere.radius, sphere.centerY - sphere.radius,
//             sphere.centerZ - sphere.radius, sphere.centerX + sphere.radius,
//             sphere.centerY + sphere.radius, sphere.centerZ + sphere.radius);
//     }
// };

// // CPU-side BVH node for construction and updates
// struct BVHNodeCPU {
//     AABB bounds;
//     std::unique_ptr<BVHNodeCPU> left;
//     std::unique_ptr<BVHNodeCPU> right;
//     size_t sphereIndex; // Valid only for leaf nodes
//     bool isLeaf;

//     BVHNodeCPU() : sphereIndex(0), isLeaf(false) {}

//     // Constructor for leaf node
//     BVHNodeCPU(size_t index, const AABB& aabb) : bounds(aabb), sphereIndex(index), isLeaf(true) {}
// };

// // GPU-friendly BVH node for shader usage
// struct BVHNodeGPU {
//     float minX, minY, minZ, maxX, maxY, maxZ;
//     int leftChild;   // Index to left child, -1 if leaf
//     int rightChild;  // Index to right child, -1 if leaf
//     int sphereIndex; // Sphere index for leaf nodes, -1 for internal nodes
//     int padding;

//     BVHNodeGPU()
//         : minX(0), minY(0), minZ(0), maxX(0), maxY(0), maxZ(0), leftChild(-1), rightChild(-1),
//           sphereIndex(-1), padding(0) {}
// };

// class DBVH {
// private:
//     std::unique_ptr<BVHNodeCPU> root;
//     std::vector<BVHNodeGPU> gpuNodes;

//     // Helper structures for SAH evaluation
//     struct SplitCandidate {
//         int axis; // 0=X, 1=Y, 2=Z
//         float position;
//         float cost;
//     };

//     // Build BVH recursively using SAH
//     std::unique_ptr<BVHNodeCPU> buildRecursive(
//         const std::vector<SphereData>& spheres, std::vector<size_t>& indices, size_t start,
//         size_t end);

//     // Find best split using SAH
//     SplitCandidate findBestSplit(
//         const std::vector<SphereData>& spheres, const std::vector<size_t>& indices, size_t start,
//         size_t end, const AABB& bounds);

//     // Calculate SAH cost for a split
//     float calculateSAHCost(
//         const std::vector<SphereData>& spheres, const std::vector<size_t>& indices, size_t start,
//         size_t end, int axis, float position, const AABB& bounds);

//     // Calculate bounding box for a range of spheres
//     AABB calculateBounds(
//         const std::vector<SphereData>& spheres, const std::vector<size_t>& indices, size_t start,
//         size_t end);

//     // Insert a sphere into the BVH using SAH-based cost evaluation
//     std::unique_ptr<BVHNodeCPU> insertRecursive(
//         std::unique_ptr<BVHNodeCPU> node,
//         const std::vector<SphereData>& spheres,
//         size_t index);

//     // Find the best sibling node for insertion using SAH cost
//     BVHNodeCPU* findBestSibling(
//         BVHNodeCPU* root,
//         const std::vector<SphereData>& spheres,
//         size_t index);

//     // Calculate insertion cost for making a node a sibling
//     float calculateInsertionCost(
//         BVHNodeCPU* node,
//         const AABB& newBounds,
//         const std::vector<SphereData>& spheres);

//     // Calculate indirect cost (ancestor bounds expansion)
//     float calculateIndirectCost(
//         BVHNodeCPU* node,
//         const AABB& oldBounds,
//         const AABB& newBounds);

//     // Count leaves in a subtree
//     int countLeaves(const BVHNodeCPU* node);

//     // Remove a sphere from the BVH
//     std::unique_ptr<BVHNodeCPU> removeRecursive(
//         std::unique_ptr<BVHNodeCPU> node,
//         const std::vector<SphereData>& spheres,
//         size_t index);

//     // Find and remove a sphere, returning the removed node
//     std::unique_ptr<BVHNodeCPU> findAndRemove(
//         std::unique_ptr<BVHNodeCPU> node,
//         size_t index,
//         bool& found);

//     // Update bounding boxes after modification
//     void updateBounds(BVHNodeCPU* node, const std::vector<SphereData>& spheres);

//     // Convert CPU BVH to GPU format
//     void convertToGPU();

//     // Recursive helper for GPU conversion
//     int convertNodeToGPU(const BVHNodeCPU* node);

//     // Check if a sphere fits well in a node (for update operation)
//     bool sphereFitsWell(const BVHNodeCPU* node, const SphereData& sphere);

// public:
//     DBVH() = default;

//     // Build initial BVH from scratch
//     void build(const std::vector<SphereData>& spheres, std::vector<size_t>& indices);

//     // Insert a new sphere into the BVH
//     void insert(const std::vector<SphereData>& spheres, size_t index);

//     // Remove a sphere from the BVH
//     void remove(const std::vector<SphereData>& spheres, size_t index);

//     // Update BVH when a sphere moves
//     void update(const std::vector<SphereData>& spheres, size_t index);

//     // Get GPU-friendly BVH data
//     const std::vector<BVHNodeGPU>& getGPUNodes() const { return gpuNodes; }

//     // Get root node for debugging
//     const BVHNodeCPU* getRoot() const { return root.get(); }
// };
