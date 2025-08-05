#ifndef BVH_H
#define BVH_H

#include "glm/glm.hpp"
#include <algorithm>
#include <memory>
#include <vector>

struct BVHNodeGPU {
    glm::vec4 aabbMin; // xyz = min bounds, w = leftChild index (or -1 if leaf)
    glm::vec4 aabbMax; // xyz = max bounds, w = rightChild index (or sphere count if leaf)
    glm::uvec4 data; // x = first sphere index (if leaf), y = sphere count (if leaf), z,w = padding

    BVHNodeGPU() : aabbMin(0), aabbMax(0), data(0) {}
};

// struct SphereData {
//   glm::vec3 center;
//   float radius;
//   glm::vec3 color;
//   float padding; // For alignment
// };

struct SphereData {
    // Explicitly use individual floats instead of vec3
    float centerX, centerY, centerZ;
    float radius;
    float colorR, colorG, colorB;
    float padding;

    // Constructor for convenience
    SphereData(const glm::vec3& center, float r, const glm::vec3& color)
        : centerX(center.x), centerY(center.y), centerZ(center.z), radius(r), colorR(color.r),
          colorG(color.g), colorB(color.b), padding(0.0f) {}

    SphereData() = default;
};

struct SphereDataPacked {
    glm::vec4 centerAndRadius;
    glm::vec4 color;

    SphereDataPacked(const SphereData& data)
        : centerAndRadius(data.centerX, data.centerY, data.centerZ, data.radius),
          color(data.colorR, data.colorG, data.colorB, data.padding) {}
};

struct BVHNodeCPU {
    glm::vec3 aabbMin, aabbMax;
    std::unique_ptr<BVHNodeCPU> leftChild;
    std::unique_ptr<BVHNodeCPU> rightChild;
    std::vector<uint32_t> sphereIndices; // leaf node stores sphere indices
    bool isLeaf = false;

    BVHNodeCPU() = default;

    void calculateAABB(const std::vector<SphereDataPacked>& spheres) {
        if (sphereIndices.empty())
            return;

        aabbMin = glm::vec3(FLT_MAX);
        aabbMax = glm::vec3(-FLT_MAX);

        for (uint32_t idx : sphereIndices) {
            glm::vec3 center = glm::vec3(spheres[idx].centerAndRadius);
            float radius = spheres[idx].centerAndRadius.w;

            aabbMin = glm::min(aabbMin, center - radius);
            aabbMax = glm::max(aabbMax, center + radius);
        }
    }

    float getSurfaceArea() const {
        glm::vec3 extent = aabbMax - aabbMin;
        return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
    }

    float getVolume() const {
        glm::vec3 extent = aabbMax - aabbMin;
        return extent.x * extent.y * extent.z;
    }
};

struct SplitInfo {
    int axis = -1;
    float position = 0.0f;
    float cost = FLT_MAX;
    size_t leftCount = 0;
    size_t rightCount = 0;

    bool isValid() const {
        return axis != -1 && cost < FLT_MAX;
    }
};

class BVH {
public:
    static constexpr int MAX_LEAF_SIZE = 4;
    static constexpr int MAX_DEPTH = 20;
    static constexpr float TRAVERSAL_COST = 1.0f;
    static constexpr float INTERSECTION_COST = 1.0f;
    static constexpr int SAH_BUCKETS = 8;

private:
    std::unique_ptr<BVHNodeCPU> root;
    std::vector<BVHNodeGPU> gpuNodes;
    std::vector<uint32_t> gpuSphereIndices;

public:
    // construct BVH from spheres
    void build(const std::vector<SphereDataPacked>& spheres);

    // rebuild BVH (for dynamic updates)
    void rebuild(const std::vector<SphereDataPacked>& spheres);

    // convert to GPU format
    void convertToGPUFormat();

    // get GPU data
    const std::vector<BVHNodeGPU>& getGPUNodes() const {
        return gpuNodes;
    }
    const std::vector<uint32_t>& getGPUSphereIndices() const {
        return gpuSphereIndices;
    }

    size_t getNodeCount() const;
    size_t getMaxDepth() const;
    void printStats() const;

private:
    // recursive build
    std::unique_ptr<BVHNodeCPU> buildRecursive(
        const std::vector<SphereDataPacked>& spheres, std::vector<uint32_t>& indices,
        int depth = 0);

    // find best split
    SplitInfo findBestSplit(
        const std::vector<SphereDataPacked>& spheres, const std::vector<uint32_t>& indices,
        const glm::vec3& aabbMin, const glm::vec3& aabbMax);

    // split
    size_t partitionSpheres(
        const std::vector<SphereDataPacked>& spheres, std::vector<uint32_t>& indices,
        const SplitInfo& split);

    // convert a single node to GPU format
    size_t convertNodeToGPU(const BVHNodeCPU* node, std::vector<BVHNodeGPU>& gpuNodes);
};

#endif // BVH_H