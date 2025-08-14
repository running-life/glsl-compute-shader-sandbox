#include "bvh.h"
#include <chrono>
#include <iostream>
#include <numeric>
#include <spdlog/spdlog.h>

void BVH::build(const std::vector<SphereDataPacked>& spheres) {
    auto start = std::chrono::high_resolution_clock::now();

    // create indices array
    std::vector<uint32_t> indices(spheres.size());
    std::iota(indices.begin(), indices.end(), 0);

    // recursively build BVH
    root = buildRecursive(spheres, indices, 0);

    convertToGPUFormat();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    spdlog::info(
        "BVH built in {} μs, {} nodes, max depth {}", duration.count(), getNodeCount(),
        getMaxDepth());
}

void BVH::rebuild(const std::vector<SphereDataPacked>& spheres) {
    // TODO：incremental update
    build(spheres);
}

#if 1
std::unique_ptr<BVHNodeCPU> BVH::buildRecursive(
    const std::vector<SphereDataPacked>& spheres, std::vector<uint32_t>& indices, int depth) {
    auto node = std::make_unique<BVHNodeCPU>();
    node->sphereIndices = indices;
    node->calculateAABB(spheres);

    if (indices.size() <= MAX_LEAF_SIZE || depth >= MAX_DEPTH) {
        node->isLeaf = true;
        return node;
    }

    // find best split
    SplitInfo bestSplit = findBestSplit(spheres, indices, node->aabbMin, node->aabbMax);

    // if there's no good split, create a leaf node
    if (!bestSplit.isValid()) {
        node->isLeaf = true;
        return node;
    }

    // calculate the cost of not splitting
    float leafCost = INTERSECTION_COST * indices.size();

    // if the split cost is not worth it, create a leaf node
    if (bestSplit.cost >= leafCost) {
        node->isLeaf = true;
        return node;
    }

    size_t splitIndex = partitionSpheres(spheres, indices, bestSplit);

    if (splitIndex == 0 || splitIndex == indices.size()) {
        node->isLeaf = true;
        return node;
    }

    std::vector<uint32_t> leftIndices(indices.begin(), indices.begin() + splitIndex);
    std::vector<uint32_t> rightIndices(indices.begin() + splitIndex, indices.end());

    node->leftChild = buildRecursive(spheres, leftIndices, depth + 1);
    node->rightChild = buildRecursive(spheres, rightIndices, depth + 1);
    node->isLeaf = false;

    return node;
}
#else

std::unique_ptr<BVHNodeCPU> BVH::buildRecursive(
    const std::vector<SphereDataPacked>& spheres, std::vector<uint32_t>& indices, int depth) {
    auto node = std::make_unique<BVHNodeCPU>();
    node->sphereIndices = indices;
    node->calculateAABB(spheres);

    if (indices.size() == 1) {
        node->isLeaf = true;
        return node;
    }

    if (indices.size() == 0) {
        node->isLeaf = true;
        return node;
    }

    if (depth >= MAX_DEPTH) {
        return forceSplit(spheres, indices, depth);
    }

    SplitInfo bestSplit = findBestSplit(spheres, indices, node->aabbMin, node->aabbMax);

    if (!bestSplit.isValid()) {
        return forceSplit(spheres, indices, depth);
    }

    float leafCost = INTERSECTION_COST * indices.size();

    size_t splitIndex = partitionSpheres(spheres, indices, bestSplit);

    if (splitIndex == 0 || splitIndex == indices.size()) {
        return forceSplit(spheres, indices, depth);
    }

    std::vector<uint32_t> leftIndices(indices.begin(), indices.begin() + splitIndex);
    std::vector<uint32_t> rightIndices(indices.begin() + splitIndex, indices.end());

    node->leftChild = buildRecursive(spheres, leftIndices, depth + 1);
    node->rightChild = buildRecursive(spheres, rightIndices, depth + 1);
    node->isLeaf = false;

    return node;
}

#endif

SplitInfo BVH::findBestSplit(
    const std::vector<SphereDataPacked>& spheres, const std::vector<uint32_t>& indices,
    const glm::vec3& aabbMin, const glm::vec3& aabbMax) {
    SplitInfo bestSplit;

    glm::vec3 extent = aabbMax - aabbMin;
    float parentSA = 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);

    for (int axis = 0; axis < 3; ++axis) {
        if (extent[axis] < 1e-6f)
            continue;

        struct Bucket {
            size_t count = 0;
            glm::vec3 minBounds = glm::vec3(FLT_MAX);
            glm::vec3 maxBounds = glm::vec3(-FLT_MAX);
        };

        std::vector<Bucket> buckets(SAH_BUCKETS);

        // distribute spheres into buckets
        for (uint32_t idx : indices) {
            glm::vec3 center = glm::vec3(spheres[idx].centerAndRadius);
            float radius = spheres[idx].centerAndRadius.w;

            int bucketIdx =
                static_cast<int>(SAH_BUCKETS * (center[axis] - aabbMin[axis]) / extent[axis]);
            bucketIdx = std::clamp(bucketIdx, 0, SAH_BUCKETS - 1);

            buckets[bucketIdx].count++;
            buckets[bucketIdx].minBounds = glm::min(buckets[bucketIdx].minBounds, center - radius);
            buckets[bucketIdx].maxBounds = glm::max(buckets[bucketIdx].maxBounds, center + radius);
        }

        // evaluate each possible split position
        for (int i = 0; i < SAH_BUCKETS - 1; ++i) {
            // left
            size_t leftCount = 0;
            glm::vec3 leftMin = glm::vec3(FLT_MAX);
            glm::vec3 leftMax = glm::vec3(-FLT_MAX);

            for (int j = 0; j <= i; ++j) {
                if (buckets[j].count > 0) {
                    leftCount += buckets[j].count;
                    leftMin = glm::min(leftMin, buckets[j].minBounds);
                    leftMax = glm::max(leftMax, buckets[j].maxBounds);
                }
            }

            // right
            size_t rightCount = 0;
            glm::vec3 rightMin = glm::vec3(FLT_MAX);
            glm::vec3 rightMax = glm::vec3(-FLT_MAX);

            for (int j = i + 1; j < SAH_BUCKETS; ++j) {
                if (buckets[j].count > 0) {
                    rightCount += buckets[j].count;
                    rightMin = glm::min(rightMin, buckets[j].minBounds);
                    rightMax = glm::max(rightMax, buckets[j].maxBounds);
                }
            }

            if (leftCount == 0 || rightCount == 0)
                continue;

            // calculate surface areas
            glm::vec3 leftExtent = leftMax - leftMin;
            glm::vec3 rightExtent = rightMax - rightMin;

            float leftSA = 2.0f
                           * (leftExtent.x * leftExtent.y + leftExtent.y * leftExtent.z
                              + leftExtent.z * leftExtent.x);
            float rightSA = 2.0f
                            * (rightExtent.x * rightExtent.y + rightExtent.y * rightExtent.z
                               + rightExtent.z * rightExtent.x);

            float cost = TRAVERSAL_COST + (leftSA / parentSA) * leftCount * INTERSECTION_COST
                         + (rightSA / parentSA) * rightCount * INTERSECTION_COST;

            if (cost < bestSplit.cost) {
                bestSplit.cost = cost;
                bestSplit.axis = axis;
                bestSplit.position = aabbMin[axis] + extent[axis] * (float(i + 1) / SAH_BUCKETS);
                bestSplit.leftCount = leftCount;
                bestSplit.rightCount = rightCount;
            }
        }
    }

    return bestSplit;
}

size_t BVH::partitionSpheres(
    const std::vector<SphereDataPacked>& spheres, std::vector<uint32_t>& indices,
    const SplitInfo& split) {
    return std::partition(
               indices.begin(), indices.end(),
               [&](uint32_t idx) {
                   glm::vec3 center = glm::vec3(spheres[idx].centerAndRadius);
                   return center[split.axis] < split.position;
               })
           - indices.begin();
}

void BVH::convertToGPUFormat() {
    gpuNodes.clear();
    gpuSphereIndices.clear();

    if (!root)
        return;

    size_t estimatedNodeCount = getNodeCount();
    gpuNodes.reserve(estimatedNodeCount);
    gpuSphereIndices.reserve(estimatedNodeCount * 2);

    convertNodeToGPU(root.get());

    // set grandparent
    gpuNodes[0].data.z = static_cast<uint32_t>(0);
    for (auto& currNode : gpuNodes) {
        currNode.data.w = gpuNodes[currNode.data.z].data.z;
    }

    for (auto& currNode : gpuNodes) {
        currNode.data.z = currNode.data.w;
        currNode.data.w = 0;
    }

    // check the num of leaf nodes

    std::vector<int> checkResults(10, 0);

    for (auto& currNode : gpuNodes) {
        if (currNode.aabbMin.w == -1) {
            checkResults[(int)currNode.aabbMax.w]++;
        }
    }

    gpuNodes.shrink_to_fit();
    gpuSphereIndices.shrink_to_fit();

    if (checkResults[1] == 64) {
        for (auto& currNode : gpuNodes) {
            if (currNode.aabbMin.w == -1) {
                currNode.data.w = gpuSphereIndices[currNode.data.x];
            }
        }
    }
}

// typical error cases
// size_t BVH::convertNodeToGPU(const BVHNodeCPU* node, std::vector<BVHNodeGPU>& gpuNodes) {
//     size_t nodeIndex = gpuNodes.size();
//     gpuNodes.emplace_back();

//     BVHNodeGPU& gpuNode = gpuNodes[nodeIndex];
//     gpuNode.aabbMin = glm::vec4(node->aabbMin, 0);
//     gpuNode.aabbMax = glm::vec4(node->aabbMax, 0);

//     if (node->isLeaf) {
//         gpuNode.aabbMin.w = -1;
//         gpuNode.aabbMax.w = static_cast<float>(node->sphereIndices.size());
//         gpuNode.data.x = static_cast<uint32_t>(gpuSphereIndices.size());

//         for (uint32_t sphereIdx : node->sphereIndices) {
//             gpuSphereIndices.push_back(sphereIdx);
//         }
//     } else {
//         size_t leftChild = convertNodeToGPU(node->leftChild.get(), gpuNodes);
//         size_t rightChild = convertNodeToGPU(node->rightChild.get(), gpuNodes);

//         gpuNode.aabbMin.w = static_cast<float>(leftChild);
//         gpuNode.aabbMax.w = static_cast<float>(rightChild);
//     }

//     return nodeIndex;
// }

size_t BVH::convertNodeToGPU(const BVHNodeCPU* node) {
    size_t nodeIndex = gpuNodes.size();
    gpuNodes.emplace_back();

    gpuNodes[nodeIndex].aabbMin = glm::vec4(node->aabbMin, 0);
    gpuNodes[nodeIndex].aabbMax = glm::vec4(node->aabbMax, 0);

    if (node->isLeaf) {
        gpuNodes[nodeIndex].aabbMin.w = -1;
        gpuNodes[nodeIndex].aabbMax.w = static_cast<float>(node->sphereIndices.size());
        gpuNodes[nodeIndex].data.y = static_cast<uint32_t>(gpuNodes[nodeIndex].aabbMax.w);
        gpuNodes[nodeIndex].data.x = static_cast<uint32_t>(gpuSphereIndices.size());

        for (uint32_t sphereIdx : node->sphereIndices) {
            gpuSphereIndices.push_back(sphereIdx);
        }
    } else {
        size_t leftChild = convertNodeToGPU(node->leftChild.get());
        size_t rightChild = convertNodeToGPU(node->rightChild.get());

        gpuNodes[nodeIndex].aabbMin.w = static_cast<float>(leftChild);
        gpuNodes[nodeIndex].aabbMax.w = static_cast<float>(rightChild);

        // set parent
        gpuNodes[leftChild].data.z = static_cast<uint32_t>(nodeIndex);
        gpuNodes[rightChild].data.z = static_cast<uint32_t>(nodeIndex);
    }

    return nodeIndex;
}

size_t BVH::getNodeCount() const {
    return gpuNodes.size();
}

size_t BVH::getMaxDepth() const {
    std::function<size_t(const BVHNodeCPU*, size_t)> getDepth = [&](const BVHNodeCPU* node,
                                                                    size_t depth) -> size_t {
        if (!node || node->isLeaf)
            return depth;
        return std::max(
            getDepth(node->leftChild.get(), depth + 1),
            getDepth(node->rightChild.get(), depth + 1));
    };
    return getDepth(root.get(), 1);
}

void BVH::printStats() const {
    spdlog::info("BVH Stats:");
    spdlog::info("  Nodes: {}", getNodeCount());
    spdlog::info("  Max Depth: {}", getMaxDepth());
    spdlog::info("  Sphere indices: {}", gpuSphereIndices.size());
}

std::unique_ptr<BVHNodeCPU> BVH::forceSplit(
    const std::vector<SphereDataPacked>& spheres, std::vector<uint32_t>& indices, int depth) {

    auto node = std::make_unique<BVHNodeCPU>();
    node->sphereIndices = indices;
    node->calculateAABB(spheres);

    if (indices.size() == 1) {
        node->isLeaf = true;
        return node;
    }

    glm::vec3 extent = node->aabbMax - node->aabbMin;
    int bestAxis = 0;
    if (extent.y > extent.x)
        bestAxis = 1;
    if (extent.z > extent[bestAxis])
        bestAxis = 2;

    std::sort(indices.begin(), indices.end(), [&](uint32_t a, uint32_t b) {
        glm::vec3 centerA = glm::vec3(spheres[a].centerAndRadius);
        glm::vec3 centerB = glm::vec3(spheres[b].centerAndRadius);
        return centerA[bestAxis] < centerB[bestAxis];
    });

    size_t splitIndex = indices.size() / 2;

    std::vector<uint32_t> leftIndices(indices.begin(), indices.begin() + splitIndex);
    std::vector<uint32_t> rightIndices(indices.begin() + splitIndex, indices.end());

    node->leftChild = buildRecursive(spheres, leftIndices, depth + 1);
    node->rightChild = buildRecursive(spheres, rightIndices, depth + 1);
    node->isLeaf = false;

    return node;
}


// Morton码实现
uint32_t SpatialSorter::expandBits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

uint64_t SpatialSorter::mortonEncode3D(uint32_t x, uint32_t y, uint32_t z) {
    uint64_t answer = 0;
    answer |= static_cast<uint64_t>(expandBits(z)) << 2;
    answer |= static_cast<uint64_t>(expandBits(y)) << 1;
    answer |= static_cast<uint64_t>(expandBits(x));
    return answer;
}

glm::vec3 SpatialSorter::calculateBounds(const std::vector<SphereData>& spheres,
                                       glm::vec3& minBounds, glm::vec3& maxBounds) {
    if (spheres.empty()) {
        minBounds = maxBounds = glm::vec3(0.0f);
        return glm::vec3(0.0f);
    }
    
    minBounds = glm::vec3(spheres[0].centerX, spheres[0].centerY, spheres[0].centerZ);
    maxBounds = minBounds;
    
    for (const auto& sphere : spheres) {
        glm::vec3 center(sphere.centerX, sphere.centerY, sphere.centerZ);
        minBounds = glm::min(minBounds, center);
        maxBounds = glm::max(maxBounds, center);
    }
    
    return maxBounds - minBounds;
}

glm::vec3 SpatialSorter::calculateBoundsPacked(const std::vector<SphereDataPacked>& spheres,
                                             glm::vec3& minBounds, glm::vec3& maxBounds) {
    if (spheres.empty()) {
        minBounds = maxBounds = glm::vec3(0.0f);
        return glm::vec3(0.0f);
    }
    
    minBounds = glm::vec3(spheres[0].centerAndRadius);
    maxBounds = minBounds;
    
    for (const auto& sphere : spheres) {
        glm::vec3 center = glm::vec3(sphere.centerAndRadius);
        minBounds = glm::min(minBounds, center);
        maxBounds = glm::max(maxBounds, center);
    }
    
    return maxBounds - minBounds;
}

void SpatialSorter::sortSpheres(std::vector<SphereData>& spheres) {
    if (spheres.size() <= 1) return;
    
    glm::vec3 minBounds, maxBounds;
    glm::vec3 extent = calculateBounds(spheres, minBounds, maxBounds);

    if (extent.x == 0.0f) extent.x = 1.0f;
    if (extent.y == 0.0f) extent.y = 1.0f;
    if (extent.z == 0.0f) extent.z = 1.0f;
    
    std::vector<std::pair<uint64_t, size_t>> mortonCodes;
    mortonCodes.reserve(spheres.size());
    
    constexpr uint32_t MORTON_MAX = (1u << 21) - 1;
    
    for (size_t i = 0; i < spheres.size(); ++i) {
        glm::vec3 center(spheres[i].centerX, spheres[i].centerY, spheres[i].centerZ);

        glm::vec3 normalized = (center - minBounds) / extent;
        normalized = glm::clamp(normalized, 0.0f, 1.0f);
        
        uint32_t x = static_cast<uint32_t>(normalized.x * MORTON_MAX);
        uint32_t y = static_cast<uint32_t>(normalized.y * MORTON_MAX);
        uint32_t z = static_cast<uint32_t>(normalized.z * MORTON_MAX);
        
        uint64_t morton = mortonEncode3D(x, y, z);
        mortonCodes.emplace_back(morton, i);
    }

    std::sort(mortonCodes.begin(), mortonCodes.end(),
              [](const auto& a, const auto& b) {
                  return a.first < b.first;
              });

    std::vector<SphereData> sortedSpheres;
    sortedSpheres.reserve(spheres.size());
    
    for (const auto& pair : mortonCodes) {
        sortedSpheres.push_back(spheres[pair.second]);
    }
    
    spheres = std::move(sortedSpheres);
    
    spdlog::info("Spheres sorted using Morton codes for spatial coherency");
}

void SpatialSorter::sortPackedSpheres(std::vector<SphereDataPacked>& packedSpheres) {
    if (packedSpheres.size() <= 1) return;
    
    glm::vec3 minBounds, maxBounds;
    glm::vec3 extent = calculateBoundsPacked(packedSpheres, minBounds, maxBounds);

    if (extent.x == 0.0f) extent.x = 1.0f;
    if (extent.y == 0.0f) extent.y = 1.0f;
    if (extent.z == 0.0f) extent.z = 1.0f;

    std::vector<std::pair<uint64_t, size_t>> mortonCodes;
    mortonCodes.reserve(packedSpheres.size());
    
    constexpr uint32_t MORTON_MAX = (1u << 21) - 1;
    
    for (size_t i = 0; i < packedSpheres.size(); ++i) {
        glm::vec3 center = glm::vec3(packedSpheres[i].centerAndRadius);

        glm::vec3 normalized = (center - minBounds) / extent;
        normalized = glm::clamp(normalized, 0.0f, 1.0f);
        
        uint32_t x = static_cast<uint32_t>(normalized.x * MORTON_MAX);
        uint32_t y = static_cast<uint32_t>(normalized.y * MORTON_MAX);
        uint32_t z = static_cast<uint32_t>(normalized.z * MORTON_MAX);
        
        uint64_t morton = mortonEncode3D(x, y, z);
        mortonCodes.emplace_back(morton, i);
    }

    std::sort(mortonCodes.begin(), mortonCodes.end(),
              [](const auto& a, const auto& b) {
                  return a.first < b.first;
              });

    std::vector<SphereDataPacked> sortedSpheres;
    sortedSpheres.reserve(packedSpheres.size());
    
    for (const auto& pair : mortonCodes) {
        sortedSpheres.push_back(packedSpheres[pair.second]);
    }
    
    packedSpheres = std::move(sortedSpheres);
    
    spdlog::info("Packed spheres sorted using Morton codes for spatial coherency");
}