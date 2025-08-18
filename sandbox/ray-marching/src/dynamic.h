#pragma once

#include <algorithm>
#include <limits>
#include <numeric>
#include <stack>
#include <vector>

namespace dynamic_bvh {

typedef int Index;
constexpr Index NULL_INDEX = static_cast<Index>(-1);

struct AABB {
    float minX, minY, minZ;
    float maxX, maxY, maxZ;

    AABB()
        : minX(std::numeric_limits<float>::max()), minY(std::numeric_limits<float>::max()),
          minZ(std::numeric_limits<float>::max()), maxX(std::numeric_limits<float>::lowest()),
          maxY(std::numeric_limits<float>::lowest()), maxZ(std::numeric_limits<float>::lowest()) {}

    AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
        : minX(minX), minY(minY), minZ(minZ), maxX(maxX), maxY(maxY), maxZ(maxZ) {}

    // Calculate surface area for SAH
    float surfaceArea() const {
        float dx = maxX - minX;
        float dy = maxY - minY;
        float dz = maxZ - minZ;
        return 2.0f * (dx * dy + dx * dz + dy * dz);
    }

    // Expand to include another AABB
    void expand(const AABB& other) {
        minX = std::min(minX, other.minX);
        minY = std::min(minY, other.minY);
        minZ = std::min(minZ, other.minZ);
        maxX = std::max(maxX, other.maxX);
        maxY = std::max(maxY, other.maxY);
        maxZ = std::max(maxZ, other.maxZ);
    }

    bool contains(const AABB& other) const {
        return minX <= other.minX && minY <= other.minY && minZ <= other.minZ && maxX >= other.maxX
               && maxY >= other.maxY && maxZ >= other.maxZ;
    }
};

// CPU-side BVH node for construction and updates
struct BVHNodeCPU {
    AABB bounds;
    Index parent;
    Index left;
    Index right;
    Index dataIndex; // Valid only for leaf nodes
    bool isLeaf;

    BVHNodeCPU()
        : parent(NULL_INDEX), left(NULL_INDEX), right(NULL_INDEX), dataIndex(NULL_INDEX),
          isLeaf(false) {}

    void reset() {
        bounds = AABB();
        parent = left = right = NULL_INDEX;
        dataIndex = NULL_INDEX;
        isLeaf = false;
    }
};

struct BVHNodeGPU {
    float minX, minY, minZ;
    int leftRight; // packed: left in high 16 bits, right in low 16 bits
    float maxX, maxY, maxZ;
    int dataParent; // packed: data in high 16 bits, parent in low 16 bits
};

class DBVH {
public:
    DBVH() : m_rootIndex(-1), m_nodes(NODE_CAPACITY), m_freeNodes(NODE_CAPACITY) {
        std::iota(m_freeNodes.begin(), m_freeNodes.end(), 0);
        std::reverse(m_freeNodes.begin(), m_freeNodes.end());
    }

    int insert(const AABB& aabb) {
        Index leafIndex = allocateNode();

        if (m_rootIndex == NULL_INDEX) {
            m_rootIndex = leafIndex;
            m_nodes[leafIndex].parent = NULL_INDEX;
            m_nodes[leafIndex].bounds = aabb;
            m_nodes[leafIndex].isLeaf = true;
            return leafIndex;
        }

        if (leafIndex == NULL_INDEX) {
            return NULL_INDEX; // No space left
        }

        m_nodes[leafIndex].bounds = aabb;
        m_nodes[leafIndex].isLeaf = true;
        // leave dataIndex uninitialized

        // find the best sibling for the new node
        Index bestSibling = findBestSibling(aabb);
        BVHNodeCPU& siblingNode = m_nodes[bestSibling];

        Index oldParentIndex = siblingNode.parent;
        Index newParentIndex = allocateNode();

        if (newParentIndex == NULL_INDEX) {
            freeNode(leafIndex);
            return NULL_INDEX;
        }

        BVHNodeCPU& newParentNode = m_nodes[newParentIndex];
        newParentNode.parent = oldParentIndex;
        newParentNode.left = bestSibling;
        newParentNode.right = leafIndex;
        newParentNode.isLeaf = false;
        newParentNode.bounds = siblingNode.bounds;
        newParentNode.bounds.expand(m_nodes[leafIndex].bounds);

        m_nodes[leafIndex].parent = newParentIndex;
        siblingNode.parent = newParentIndex;

        // Update the root index
        if (oldParentIndex == NULL_INDEX) {
            m_rootIndex = newParentIndex;
        } else {
            m_nodes[oldParentIndex].left == bestSibling
                ? m_nodes[oldParentIndex].left = newParentIndex
                : m_nodes[oldParentIndex].right = newParentIndex;
        }

        refitAncestors(newParentIndex);

        return leafIndex;
    }

    void remove(int index) {
        if (index == NULL_INDEX) {
            return;
        }

        if (index == m_rootIndex) {
            m_rootIndex = NULL_INDEX;
            freeNode(index);
            return;
        }

        BVHNodeCPU& nodeToRemove = m_nodes[index];
        Index parentIndex = nodeToRemove.parent;
        BVHNodeCPU& parentNode = m_nodes[parentIndex];
        Index grandParentIndex = parentNode.parent;

        Index siblingIndex = (parentNode.left == index) ? parentNode.right : parentNode.left;
        if (grandParentIndex == NULL_INDEX) {
            m_rootIndex = siblingIndex;
            m_nodes[siblingIndex].parent = NULL_INDEX;
        } else {
            BVHNodeCPU& grandParentNode = m_nodes[grandParentIndex];
            if (grandParentNode.left == parentIndex) {
                grandParentNode.left = siblingIndex;
            } else {
                grandParentNode.right = siblingIndex;
            }
            m_nodes[siblingIndex].parent = grandParentIndex;
        }

        freeNode(index);
        freeNode(parentIndex);

        if (grandParentIndex != NULL_INDEX) {
            refitAncestors(grandParentIndex);
        }
    }

    void update(int index, const AABB& aabb) {
        if (index == NULL_INDEX) {
            return;
        }

        // so far, i've used tight aabb, and the moved object may not be inside the original aabb
        if (m_nodes[index].bounds.contains(aabb)) {
            return;
        }

        BVHNodeCPU& node = m_nodes[index];
        Index dataIndex = node.dataIndex;

        remove(index);
        Index newIndex = insert(aabb);

        if (newIndex != NULL_INDEX) {
            m_nodes[newIndex].dataIndex = dataIndex;
        }
    }

    void setDataIndex(int nodeIndex, int dataIndex) {
        m_nodes[nodeIndex].dataIndex = dataIndex;
    }

    /**
     * @brief Optimize BVH memory layout to improve GPU cache hit rate
     * * This operation rearranges the internal node array, invalidating any existing node indices
     * @return std::vector<Index> a mapping table from old index to new index
     */
    std::vector<int> optimize() {
        if (m_rootIndex == NULL_INDEX) {
            return {};
        }

        std::vector<OptimizeHelper> data(m_nodes.size());
        int counter = 0;
        generateOrderDfs(m_rootIndex, counter, data);

        std::sort(data.begin(), data.end(), [](const OptimizeHelper& a, const OptimizeHelper& b) {
            return a.order < b.order;
        });

        std::vector<Index> oldToNewLUT(m_nodes.size(), NULL_INDEX);
        std::vector<BVHNodeCPU> newNodes(m_nodes.size());

        int activeNodeCount = counter;

        for (int i = 0; i < activeNodeCount; ++i) {
            oldToNewLUT[data[i].oldIndex] = i;
        }

        for (int i = 0; i < activeNodeCount; ++i) {
            Index oldIndex = data[i].oldIndex;
            BVHNodeCPU& oldNode = m_nodes[oldIndex];

            newNodes[i] = oldNode;

            if (newNodes[i].parent != NULL_INDEX) {
                newNodes[i].parent = oldToNewLUT[newNodes[i].parent];
            }
            if (!newNodes[i].isLeaf) {
                newNodes[i].left = oldToNewLUT[newNodes[i].left];
                newNodes[i].right = oldToNewLUT[newNodes[i].right];
            }
        }

        m_nodes = std::move(newNodes);
        m_rootIndex = oldToNewLUT[m_rootIndex];

        m_freeNodes.clear();
        for (size_t i = activeNodeCount; i < m_nodes.size(); ++i) {
            m_freeNodes.push_back(static_cast<Index>(i));
        }
        std::reverse(m_freeNodes.begin(), m_freeNodes.end());

        return oldToNewLUT;
    }

    std::vector<BVHNodeGPU> serializeForGPU() const {
        std::vector<BVHNodeGPU> gpuNodes(m_nodes.size());

        for (size_t i = 0; i < m_nodes.size(); ++i) {
            const BVHNodeCPU& node = m_nodes[i];
            BVHNodeGPU& gpuNode = gpuNodes[i];

            gpuNode.minX = node.bounds.minX;
            gpuNode.minY = node.bounds.minY;
            gpuNode.minZ = node.bounds.minZ;
            gpuNode.maxX = node.bounds.maxX;
            gpuNode.maxY = node.bounds.maxY;
            gpuNode.maxZ = node.bounds.maxZ;

            // Pack indices to save space in the shader
            if (node.isLeaf) {
                gpuNode.leftRight =
                    (1 << 31) | ((node.left & 0xFFFF) << 16) | (node.right & 0xFFFF);
                gpuNode.dataParent = (node.dataIndex << 16) | (node.parent & 0xFFFF);
            } else {
                gpuNode.leftRight = ((node.left & 0xFFFF) << 16) | (node.right & 0xFFFF);
                gpuNode.dataParent = (0 << 16) | (node.parent & 0xFFFF);
            }
            // gpuNode.leftRight = (node.left << 16) | (node.right & 0xFFFF);
            // gpuNode.dataParent = (node.dataIndex << 16) | (node.parent & 0xFFFF);
        }
        return gpuNodes;
    }

    std::vector<int> rebuild(const std::vector<AABB>& aabbs) {
        // Reset the BVH structure
        m_rootIndex = NULL_INDEX;
        m_freeNodes.clear();
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            m_nodes[i].reset();
            m_freeNodes.push_back(static_cast<Index>(i));
        }
        std::reverse(m_freeNodes.begin(), m_freeNodes.end());

        // If no AABBs provided, return empty vector
        if (aabbs.empty()) {
            return {};
        }

        std::vector<Index> leafIndices(aabbs.size(), NULL_INDEX);

        // Build new tree using the provided AABBs
        m_rootIndex = buildRecursiveTopDown(aabbs, leafIndices);
        return leafIndices;
    }

private:
    // Helper method for top-down BVH construction
    Index buildRecursiveTopDown(
        const std::vector<AABB>& aabbs, std::vector<Index>& leafIndices, int depth = 0) {

        // Create indices array
        std::vector<Index> indices(aabbs.size());
        std::iota(indices.begin(), indices.end(), 0);

        return buildRecursiveTopDownImpl(indices, 0, aabbs.size(), aabbs, leafIndices, depth);
    }

    Index buildRecursiveTopDownImpl(
        std::vector<Index>& indices, size_t start, size_t end, const std::vector<AABB>& aabbs,
        std::vector<Index>& leafIndices, int depth) {

        // Allocate a new node
        Index nodeIndex = allocateNode();
        if (nodeIndex == NULL_INDEX)
            return NULL_INDEX;

        BVHNodeCPU& node = m_nodes[nodeIndex];

        // Calculate AABB for this node
        for (size_t idx = start; idx < end; ++idx) {
            node.bounds.expand(aabbs[indices[idx]]);
        }

        // If only one primitive, create leaf node
        if (end - start == 1) {
            node.isLeaf = true;
            node.dataIndex = indices[start];
            leafIndices[indices[start]] = nodeIndex;
            return nodeIndex;
        }

        // Use SAH to find best split
        SplitCandidate bestSplit = findBestSplitSAH(indices, start, end, aabbs, node.bounds);

        // If no good split found, use simple split
        if (!bestSplit.isValid() || end - start <= 4) {
            return buildSimpleSplit(indices, start, end, aabbs, leafIndices, depth, nodeIndex);
        }

        // Partition based on SAH split
        auto partitionPoint =
            std::partition(indices.begin() + start, indices.begin() + end, [&](Index idx) {
                const AABB& aabb = aabbs[idx];
                float center;
                switch (bestSplit.axis) {
                case 0: center = (aabb.minX + aabb.maxX) * 0.5f; break;
                case 1: center = (aabb.minY + aabb.maxY) * 0.5f; break;
                case 2: center = (aabb.minZ + aabb.maxZ) * 0.5f; break;
                default: center = 0.0f;
                }
                return center < bestSplit.position;
            });

        size_t splitIndex = partitionPoint - indices.begin();

        // Ensure we have valid splits
        if (splitIndex == start || splitIndex == end) {
            return buildSimpleSplit(indices, start, end, aabbs, leafIndices, depth, nodeIndex);
        }

        // Build children recursively
        Index leftChild =
            buildRecursiveTopDownImpl(indices, start, splitIndex, aabbs, leafIndices, depth + 1);
        Index rightChild =
            buildRecursiveTopDownImpl(indices, splitIndex, end, aabbs, leafIndices, depth + 1);

        // Connect children to parent
        node.isLeaf = false;
        node.left = leftChild;
        node.right = rightChild;

        if (leftChild != NULL_INDEX)
            m_nodes[leftChild].parent = nodeIndex;
        if (rightChild != NULL_INDEX)
            m_nodes[rightChild].parent = nodeIndex;

        return nodeIndex;
    }

    // SAH split evaluation structure
    struct SplitCandidate {
        int axis = -1;
        float position = 0.0f;
        float cost = std::numeric_limits<float>::max();
        size_t leftCount = 0;
        size_t rightCount = 0;

        bool isValid() const {
            return axis != -1 && cost < std::numeric_limits<float>::max();
        }
    };

    // SAH-based split finding
    SplitCandidate findBestSplitSAH(
        const std::vector<Index>& indices, size_t start, size_t end, const std::vector<AABB>& aabbs,
        const AABB& parentBounds) {
        static constexpr int SAH_BUCKETS = 12;

        SplitCandidate bestSplit;

        float parentSA = parentBounds.surfaceArea();
        if (parentSA <= 0.0f)
            return bestSplit;

        // Test each axis
        for (int axis = 0; axis < 3; ++axis) {
            float extent = 0.0f;
            float minVal = 0.0f, maxVal = 0.0f;

            switch (axis) {
            case 0:
                extent = parentBounds.maxX - parentBounds.minX;
                minVal = parentBounds.minX;
                maxVal = parentBounds.maxX;
                break;
            case 1:
                extent = parentBounds.maxY - parentBounds.minY;
                minVal = parentBounds.minY;
                maxVal = parentBounds.maxY;
                break;
            case 2:
                extent = parentBounds.maxZ - parentBounds.minZ;
                minVal = parentBounds.minZ;
                maxVal = parentBounds.maxZ;
                break;
            }

            if (extent < 1e-6f)
                continue;

            // Create buckets
            struct Bucket {
                size_t count = 0;
                AABB bounds;
            };

            Bucket buckets[SAH_BUCKETS] = {};

            // Distribute AABBs into buckets
            for (size_t idx = start; idx < end; ++idx) {
                const AABB& aabb = aabbs[indices[idx]];
                float center = 0.0f;
                switch (axis) {
                case 0: center = (aabb.minX + aabb.maxX) * 0.5f; break;
                case 1: center = (aabb.minY + aabb.maxY) * 0.5f; break;
                case 2: center = (aabb.minZ + aabb.maxZ) * 0.5f; break;
                }

                int bucketIdx = static_cast<int>(SAH_BUCKETS * (center - minVal) / extent);
                bucketIdx = std::clamp(bucketIdx, 0, SAH_BUCKETS - 1);

                buckets[bucketIdx].count++;
                buckets[bucketIdx].bounds.expand(aabb);
            }

            // Evaluate each split position
            for (int i = 0; i < SAH_BUCKETS - 1; ++i) {
                // Calculate left side
                size_t leftCount = 0;
                AABB leftBounds;
                for (int j = 0; j <= i; ++j) {
                    if (buckets[j].count > 0) {
                        leftCount += buckets[j].count;
                        leftBounds.expand(buckets[j].bounds);
                    }
                }

                // Calculate right side
                size_t rightCount = 0;
                AABB rightBounds;
                for (int j = i + 1; j < SAH_BUCKETS; ++j) {
                    if (buckets[j].count > 0) {
                        rightCount += buckets[j].count;
                        rightBounds.expand(buckets[j].bounds);
                    }
                }

                if (leftCount == 0 || rightCount == 0)
                    continue;

                // Calculate SAH cost
                float leftSA = leftBounds.surfaceArea();
                float rightSA = rightBounds.surfaceArea();
                float cost = (leftSA / parentSA) * leftCount + (rightSA / parentSA) * rightCount;

                if (cost < bestSplit.cost) {
                    bestSplit.cost = cost;
                    bestSplit.axis = axis;
                    bestSplit.position = minVal + extent * (float(i + 1) / SAH_BUCKETS);
                    bestSplit.leftCount = leftCount;
                    bestSplit.rightCount = rightCount;
                }
            }
        }

        return bestSplit;
    }

    // Fallback simple split method
    Index buildSimpleSplit(
        std::vector<Index>& indices, size_t start, size_t end, const std::vector<AABB>& aabbs,
        std::vector<Index>& leafIndices, int depth, Index nodeIndex) {
        BVHNodeCPU& node = m_nodes[nodeIndex];

        // Find axis with largest extent
        float extentX = node.bounds.maxX - node.bounds.minX;
        float extentY = node.bounds.maxY - node.bounds.minY;
        float extentZ = node.bounds.maxZ - node.bounds.minZ;

        int bestAxis = 0;
        if (extentY > extentX)
            bestAxis = 1;
        if (extentZ > std::max(extentX, extentY))
            bestAxis = 2;

        // Sort indices based on centers along the best axis
        auto centerOf = [&aabbs](Index idx, int axis) {
            const AABB& aabb = aabbs[idx];
            switch (axis) {
            case 0: return (aabb.minX + aabb.maxX) * 0.5f;
            case 1: return (aabb.minY + aabb.maxY) * 0.5f;
            case 2: return (aabb.minZ + aabb.maxZ) * 0.5f;
            default: return 0.0f;
            }
        };

        std::sort(indices.begin() + start, indices.begin() + end, [&](Index a, Index b) {
            return centerOf(a, bestAxis) < centerOf(b, bestAxis);
        });

        size_t median = start + (end - start) / 2;

        // Build children recursively
        Index leftChild =
            buildRecursiveTopDownImpl(indices, start, median, aabbs, leafIndices, depth + 1);
        Index rightChild =
            buildRecursiveTopDownImpl(indices, median, end, aabbs, leafIndices, depth + 1);

        // Connect children to parent
        node.isLeaf = false;
        node.left = leftChild;
        node.right = rightChild;

        if (leftChild != NULL_INDEX)
            m_nodes[leftChild].parent = nodeIndex;
        if (rightChild != NULL_INDEX)
            m_nodes[rightChild].parent = nodeIndex;

        return nodeIndex;
    }

    Index allocateNode() {
        if (m_freeNodes.empty()) {
            return NULL_INDEX;
        }
        Index index = m_freeNodes.back();
        m_freeNodes.pop_back();
        m_nodes[index].reset();
        return index;
    }

    void freeNode(Index index) {
        if (index != NULL_INDEX) {
            m_freeNodes.push_back(index);
        }
    }

    void refitAncestors(Index index) {
        Index currentIndex = index;
        while (currentIndex != NULL_INDEX) {
            BVHNodeCPU& node = m_nodes[currentIndex];
            if (!node.isLeaf) {
                const BVHNodeCPU& leftChild = m_nodes[node.left];
                const BVHNodeCPU& rightChild = m_nodes[node.right];

                node.bounds = leftChild.bounds;
                node.bounds.expand(rightChild.bounds);
            }
            currentIndex = node.parent;
        }
    }

    Index findBestSibling(const AABB& aabb) const {
        Index bestSibling = m_rootIndex;
        float bestCost = std::numeric_limits<float>::max();

        std::stack<std::pair<Index, float>> stack;
        stack.push({m_rootIndex, 0.0f});

        AABB mergedBounds;

        while (!stack.empty()) {
            auto [currentIndex, inheritedCost] = stack.top();
            stack.pop();

            const BVHNodeCPU& currentNode = m_nodes[currentIndex];

            mergedBounds = currentNode.bounds;
            mergedBounds.expand(aabb);
            float directCost = mergedBounds.surfaceArea();
            float totalCost = directCost + inheritedCost;

            if (totalCost >= bestCost) {
                continue;
            } else {
                bestCost = totalCost;
                bestSibling = currentIndex;
            }

            if (!currentNode.isLeaf) {
                float newInheritedCost =
                    inheritedCost + directCost - currentNode.bounds.surfaceArea();
                stack.push({currentNode.left, newInheritedCost});
                stack.push({currentNode.right, newInheritedCost});
            }
        }

        return bestSibling;
    }

    void collectAllLeafIndices(Index nodeIndex, std::vector<Index>& leafIndices) {
        if (nodeIndex == NULL_INDEX) {
            return;
        }

        const BVHNodeCPU& node = m_nodes[nodeIndex];
        if (node.isLeaf) {
            leafIndices.push_back(nodeIndex);
        } else {
            collectAllLeafIndices(node.left, leafIndices);
            collectAllLeafIndices(node.right, leafIndices);
        }
    }

    struct OptimizeHelper {
        Index oldIndex = NULL_INDEX;
        Index order = 1000; // a large value
    };

    void generateOrderDfs(Index nodeIndex, int& counter, std::vector<OptimizeHelper>& data) {
        if (nodeIndex == NULL_INDEX) {
            return;
        }

        data[nodeIndex].oldIndex = nodeIndex;
        data[nodeIndex].order = counter++;

        const BVHNodeCPU& node = m_nodes[nodeIndex];
        if (!node.isLeaf) {
            generateOrderDfs(node.left, counter, data);
            generateOrderDfs(node.right, counter, data);
        }
    }

private:
    Index m_rootIndex;
    std::vector<BVHNodeCPU> m_nodes;
    std::vector<Index> m_freeNodes;

public:
    static constexpr unsigned int NODE_CAPACITY = 128;
};

} // namespace dynamic_bvh