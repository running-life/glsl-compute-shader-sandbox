#include "dbvh.h"
#include <cassert>
#include <iostream>
#include <functional>

void DBVH::build(const std::vector<SphereData>& spheres, std::vector<size_t>& indices) {
    if (indices.empty()) {
        root = nullptr;
        gpuNodes.clear();
        return;
    }

    root = buildRecursive(spheres, indices, 0, indices.size());
    convertToGPU();
}

std::unique_ptr<BVHNodeCPU> DBVH::buildRecursive(
    const std::vector<SphereData>& spheres, std::vector<size_t>& indices, size_t start,
    size_t end) {

    assert(start < end);

    // Calculate bounding box for current range
    AABB bounds = calculateBounds(spheres, indices, start, end);

    // Create leaf node if only one sphere
    if (end - start == 1) {
        auto node = std::make_unique<BVHNodeCPU>(indices[start], bounds);
        return node;
    }

    // Find best split using SAH
    SplitCandidate bestSplit = findBestSplit(spheres, indices, start, end, bounds);

    // Partition spheres based on best split
    auto partition =
        std::partition(indices.begin() + start, indices.begin() + end, [&](size_t index) {
            const SphereData& sphere = spheres[index];
            float centerPos;
            switch (bestSplit.axis) {
            case 0: centerPos = sphere.centerX; break;
            case 1: centerPos = sphere.centerY; break;
            case 2: centerPos = sphere.centerZ; break;
            default: centerPos = sphere.centerX; break;
            }
            return centerPos < bestSplit.position;
        });

    size_t mid = partition - indices.begin();

    // Ensure we don't create empty partitions
    if (mid == start)
        mid = start + 1;
    if (mid == end)
        mid = end - 1;

    // Create internal node
    auto node = std::make_unique<BVHNodeCPU>();
    node->bounds = bounds;
    node->isLeaf = false;

    // Recursively build children
    node->left = buildRecursive(spheres, indices, start, mid);
    node->right = buildRecursive(spheres, indices, mid, end);

    return node;
}

DBVH::SplitCandidate DBVH::findBestSplit(
    const std::vector<SphereData>& spheres, const std::vector<size_t>& indices, size_t start,
    size_t end, const AABB& bounds) {

    SplitCandidate bestSplit;
    bestSplit.cost = std::numeric_limits<float>::max();

    // Try splits along each axis
    for (int axis = 0; axis < 3; ++axis) {
        // Sample multiple split positions along the axis
        float minPos, maxPos;
        switch (axis) {
        case 0:
            minPos = bounds.minX;
            maxPos = bounds.maxX;
            break;
        case 1:
            minPos = bounds.minY;
            maxPos = bounds.maxY;
            break;
        case 2:
            minPos = bounds.minZ;
            maxPos = bounds.maxZ;
            break;
        }

        // Try several split positions
        const int numSamples = 16;
        for (int i = 1; i < numSamples; ++i) {
            float position = minPos + (maxPos - minPos) * static_cast<float>(i) / numSamples;
            float cost = calculateSAHCost(spheres, indices, start, end, axis, position, bounds);

            if (cost < bestSplit.cost) {
                bestSplit.axis = axis;
                bestSplit.position = position;
                bestSplit.cost = cost;
            }
        }
    }

    return bestSplit;
}

float DBVH::calculateSAHCost(
    const std::vector<SphereData>& spheres, const std::vector<size_t>& indices, size_t start,
    size_t end, int axis, float position, const AABB& bounds) {

    size_t leftCount = 0;
    size_t rightCount = 0;
    AABB leftBounds, rightBounds;

    // Count primitives and calculate bounds for each side
    for (size_t i = start; i < end; ++i) {
        const SphereData& sphere = spheres[indices[i]];
        float centerPos;
        switch (axis) {
        case 0: centerPos = sphere.centerX; break;
        case 1: centerPos = sphere.centerY; break;
        case 2: centerPos = sphere.centerZ; break;
        default: centerPos = sphere.centerX; break;
        }

        AABB sphereAABB = AABB::fromSphere(sphere);

        if (centerPos < position) {
            leftCount++;
            if (leftCount == 1) {
                leftBounds = sphereAABB;
            } else {
                leftBounds.expand(sphereAABB);
            }
        } else {
            rightCount++;
            if (rightCount == 1) {
                rightBounds = sphereAABB;
            } else {
                rightBounds.expand(sphereAABB);
            }
        }
    }

    // Avoid empty partitions
    if (leftCount == 0 || rightCount == 0) {
        return std::numeric_limits<float>::max();
    }

    // Calculate SAH cost
    float parentSA = bounds.surfaceArea();
    float leftSA = leftBounds.surfaceArea();
    float rightSA = rightBounds.surfaceArea();

    // SAH formula: SA_left/SA_parent * N_left + SA_right/SA_parent * N_right
    float cost = (leftSA / parentSA) * leftCount + (rightSA / parentSA) * rightCount;
    return cost;
}

AABB DBVH::calculateBounds(
    const std::vector<SphereData>& spheres, const std::vector<size_t>& indices, size_t start,
    size_t end) {

    assert(start < end);

    AABB bounds = AABB::fromSphere(spheres[indices[start]]);
    for (size_t i = start + 1; i < end; ++i) {
        AABB sphereAABB = AABB::fromSphere(spheres[indices[i]]);
        bounds.expand(sphereAABB);
    }

    return bounds;
}

void DBVH::insert(const std::vector<SphereData>& spheres, size_t index) {
    AABB newSphereBounds = AABB::fromSphere(spheres[index]);
    
    if (!root) {
        // Create root node if BVH is empty
        root = std::make_unique<BVHNodeCPU>(index, newSphereBounds);
    } else {
        // Find the best sibling node for the new sphere
        BVHNodeCPU* bestSibling = findBestSibling(root.get(), spheres, index);
        
        // Create new leaf for the inserted sphere
        auto newLeaf = std::make_unique<BVHNodeCPU>(index, newSphereBounds);
        
        // If best sibling is root, create new root
        if (bestSibling == root.get()) {
            auto newRoot = std::make_unique<BVHNodeCPU>();
            newRoot->isLeaf = false;
            newRoot->left = std::move(root);
            newRoot->right = std::move(newLeaf);
            newRoot->bounds = newRoot->left->bounds;
            newRoot->bounds.expand(newRoot->right->bounds);
            root = std::move(newRoot);
        } else {
            // Find parent of best sibling
            BVHNodeCPU* parent = nullptr;
            bool isLeftChild = false;
            
            // Search for parent (simplified - in practice might use parent pointers)
            std::function<bool(BVHNodeCPU*, BVHNodeCPU*)> findParent = 
                [&](BVHNodeCPU* node, BVHNodeCPU* target) -> bool {
                if (!node || node->isLeaf) return false;
                
                if (node->left.get() == target) {
                    parent = node;
                    isLeftChild = true;
                    return true;
                }
                if (node->right.get() == target) {
                    parent = node;
                    isLeftChild = false;
                    return true;
                }
                
                return findParent(node->left.get(), target) || 
                       findParent(node->right.get(), target);
            };
            
            findParent(root.get(), bestSibling);
            
            if (parent) {
                // Create new internal node to hold bestSibling and newLeaf
                auto newInternal = std::make_unique<BVHNodeCPU>();
                newInternal->isLeaf = false;
                newInternal->bounds = bestSibling->bounds;
                newInternal->bounds.expand(newSphereBounds);
                
                // Take ownership of bestSibling from parent
                if (isLeftChild) {
                    newInternal->left = std::move(parent->left);
                    newInternal->right = std::move(newLeaf);
                    parent->left = std::move(newInternal);
                } else {
                    newInternal->left = std::move(parent->right);
                    newInternal->right = std::move(newLeaf);
                    parent->right = std::move(newInternal);
                }
                
                // Update bounds up the tree
                updateBounds(root.get(), spheres);
            }
        }
    }
    convertToGPU();
}

BVHNodeCPU* DBVH::findBestSibling(
    BVHNodeCPU* root,
    const std::vector<SphereData>& spheres,
    size_t index) {
    
    AABB newSphereBounds = AABB::fromSphere(spheres[index]);
    BVHNodeCPU* bestNode = root;
    float bestCost = std::numeric_limits<float>::max();
    
    // Use a priority queue approach for efficient traversal
    std::vector<std::pair<BVHNodeCPU*, float>> candidates;
    candidates.push_back({root, 0.0f});
    
    while (!candidates.empty()) {
        auto [node, inheritedCost] = candidates.back();
        candidates.pop_back();
        
        // Calculate direct cost of making this node a sibling
        float directCost = calculateInsertionCost(node, newSphereBounds, spheres);
        float totalCost = directCost + inheritedCost;
        
        // Pruning: if total cost already exceeds best, skip this branch
        if (totalCost >= bestCost) {
            continue;
        }
        
        // Update best candidate
        if (totalCost < bestCost) {
            bestCost = totalCost;
            bestNode = node;
        }
        
        // If not a leaf, consider children
        if (!node->isLeaf) {
            // Calculate inherited cost for children (expansion cost of ancestors)
            AABB expandedBounds = node->bounds;
            expandedBounds.expand(newSphereBounds);
            float inheritedIncrease = calculateIndirectCost(node, node->bounds, expandedBounds);
            
            candidates.push_back({node->left.get(), inheritedCost + inheritedIncrease});
            candidates.push_back({node->right.get(), inheritedCost + inheritedIncrease});
        }
    }
    
    return bestNode;
}

// float DBVH::calculateInsertionCost(
//     BVHNodeCPU* node,
//     const AABB& newBounds,
//     const std::vector<SphereData>& spheres) {
    
//     // Direct cost: surface area of new parent node
//     AABB combinedBounds = node->bounds;
//     combinedBounds.expand(newBounds);
    
//     // Cost is proportional to surface area and number of primitives
//     int nodeLeafCount = countLeaves(node);
//     return combinedBounds.surfaceArea() * (nodeLeafCount + 1);
// }

// float DBVH::calculateIndirectCost(
//     BVHNodeCPU* node,
//     const AABB& oldBounds,
//     const AABB& newBounds) {
    
//     // Indirect cost: increase in surface area due to expansion
//     float oldSA = oldBounds.surfaceArea();
//     float newSA = newBounds.surfaceArea();
//     int leafCount = countLeaves(node);
    
//     return (newSA - oldSA) * leafCount;
// }

float DBVH::calculateInsertionCost(
    BVHNodeCPU* node,
    const AABB& newBounds,
    const std::vector<SphereData>& spheres) {
    
    // Direct cost: surface area of new parent node
    AABB combinedBounds = node->bounds;
    combinedBounds.expand(newBounds);
    
    return combinedBounds.surfaceArea();
}

float DBVH::calculateIndirectCost(
    BVHNodeCPU* node,
    const AABB& oldBounds,
    const AABB& newBounds) {
    
    // Indirect cost: increase in surface area due to expansion
    float oldSA = oldBounds.surfaceArea();
    float newSA = newBounds.surfaceArea();

    return (newSA - oldSA);
}



int DBVH::countLeaves(const BVHNodeCPU* node) {
    if (!node) return 0;
    if (node->isLeaf) return 1;
    return countLeaves(node->left.get()) + countLeaves(node->right.get());
}

std::unique_ptr<BVHNodeCPU> DBVH::insertRecursive(
    std::unique_ptr<BVHNodeCPU> node, const std::vector<SphereData>& spheres, size_t index) {
    // This method is kept for compatibility but not used in the new approach
    return node;
}

void DBVH::remove(const std::vector<SphereData>& spheres, size_t index) {
    if (!root)
        return;

    bool found = false;
    root = findAndRemove(std::move(root), index, found);

    if (found) {
        if (root) {
            updateBounds(root.get(), spheres);
        }
        convertToGPU();
    }
}

std::unique_ptr<BVHNodeCPU> DBVH::findAndRemove(
    std::unique_ptr<BVHNodeCPU> node, size_t index, bool& found) {

    if (!node)
        return nullptr;

    if (node->isLeaf) {
        if (node->sphereIndex == index) {
            found = true;
            return nullptr; // Remove this node
        }
        return node;
    }

    // Recursively search children
    node->left = findAndRemove(std::move(node->left), index, found);
    if (!found) {
        node->right = findAndRemove(std::move(node->right), index, found);
    }

    // If one child was removed, replace this node with the remaining child
    if (!node->left && !node->right) {
        return nullptr;
    } else if (!node->left) {
        return std::move(node->right);
    } else if (!node->right) {
        return std::move(node->left);
    }

    return node;
}

void DBVH::update(const std::vector<SphereData>& spheres, size_t index) {
    if (!root)
        return;

    // Check if the sphere still fits well in its current position
    if (sphereFitsWell(root.get(), spheres[index])) {
        // Just update bounds if sphere still fits reasonably well
        updateBounds(root.get(), spheres);
        convertToGPU();
    } else {
        // Remove and reinsert if sphere moved significantly
        remove(spheres, index);
        insert(spheres, index);
    }
}

bool DBVH::sphereFitsWell(const BVHNodeCPU* node, const SphereData& sphere) {
    if (!node)
        return false;

    AABB sphereBounds = AABB::fromSphere(sphere);

    // Check if sphere is still within reasonable bounds
    float tolerance = sphere.radius * 0.1f; // 10% tolerance

    return sphereBounds.minX >= node->bounds.minX - tolerance
           && sphereBounds.maxX <= node->bounds.maxX + tolerance
           && sphereBounds.minY >= node->bounds.minY - tolerance
           && sphereBounds.maxY <= node->bounds.maxY + tolerance
           && sphereBounds.minZ >= node->bounds.minZ - tolerance
           && sphereBounds.maxZ <= node->bounds.maxZ + tolerance;
}

void DBVH::updateBounds(BVHNodeCPU* node, const std::vector<SphereData>& spheres) {
    if (!node)
        return;

    if (node->isLeaf) {
        node->bounds = AABB::fromSphere(spheres[node->sphereIndex]);
        return;
    }

    // Recursively update children
    updateBounds(node->left.get(), spheres);
    updateBounds(node->right.get(), spheres);

    // Update this node's bounds to encompass children
    if (node->left && node->right) {
        node->bounds = node->left->bounds;
        node->bounds.expand(node->right->bounds);
    } else if (node->left) {
        node->bounds = node->left->bounds;
    } else if (node->right) {
        node->bounds = node->right->bounds;
    }
}

void DBVH::convertToGPU() {
    gpuNodes.clear();
    if (!root)
        return;

    convertNodeToGPU(root.get());
}

int DBVH::convertNodeToGPU(const BVHNodeCPU* node) {
    if (!node)
        return -1;

    int nodeIndex = static_cast<int>(gpuNodes.size());
    gpuNodes.emplace_back();

    BVHNodeGPU& gpuNode = gpuNodes[nodeIndex];
    gpuNode.minX = node->bounds.minX;
    gpuNode.minY = node->bounds.minY;
    gpuNode.minZ = node->bounds.minZ;
    gpuNode.maxX = node->bounds.maxX;
    gpuNode.maxY = node->bounds.maxY;
    gpuNode.maxZ = node->bounds.maxZ;

    if (node->isLeaf) {
        gpuNode.sphereIndex = static_cast<int>(node->sphereIndex);
        gpuNode.leftChild = -1;
        gpuNode.rightChild = -1;
    } else {
        gpuNode.sphereIndex = -1;
        gpuNode.leftChild = convertNodeToGPU(node->left.get());
        gpuNode.rightChild = convertNodeToGPU(node->right.get());
    }

    return nodeIndex;
}