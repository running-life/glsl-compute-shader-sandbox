针对ray marching过程中，SDF体的剔除，我总共尝试了两种方案。

# 一、spatial hash
主要思路就是将空间划分成多个子网格，预计算子网格和哪些SDF体相交，在ray marching过程中，查询步进点所在子网格的预计算结果。


它的主要实现放在两个文件中：
1. `build-spatial-hash.comp`：预计算，构建hash表
2. `ray-marching-hash.comp`：使用预计算的hash表做ray marching


但是，由于该方法的加速效果相较于基于BVH的方法，并不理想，所以，没有做进一步优化，现已弃用



# 二、BVH
基于BVH，有两种方案，在（二）和（三）中阐述。两种方法都依赖BVH，BVH构建方法如下：
## （一）BVH构建
由于参与ray marching的SDF体较少，所以，BVH放在CPU端构建，我尝试了static BVH和dynamic BVH，现均整合至dynamic BVH中，所有实现放在文件`dynamic.h`中。

由于我并不知道SDF云的数据结构是怎样的，所以，设计BVH时，让BVH只管理AABB包围盒，将其与用户数据解藕。BVH的叶子节点，会存储一个index，指向SDF体的index。

DBVH主要提供以下函数：
1. `int insert(const AABB& aabb)`：将aabb包围盒插入BVH中，并返回这个节点的index，用户需要自己维护这个节点到SDF云数据的映射（通过setDataIndex函数设置），该方法参考[DynamicBVH](https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf)实现。
2. `void remove(int index)`：删除index指向的叶子节点
3. `void update(int index, const AABB& aabb)`：更新index指向的叶子节点，更新后的包围盒为aabb，即使用aabb这个新包围盒替换旧包围盒
4. `void setDataIndex(int nodeIndex, int dataIndex)`：建立BVH中叶子节点到SDF云的链接，nodeIndex为叶子节点的index，dataIndex为SDF云的index
5. `std::vector<int> optimize()`：为了优化BVH遍历过程中的缓存命中率，我觉得需要在每次修改该BVH之后，均需要执行一次该函数（该函数开销较小，只有几微秒）。该函数返回的是optimize前的node index到optimize后的index的映射。
6. `std::vector<BVHNodeGPU> serializeForGPU()`：转化为GPU访问友好的格式，`BVHNodeGPU`数据结构的每个字段含义，在代码中有解释。这个数据结构可能需要修改，因为我为了让数据更紧凑，引入一些位运算，或许会出现一些不必要的开销。
7. `std::vector<int> rebuild(const std::vector<AABB>& aabbs)`：静态BVH构建，从一组AABB包围盒，重建BVH，返回值是每个传入的AABB包围盒对应的BVH中节点index。


## （二）基于BVH的自适应步长
实现是在`ray-marching-bvh-step.comp`中。
主要逻辑在函数`traverseBVH3`中，核心思路是：
1. 步进过程中，从步进点出发，沿光路发射光线，与BVH求交
2. 与包围盒相交情况分为两大类，一是该步进点不在包围盒中，则调整下一步的步进距离（快速跳过空旷区域）
3. 二是该步进点在包围盒中，则根据当前包围盒是否为叶子节点，做不同的处理，如果为叶子节点，则判断点是否在SDF体中，如果不是，则将它的左右孩子加入遍历栈


## （三）基于BVH查询光路
实现是在`ray-marching-bvh-once-opt.comp`中。
主要思路如下：
1. 对于一条光路，遍历一次BVH，求出所有与光路相交的SDF体或者BVH叶子节点（统称为候选节点），记录tEnter和tExit，见`collectCandidateSpheres`函数。
2. 将上一步求出的候选节点，依据tEnter，作插入排序，见`collectCandidateSpheres`函数。
3. 根据排序后的候选节点以及其tEnter和tExit信息，可以做快速剔除以及自适应步长，见`countSpheresAtPoint2`函数。

在计算步进点颜色的时候，会从步进点，朝光源发射光线。这根光线也可用前述方法处理，函数`countSphereOfSunRay`就是使用前述方法。不过，经测试，对于计算颜色的光线，求出与光路相交的候选节点之后，每一个步进点做暴力剔除更高效，该暴力剔除方法在`countSphereOfSunRay2`中实现。


# 三、附录
## (一)、目录结构

```
|-- externals：外部依赖库目录
|-- include：功能头文件目录
|-- ray-marching：主要实现目录
    |-- shaders：shader目录
    |   |-- build-spatial-hash.comp：预计算，构建hash表
    |   |-- cull-statistics.comp：统计culling信息（已弃用）
    |   |-- ray-marching-brute-force.comp：暴力法ray marching
    |   |-- ray-marching-bvh-once-opt.comp：基于BVH查询光路，并自适应步长
    |   |-- ray-marching-bvh-once.comp：基于BVH查询光路
    |   |-- ray-marching-bvh-step.comp：基于BVH的ray marching，并自适应步长
    |   |-- ray-marching-bvh.comp：基于BVH的ray marching
    |   |-- ray-marching-hash.comp：基于spatial hash的ray marching
    |   |-- render.frag：fragment shader
    |   |-- render.vert：vertex shader
    |   `-- time-consume.comp：耗时统计（已弃用）
    `-- src：代码目录
        |-- bvh.cpp：static BVH实现目录（已整合至dynamic BVH中）
        |-- bvh.h：static BVH头文件（已整合至dynamic BVH中）
        |-- dynamic.h：dynamic BVH实现
        |-- main.cpp：程序入口+imgui设置
        `-- renderer.h：pass控制代码
```