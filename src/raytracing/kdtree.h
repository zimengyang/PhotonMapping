#ifndef KDTREE
#define KDTREE
#include <stdint.h>

struct KdNode
{
    float splitPos;
    uint32_t splitAxis;
    uint32_t hasLeftChild,rightChild;

    void init(float p, uint32_t a)
    {
        splitPos = p;
        splitAxis = a;
        rightChild = (1<<29)-1;
        hasLeftChild = 0;
    }

    void initLeaf()
    {
        splitAxis = 3;
        rightChild = (1<<29)-1;
        hasLeftChild = 0;
    }

};


struct Bound
{
    glm::vec3 minPos;
    glm::vec3 maxPos;

    Bound()
    {
        minPos = glm::vec3(INT32_MAX,INT32_MAX,INT32_MAX);
        maxPos = glm::vec3(INT32_MIN,INT32_MIN,INT32_MIN);
    }

    int MaximumExtent()
    {

        float maxExt = INT32_MIN;
        float index;
        for(int i=0;i<3;i++)
        {
            float tmp = fabs(minPos[i]-maxPos[i]);
            if(tmp > maxExt)
            {
                maxExt = tmp;
                index = i;
            }
        }
        return index;
    }

    void Union(const glm::vec3 &p)
    {
        minPos.x = fmin(minPos.x,p.x);
        minPos.y = fmin(minPos.y,p.y);
        minPos.z = fmin(minPos.z,p.z);

        maxPos.x = fmax(maxPos.x,p.x);
        maxPos.y = fmax(maxPos.y,p.y);
        maxPos.z = fmax(maxPos.z,p.z);
    }
};

template <typename NodeData>
class KdTree
{
public:
    KdTree(const std::vector<NodeData> &data);
    ~KdTree()
    {
        if(nodes) delete nodes;
        if(nodeData) delete nodeData;
    }

    template <typename LookupProc>
    void Lookup(const glm::vec3 &p, LookupProc &process, float &maxDistSquared) const;


    void printTreeInformation() const
    {
        for(int i=0;i<nNodes;++i)
        {
            std::cout<<"hasLeftChild:"<<nodes[i].hasLeftChild<<", "
                     <<"rightChild:"<<nodes[i].rightChild<<", "
                     <<"splitAxis:"<<nodes[i].splitAxis<<", "
                     <<"splitPos:"<<nodes[i].splitPos<<std::endl;
        }
    }
private:
    KdNode* nodes;
    NodeData* nodeData;
    uint32_t nNodes, nextFreeNode;

    void recursiveBuild(uint32_t nodeNum, int start,int end,const NodeData **buildNodes);

    template<typename LookupProc>
    void privateLookup(uint32_t nodeNum, const glm::vec3 &p, LookupProc &process, float &maxDistSquared) const;

    float DistanceSquared(const glm::vec3 &a, const glm::vec3 &b) const
    {
        float dis = glm::distance(a,b);
        return dis*dis;
    }
};

template <typename NodeData>
struct CompareNode {
    CompareNode(int a) { axis = a; }
    int axis;
    bool operator()(const NodeData *d1, const NodeData *d2) const {
        return d1->p[axis] == d2->p[axis] ? (d1 < d2) :
                                            d1->p[axis] < d2->p[axis];
    }
};

// KdTree Method Definitions
template <typename NodeData>
KdTree<NodeData>::KdTree(const std::vector<NodeData> &d) {
    nNodes = d.size();
    nextFreeNode = 1;
    //nodes = AllocAligned<KdNode>(nNodes);
    //nodeData = AllocAligned<NodeData>(nNodes);
    nodes = new KdNode[nNodes];
    nodeData = new NodeData[nNodes];

    std::vector<const NodeData *> buildNodes(nNodes, NULL);
    for (uint32_t i = 0; i < nNodes; ++i)
        buildNodes[i] = &d[i];
    // Begin the KdTree building process
    recursiveBuild(0, 0, nNodes, &buildNodes[0]);
}

template <typename NodeData>
void KdTree<NodeData>::recursiveBuild(uint32_t nodeNum, int start, int end,
        const NodeData **buildNodes) {
    // Create leaf node of kd-tree if we've reached the bottom
    if (start + 1 == end) {
        nodes[nodeNum].initLeaf();
        nodeData[nodeNum] = *buildNodes[start];
        return;
    }

    // Choose split direction and partition data

    // Compute bounds of data from _start_ to _end_
    Bound bound;
    for (int i = start; i < end; ++i)
        bound.Union(buildNodes[i]->p);
        //bound = Union(bound, buildNodes[i]->p);
    int splitAxis = bound.MaximumExtent();
    int splitPos = (start+end)/2;
    std::nth_element(&buildNodes[start], &buildNodes[splitPos],
                     &buildNodes[end], CompareNode<NodeData>(splitAxis));

    // Allocate kd-tree node and continue recursively
    nodes[nodeNum].init(buildNodes[splitPos]->p[splitAxis], splitAxis);
    nodeData[nodeNum] = *buildNodes[splitPos];
    if (start < splitPos) {
        nodes[nodeNum].hasLeftChild = 1;
        uint32_t childNum = nextFreeNode++;
        recursiveBuild(childNum, start, splitPos, buildNodes);
    }
    if (splitPos+1 < end) {
        nodes[nodeNum].rightChild = nextFreeNode++;
        recursiveBuild(nodes[nodeNum].rightChild, splitPos+1,
                       end, buildNodes);
    }
}


template <typename NodeData> template <typename LookupProc>
void KdTree<NodeData>::Lookup(const glm::vec3 &p, LookupProc &proc,
                              float &maxDistSquared) const {
    privateLookup(0, p, proc, maxDistSquared);
}


template <typename NodeData> template <typename LookupProc>
void KdTree<NodeData>::privateLookup(uint32_t nodeNum, const glm::vec3 &p,
        LookupProc &process, float &maxDistSquared) const {
    KdNode *node = &nodes[nodeNum];
    // Process kd-tree node's children
    int axis = node->splitAxis;
    if (axis != 3) {
        float dist2 = (p[axis] - node->splitPos) * (p[axis] - node->splitPos);
        if (p[axis] <= node->splitPos) {
            if (node->hasLeftChild)
                privateLookup(nodeNum+1, p, process, maxDistSquared);
            if (dist2 < maxDistSquared && node->rightChild < nNodes)
                privateLookup(node->rightChild, p, process, maxDistSquared);
        }
        else {
            if (node->rightChild < nNodes)
                privateLookup(node->rightChild, p, process, maxDistSquared);
            if (dist2 < maxDistSquared && node->hasLeftChild)
                privateLookup(nodeNum+1, p, process, maxDistSquared);
        }
    }

    // Hand kd-tree node to processing function
    float dist2 = DistanceSquared(nodeData[nodeNum].p, p);
    if (dist2 < maxDistSquared)
        process(p, nodeData[nodeNum], dist2, maxDistSquared);
}


#endif // KDTREE

