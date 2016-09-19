#include <raytracing/intersection.h>
#include <raytracing/intersectionengine.h>

Intersection::Intersection():
    point(glm::vec3(0)),
    normal(glm::vec3(0)),
    tangent(glm::vec3(0)),
    bitangent(glm::vec3(0)),
    t(-1),
    texture_color(glm::vec3(1.0f))
{
    object_hit = NULL;
}

IntersectionEngine::IntersectionEngine()
{
    scene = NULL;
    root = NULL;
}

Intersection IntersectionEngine::GetIntersection(Ray r)
{
    /// original getIntersection()
    /*
    Intersection nearest;
    for(Geometry* g : scene->objects)
    {
        Intersection isx = g->GetIntersection(r);
        if((isx.t < nearest.t && isx.object_hit != NULL) || nearest.t < 0)
        {
            nearest = isx;
        }
    }
    return nearest;
    */

    /// using bvg tree GetIntersection()
    Intersection inter = root->getIntersection(r);
    if(inter.t > 0)
    {
        return inter;
    }
    else
    {
        return Intersection();
    }
}

bool IntersectionComp(const Intersection &lhs, const Intersection &rhs)
{
    return lhs.t < rhs.t;
}

QList<Intersection> IntersectionEngine::GetAllIntersections(Ray r)
{
    QList<Intersection> result;
    for(Geometry* g : scene->objects)
    {
        Intersection isx = g->GetIntersection(r);
        if(isx.t > 0)
        {
            result.append(isx);
        }
    }
    std::sort(result.begin(), result.end(), IntersectionComp);
    return result;
}

glm::vec3 Intersection::ToLocalNormalCoordinate(const glm::vec3& w_world)
{
    glm::vec3 w_local;

    glm::mat4 M;
    M[0] = glm::vec4(tangent,0.0f);
    M[1] = glm::vec4(bitangent,0.0f);
    M[2] = glm::vec4(normal,0.0f);
    M[3] = glm::vec4(0,0,0,1);

    M = glm::transpose(M);

    w_local = glm::vec3(M * glm::vec4(w_world, 0.0f));

    return glm::normalize(w_local);
}

glm::vec3 Intersection::ToWorldNormalCoordinate(const glm::vec3& w_local)
{
    glm::vec3 w_world;

    glm::mat4 M;
    M[0] = glm::vec4(tangent,0.0f);
    M[1] = glm::vec4(bitangent,0.0f);
    M[2] = glm::vec4(normal,0.0f);
    M[3] = glm::vec4(0,0,0,1);

    //M = glm::transpose(M);

    w_world = glm::vec3(M * glm::vec4(w_local, 0.0f));

    return glm::normalize(w_world);
}

BVHNode::BVHNode()
{
    left = NULL;
    right = NULL;
    depth = 0;
    geometryAttached.clear();
}

BVHNode* BVHNode::buildBVHTree(QList<Geometry *> objects)
{
    BVHNode* root = new BVHNode();
    root->depth = 0;
    for(Geometry* g : objects)
    {
        root->geometryAttached.push_back(g);
        root->bBox.minBounding = BBox::mMinVector(g->bBox->minBounding, root->bBox.minBounding);
        root->bBox.maxBounding = BBox::mMaxVector(g->bBox->maxBounding, root->bBox.maxBounding);
    }
    root->createBVH(0);
    return root;

}

void BVHNode::createBVH(int dep)
{
    depth = dep;
    if(geometryAttached.size() == 1) // leaf node
    {
        bBox = BBox(*geometryAttached[0]->bBox);
        bBox.create();
        return;
    }
    else
    {
        int compareIndex = depth % 3;

        switch (compareIndex)
        {
        case 0:
            qSort(geometryAttached.begin(), geometryAttached.end(), BVHNode::compareNodeX);
            break;
        case 1:
            qSort(geometryAttached.begin(), geometryAttached.end(), BVHNode::compareNodeY);
            break;
        case 2:
            qSort(geometryAttached.begin(), geometryAttached.end(), BVHNode::compareNodeZ);
            break;
        }

        int middle = geometryAttached.size() / 2;
        if(middle != 0) //have left tree
        {
            left = new BVHNode();
            for(int i=0;i<middle;i++)
            {
                Geometry* tmp = geometryAttached[i];
                left->geometryAttached.push_back(tmp);
                left->bBox.minBounding = BBox::mMinVector(left->bBox.minBounding, tmp->bBox->minBounding);
                left->bBox.maxBounding = BBox::mMaxVector(left->bBox.maxBounding, tmp->bBox->maxBounding);
            }
            left->createBVH(dep + 1);
        }

        right = new BVHNode();
        for(int i=middle;i<geometryAttached.size();i++)
        {
            Geometry* tmp = geometryAttached[i];
            right->geometryAttached.push_back(tmp);
            right->bBox.minBounding = BBox::mMinVector(right->bBox.minBounding, tmp->bBox->minBounding);
            right->bBox.maxBounding = BBox::mMaxVector(right->bBox.maxBounding, tmp->bBox->maxBounding);
        }
        right->createBVH(dep + 1);
    }

    bBox = BBox(bBox.minBounding, bBox.maxBounding);
    bBox.create();
}

void BVHNode::releaseTree(BVHNode *root)
{
    if(root == NULL)
        return;

    if(root->left != NULL)
        releaseTree(root->left);
    if(root->right != NULL)
        releaseTree(root->right);

    //std::cout<<"release BVH node: depth = "<<root->depth<<", objects = "<<root->geometryAttached.size()<<std::endl;
    delete root;
    root = NULL;
}

Intersection BVHNode::getIntersection(const Ray &r)
{
    if(geometryAttached.size() < 1)
        return Intersection();

    Intersection intersection = bBox.getIntersection(r);
    if(intersection.t <= 0)
        return Intersection();

    if(left == NULL && right == NULL) // leaf
    {
        Geometry* g = geometryAttached[0];
        //Ray transformedRay(r.GetTransformedCopy(g->transform.invT()));
        //Intersection inter = g->GetIntersection(transformedRay);
        Intersection inter = g->GetIntersection(r);
        if(inter.t > 0)
        {
            return inter;
        }
        else
            return Intersection();
    }

    Intersection leftInter, rightInter;
    if(left != NULL)
        leftInter = left->getIntersection(r);
    if(right != NULL)
        rightInter = right->getIntersection(r);

    if(leftInter.t > 0 && rightInter.t > 0)
    {
        if(leftInter.t < rightInter.t)
            return leftInter;
        else
            return rightInter;
    }

    if(leftInter.t <= 0 && rightInter.t > 0)
        return rightInter;

    if(leftInter.t > 0 && rightInter.t <= 0)
        return leftInter;

    if(leftInter.t <=0 && rightInter.t <= 0)
        return Intersection();
}


bool BVHNode::compareNodeX(Geometry *&a, Geometry *&b)
{
    return a->bBox->maxBounding[0] < b->bBox->minBounding[0];
}

bool BVHNode::compareNodeY(Geometry *&a, Geometry *&b)
{
    return a->bBox->maxBounding[1] < b->bBox->minBounding[1];
}
bool BVHNode::compareNodeZ(Geometry *&a, Geometry *&b)
{
    return a->bBox->maxBounding[2] < b->bBox->minBounding[2];
}
