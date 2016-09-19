#include "scene/geometry/bbox.h"
#include "raytracing/intersection.h"
#include "raytracing/ray.h"

#include <iostream>

BBox::BBox():
    minBounding(glm::vec3(INFINITY, INFINITY, INFINITY)),
    maxBounding(glm::vec3(-INFINITY, -INFINITY, -INFINITY))
{
    // default by rendering in green box
    boxColor = glm::vec3(0,1,0);
}

BBox::BBox(const glm::vec3 &minB, const glm::vec3 &maxB)
{
     minBounding = minB;
     maxBounding = maxB;

     // -1 -1 -1  #0
     bBoxPos.push_back(glm::vec3(minBounding.x,minBounding.y,minBounding.z));
     // -1 1 -1   #1
     bBoxPos.push_back(glm::vec3(minBounding.x,maxBounding.y,minBounding.z));
     // 1 1 -1    #2
     bBoxPos.push_back(glm::vec3(maxBounding.x,maxBounding.y,minBounding.z));
     // 1 -1 -1   #3
     bBoxPos.push_back(glm::vec3(maxBounding.x,minBounding.y,minBounding.z));

     // -1 -1 1  #4
     bBoxPos.push_back(glm::vec3(minBounding.x,minBounding.y,maxBounding.z));
     // -1 1 1   #5
     bBoxPos.push_back(glm::vec3(minBounding.x,maxBounding.y,maxBounding.z));
     // 1 1 1    #6
     bBoxPos.push_back(glm::vec3(maxBounding.x,maxBounding.y,maxBounding.z));
     // 1 -1 1   #7
     bBoxPos.push_back(glm::vec3(maxBounding.x,minBounding.y,maxBounding.z));
}

BBox::BBox(const BBox & bb)
{
    minBounding = bb.minBounding;
    maxBounding = bb.maxBounding;
    bBoxPos = bb.bBoxPos;
}

glm::vec3 BBox::getCenterpoint()
{
    return glm::vec3((minBounding + maxBounding) / 2.0f);
}

void BBox::setMaxBounding(glm::vec3 &mb)
{
    maxBounding = mb;
}

void BBox::setMinBounding(glm::vec3 &mb)
{
    minBounding = mb;
}

glm::vec3 BBox::getMinBounding() const
{
    return minBounding;
}

glm::vec3 BBox::getMaxBoudning() const
{
    return maxBounding;
}

void BBox::reComputeBoundingBox(const glm::mat4 &T)
{
    minBounding=glm::vec3(INFINITY);
    maxBounding=glm::vec3(-INFINITY);

    for(int i=0; i<bBoxPos.size();i++)
    {
        glm::vec4 transformedP = T * glm::vec4(bBoxPos[i],1.0f);
        bBoxPos[i] = glm::vec3(transformedP / transformedP.w);
        minBounding = mMinVector(minBounding, bBoxPos[i]);
        maxBounding = mMaxVector(maxBounding, bBoxPos[i]);
    }
}

glm::vec3 BBox::mMinVector(glm::vec3 &a, glm::vec3 &b)
{
    return glm::vec3(std::min(a.x,b.x), std::min(a.y,b.y), std::min(a.z,b.z));
}

glm::vec3 BBox::mMaxVector(glm::vec3 &a, glm::vec3 &b)
{
    return glm::vec3(std::max(a.x,b.x), std::max(a.y,b.y), std::max(a.z,b.z));
}

glm::vec3 BBox::mMinVector(glm::vec3 &a, glm::vec3 &b, glm::vec3 &c)
{
    glm::vec3 minab = mMinVector(a,b);
    glm::vec3 minbc = mMinVector(b,c);
    return mMinVector(minab, minbc);
}

glm::vec3 BBox::mMaxVector(glm::vec3 &a, glm::vec3 &b, glm::vec3 &c)
{
    glm::vec3 maxab = mMaxVector(a,b);
    glm::vec3 maxbc = mMaxVector(b,c);
    return mMaxVector(maxab, maxbc);
}

//Methods inherited from Drawable
void BBox::create()
{
    std::vector<glm::vec3> pos;
    std::vector<glm::vec3> col;
    std::vector<GLuint> idx;

    for(glm::vec3 p: bBoxPos)
        pos.push_back(p);

    for(int i=0;i<8;i++)
        col.push_back(glm::vec3(0,1,0));

    //back face
    idx.push_back(0);idx.push_back(1);
    idx.push_back(1);idx.push_back(2);
    idx.push_back(2);idx.push_back(3);
    idx.push_back(0);idx.push_back(3);
    //front face
    idx.push_back(4);idx.push_back(5);
    idx.push_back(5);idx.push_back(6);
    idx.push_back(6);idx.push_back(7);
    idx.push_back(4);idx.push_back(7);
    //between
    idx.push_back(0);idx.push_back(4);
    idx.push_back(1);idx.push_back(5);
    idx.push_back(2);idx.push_back(6);
    idx.push_back(3);idx.push_back(7);

    count = idx.size();

    bufIdx.create();
    bufIdx.bind();
    bufIdx.setUsagePattern(QOpenGLBuffer::StaticDraw);
    bufIdx.allocate(idx.data(), count * sizeof(GLuint));

    bufPos.create();
    bufPos.bind();
    bufPos.setUsagePattern(QOpenGLBuffer::StaticDraw);
    bufPos.allocate(pos.data(), pos.size() * sizeof(glm::vec3));

    bufCol.create();
    bufCol.bind();
    bufCol.setUsagePattern(QOpenGLBuffer::StaticDraw);
    bufCol.allocate(col.data(), col.size() * sizeof(glm::vec3));

}

GLenum BBox::drawMode()
{
    return GL_LINES;
}

Intersection BBox::getIntersection(const Ray &r)
{
    Intersection intersection;
    if(r.origin.x <= maxBounding.x && r.origin.x >= minBounding.x &&
       r.origin.y <= maxBounding.y && r.origin.y >= minBounding.y &&
       r.origin.z <= maxBounding.z && r.origin.z >= minBounding.z)
    {
        intersection.t = 1.0f; //whatever t is , it can hit the box if the ray is inside the bounding box
        return intersection;
    }


    // define xyz_min and xyz_max as -0.5 and 0.5
    float t_near = -INFINITY,t_far = INFINITY;

    bool miss = false;

    // process x, y and z components separately
    for (int i=0; i<3; i++)
    {
        if(fequal(r.direction[i], 0.0f)) // parallel
        {
            if(r.origin[i] < minBounding[i] || r.origin[i] > maxBounding[i])
            {
                miss = true;
                break;
            }
        }
        else  // not parallel
        {
            float t0 = (minBounding[i] - r.origin[i]) / r.direction[i];
            float t1 = (maxBounding[i] - r.origin[i]) / r.direction[i];

            if(t0 > t1) {float temp=t0; t0=t1; t1=temp;}

            if(t0 > t_near) // update t_near
                t_near = t0;

            if(t1 < t_far) // update t_far
                t_far = t1;
        }
    }

    if(t_near > t_far)
        miss = true;

    if(!miss)
    {
        intersection.t = t_near;
        //intersection.object_hit = this;
        intersection.point = r.origin + t_near*r.direction;
    }

    return intersection;
}
