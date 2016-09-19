#pragma once

#include <scene/geometry/geometry.h>

//A sphere is assumed to have a radius of 0.5 and a center of <0,0,0>.
//These attributes can be altered by applying a transformation matrix to the sphere.
class Sphere : public Geometry
{
public:
    Intersection GetIntersection(Ray r);
    virtual glm::vec2 GetUVCoordinates(const glm::vec3 &point);
    virtual glm::vec3 ComputeNormal(const glm::vec3 &P);
    void create();

    virtual void ComputeArea();
    virtual Intersection SampleOnGeometrySurface(const float &u, const float &v, const glm::vec3 &point);
    virtual Intersection RandomSampleOnSurface(const float &u, const float &v);

    Intersection GetSurfaceSample(const float &u1,const float &u2,const glm::vec3&);

    virtual void SetNormalTangentBitangent(const glm::vec3& point_local, Intersection& isx);

    virtual void setBBox();
    virtual float RayPDF(const Intersection &isx, const Ray &ray);

    virtual glm::vec3 Sample_L(const Scene *scene, float u[], Ray &ray, glm::vec3 &normal, float &pdf);
};
