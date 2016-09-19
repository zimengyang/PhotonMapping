#pragma once
#include <la.h>
#include <raytracing/ray.h>
#include <raytracing/intersection.h>
#include <raytracing/intersectionengine.h>
#include <scene/scene.h>
#include <raytracing/kdtree.h>

class Scene;

//The Integrator class recursively evaluates the path a ray takes throughout a scene
//and computes the color a ray becomes as it bounces.
//It samples the materials, probability density functions, and BRDFs of the surfaces the ray hits
//to do this.

class Integrator
{
public:

    int Number_Light;
    int Number_BRDF;

    Integrator();
    virtual ~Integrator();
    Integrator(Scene *s);
    virtual glm::vec3 TraceRay(Ray r, unsigned int depth);
    void SetDepth(unsigned int depth);

    Scene* scene;
    IntersectionEngine* intersection_engine;
    unsigned int getMaxDepth(){return max_depth;}

    //random number generator and uniform distribution
    int seed;
    std::mt19937 generator;//(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> uniform_distribution;//(0.0f,1.0f);

    float PowerHeuristic(const float &pdf_s, const float &n_s, const float &pdf_f, const float &n_f);

    glm::vec3 MIS_SampleLight(Intersection&, Ray&, Geometry* &);
    glm::vec3 MIS_SampleBRDF(Intersection&, Ray&, Geometry *&);


    glm::vec3 MIS_SampleLight_Ld(Intersection&, Ray&, Geometry *&);
    glm::vec3 MIS_SampleBRDF_Ld(Intersection&, Ray&, Geometry *&, glm::vec3&); // return brdf sample direction

    glm::vec3 EstimateDirectLight(Intersection&, Ray&, Geometry* &,glm::vec3&);
    glm::vec3 EstimateDirectLight(Intersection&, Ray&, Geometry* &);

    //glm::vec3 EstimateIndirectLight(Intersection &, Ray &, glm::vec3 &,float &pdf);

    glm::vec3 EstimateLight(Geometry* &light, Ray r, unsigned int depth);

    bool RussianRoulette(const glm::vec3 &color,const int& depth);

    float throughput;

    void CoordinateSystem(const glm::vec3 &v1, glm::vec3 *v2, glm::vec3 *v3);

protected:
    unsigned int max_depth;//Default value is 5.
};

class DirectLightingIntegrator : public Integrator
{

public:
    DirectLightingIntegrator();
    virtual ~DirectLightingIntegrator();

    virtual glm::vec3 TraceRay(Ray r, unsigned int depth);
};

class SHLightingIntegrator : public Integrator
{
public:
    SHLightingIntegrator();
    virtual ~SHLightingIntegrator();

    virtual glm::vec3 TraceRay(Ray r, unsigned int depth);

    glm::vec3 SHCalculateDiffuseColor(glm::vec3 normal);

    glm::vec3 light_SHCoeffs[9];
};

