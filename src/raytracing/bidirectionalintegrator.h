#ifndef BIDIRECTIONALINTEGRATOR
#define BIDIRECTIONALINTEGRATOR

#include <la.h>
#include <raytracing/integrator.h>

struct PathNode
{
    Intersection isx;
    glm::vec3 dirIn_world,dirOut_world;
    glm::vec3 dirIn_local,dirOut_local;
    glm::vec3 F;//base color and texture color all included here
    float pdf;
};

class BidirectionalIntegrator: public Integrator
{
public:
    BidirectionalIntegrator();
    virtual ~BidirectionalIntegrator();

    virtual glm::vec3 TraceRay(Ray r, unsigned int depth);
    std::vector<PathNode> generateLightPath(Geometry* &light);
    std::vector<PathNode> generateEyePath(Ray r);

    glm::vec3 EvaluatePath(std::vector<PathNode>& eyePath,int nEye,std::vector<PathNode>& lightPath,int nLight);

    float G(PathNode& a,PathNode& b);

    float WeightPath(int eyeL,int lightL) {return float(eyeL+lightL);}
    bool isBlocked(const Intersection& isx1,const Intersection & isx2);

    glm::vec3 EstimateDirectLight(Intersection& isx,Ray& r,Geometry*& light);
    glm::vec3 MIS_SampleLight(Intersection&, Ray&, Geometry* &);
    glm::vec3 MIS_SampleBRDF(Intersection&, Ray&, Geometry *&);

    float PowerHeuristic(const float &pdf_s, const float &n_s, const float &pdf_f, const float &n_f);

};

#endif // BIDIRECTIONALINTEGRATOR

