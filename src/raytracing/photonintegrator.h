#ifndef PHOTONINTEGRATOR
#define PHOTONINTEGRATOR

#include <la.h>
#include <raytracing/integrator.h>
#include <QMutex>
#include <QThread>


struct Photon;
struct ClosePhoton;

struct RadiancePhoton;

// look up process
struct PhotonProcess;
struct RadiancePhotonProcess;


class PhotonIntegrator: public Integrator
{
public:
    PhotonIntegrator();
    virtual ~PhotonIntegrator();

    virtual glm::vec3 TraceRay(Ray r, unsigned int depth);

    void Preprocessing();

    friend class PhotonShootingTask;

    // helper function for waiting for all threads
    void WaitForAllThreads(QThread **threads, int nTasks);

    glm::vec3 SpecularTransmit(Ray r, int depth, Intersection& isx);
    glm::vec3 SpecularReflect(Ray r, int depth, Intersection& isx);

    // data
    uint32_t nCausticPhotonsWanted, nIndirectPhotonsWanted, nLookup;
    float maxDistSquared;
    int maxSpecularDepth, maxPhotonDepth;
    bool finalGather;
    int gatherSamples;
    float cosGatherAngle;

    int nCausticPaths, nIndirectPaths;
    KdTree<Photon> *causticMap;
    KdTree<Photon> *indirectMap;
    KdTree<RadiancePhoton> *radianceMap;
};

class PhotonShootingTask : public QThread
{
public:

    PhotonShootingTask(
            int _taskNum,float _time,PhotonIntegrator* _integrator,
            bool &_abortTasks, int &_nDirectPaths,
            std::vector<Photon> &_direct, std::vector<Photon> &_indirect, std::vector<Photon> &_caustic,
            std::vector<RadiancePhoton> &_radiance,
            uint32_t &_nshot, const Scene* _scene,
            std::vector<glm::vec3> &_rpReflectances, std::vector<glm::vec3> &_rpTransmittances):
        taskNum(_taskNum), time(_time), integrator(_integrator),
        abortTasks(_abortTasks), nDirectPaths(_nDirectPaths),
        directPhotons(_direct),indirectPhotons(_indirect),causticPhotons(_caustic),
        radiancePhotons(_radiance),
        nshot(_nshot), scene(_scene),
        rpReflectances(_rpReflectances),rpTransmittances(_rpTransmittances)
    {}
    
    int taskNum;
    float time;
    
    static QMutex mutex;
    PhotonIntegrator *integrator;
    
    bool &abortTasks;
    int &nDirectPaths;
    
    std::vector<Photon> &directPhotons, &indirectPhotons, &causticPhotons;
    std::vector<RadiancePhoton> &radiancePhotons;
    std::vector<glm::vec3> &rpReflectances, &rpTransmittances;

    uint32_t &nshot;
    
    const Scene *scene;
    
protected:
    virtual void run();
      
};

class ComputeRadianceTask : public QThread
{
public:

    ComputeRadianceTask(
        uint32_t _taskNum, uint32_t _numTasks,
        std::vector<RadiancePhoton> &rps,
        const std::vector<glm::vec3> &rhor,
        const std::vector<glm::vec3> &rhot,
        uint32_t nlookup, float md2,
        int ndirect, KdTree<Photon> *direct,
        int nindirect, KdTree<Photon> *indirect,
        int ncaus, KdTree<Photon> *caustic):
        taskNum(_taskNum), numTasks(_numTasks),
        radiancePhotons(rps),
        rpReflectances(rhor), rpTransmittances(rhot),
        nLookup(nlookup), maxDistSquared(md2),
        nDirectPaths(ndirect), nIndirectPaths(nindirect), nCausticPaths(ncaus),
        directMap(direct), indirectMap(indirect), causticMap(caustic) { }

private:

    uint32_t taskNum, numTasks;
    std::vector<RadiancePhoton> &radiancePhotons;
    const std::vector<glm::vec3> &rpReflectances, &rpTransmittances;
    uint32_t nLookup;
    float maxDistSquared;
    int nDirectPaths, nIndirectPaths, nCausticPaths;
    KdTree<Photon> *directMap, *indirectMap, *causticMap;


    virtual void run();
};


#endif // PHOTONINTEGRATOR

