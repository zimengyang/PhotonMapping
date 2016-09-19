#include <raytracing/photonintegrator.h>
#include <raytracing/samplers/samplercommon.h>

QMutex PhotonShootingTask::mutex;


struct Photon {
    Photon(const glm::vec3 &pp, const glm::vec3 &wt, const glm::vec3 &w)
        : p(pp), alpha(wt), wi(w) { }
    Photon() { }

    glm::vec3 p;
    glm::vec3 alpha;
    glm::vec3 wi;
};

struct RadiancePhoton {
    RadiancePhoton(const glm::vec3 &pp, const glm::vec3 &nn)
        : p(pp), n(nn), Lo(0.f) { }
    RadiancePhoton() { }

    glm::vec3 p;
    glm::vec3 n;
    glm::vec3 Lo;
};

struct PhotonProcess {

    PhotonProcess(uint32_t mp, ClosePhoton *buf);
    void operator()(const glm::vec3 &p, const Photon &photon, float dist2,
                    float &maxDistSquared);
    ClosePhoton *photons;
    uint32_t nLookup, nFound;
};

struct RadiancePhotonProcess {
    // RadiancePhotonProcess Methods
    RadiancePhotonProcess(const glm::vec3 &nn)
        :  n(nn) {
        photon = NULL;
    }
    void operator()(const glm::vec3 &p, const RadiancePhoton &rp,
                    float distSquared, float &maxDistSquared) {
        if (glm::dot(rp.n, n) > 0) {
            photon = &rp;
            maxDistSquared = distSquared;
        }
    }
    const glm::vec3 &n;
    const RadiancePhoton *photon;
};

// face forward
glm::vec3 Faceforward(glm::vec3 n, glm::vec3 v)
{
    return (glm::dot(n,v) < 0) ? -n : n;
}

inline float kernel(const Photon *photon, const glm::vec3 &p, float maxDist2);

static glm::vec3 LPhoton(KdTree<Photon> *map, int nPaths, int nLookup,
                         ClosePhoton *lookupBuf, Intersection &isx,
                         glm::vec3 &wo, float maxDistSquared);

static glm::vec3 EPhoton(
        KdTree<Photon> *map, int count, int nLookup,
        ClosePhoton *lookupBuf, float maxDist2,
        const glm::vec3 &p, const glm::vec3 &n);

PhotonProcess::PhotonProcess(uint32_t mp, ClosePhoton *buf) {
    photons = buf;
    nLookup = mp;
    nFound = 0;
}

struct ClosePhoton {

    ClosePhoton(const Photon *p = NULL, float md2 = INFINITY)
        : photon(p), distanceSquared(md2) { }
    bool operator<(const ClosePhoton &p2) const {
        return distanceSquared == p2.distanceSquared ?
                    (photon < p2.photon) : (distanceSquared < p2.distanceSquared);
    }
    const Photon *photon;
    float distanceSquared;
};


inline void PhotonProcess::operator()(
        const glm::vec3 &p,
        const Photon &photon, float distSquared, float &maxDistSquared)
{
    if (nFound < nLookup) {
        // Add photon to unordered array of photons
        photons[nFound++] = ClosePhoton(&photon, distSquared);
        if (nFound == nLookup) {
            std::make_heap(&photons[0], &photons[nLookup]);
            maxDistSquared = photons[0].distanceSquared;
        }
    }
    else {
        // Remove most distant photon from heap and add new photon
        std::pop_heap(&photons[0], &photons[nLookup]);
        photons[nLookup-1] = ClosePhoton(&photon, distSquared);
        std::push_heap(&photons[0], &photons[nLookup]);
        maxDistSquared = photons[0].distanceSquared;
    }
}

glm::vec3 EPhoton(
        KdTree<Photon> *map, int count, int nLookup,
        ClosePhoton *lookupBuf, float maxDist2,
        const glm::vec3 &p, const glm::vec3 &n)
{
    if (!map)
        return glm::vec3(0.f);

    // Lookup nearby photons at irradiance computation point
    PhotonProcess proc(nLookup, lookupBuf);

    float md2 = maxDist2;
    map->Lookup(p, proc, md2);
    assert(md2 > 0.f);

    // Accumulate irradiance value from nearby photons
    if (proc.nFound == 0)
        return glm::vec3(0.f);

    ClosePhoton *photons = proc.photons;
    glm::vec3 E(0.);
    for (uint32_t i = 0; i < proc.nFound; ++i)
        if (glm::dot(n, photons[i].photon->wi) > 0.)
            E += photons[i].photon->alpha;

    return E / (count * md2 * PI);
}

inline float kernel(const Photon *photon, const glm::vec3 &p,
                    float maxDist2) {
    float s = (1.f - glm::distance2(photon->p, p) / maxDist2);
    return 3.f * INV_PI * s * s;
}

glm::vec3 LPhoton(KdTree<Photon> *map,
                  int nPaths, int nLookup,
                  ClosePhoton *lookupBuf,
                  Intersection &isx,
                  glm::vec3 &wo, float maxDistSquared)
{
    std::uniform_real_distribution<float> rndDis(0,1);
    std::mt19937 rndGenerator(std::chrono::system_clock::now().time_since_epoch().count());

    glm::vec3 L(0);
    BxDFType nonSpecular = BxDFType(BSDF_REFLECTION | BSDF_TRANSMISSION |
                                    BSDF_DIFFUSE | BSDF_GLOSSY);

    if(map && isx.object_hit->material->NumComponents(nonSpecular) > 0)
    {

        //* do photon map lookup at intersection point *//
        PhotonProcess proc(nLookup, lookupBuf);
        map->Lookup(isx.point,proc,maxDistSquared);

        glm::vec3 woLocal = isx.ToLocalNormalCoordinate(wo);

        //* estimate reflected radiance dut to incident photons *//
        ClosePhoton *photons = proc.photons;
        int nFound = proc.nFound;
        glm::vec3 Nf = Faceforward(isx.normal, wo);
        if(isx.object_hit->material->NumComponents(
                    BxDFType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_GLOSSY)) > 0)
        {
            //* compute exitant radiance from photons for glossy surface *//
            for(int i=0;i<nFound;++i)
            {
                float pdf;
                const Photon* p = photons[i].photon;
                float k = kernel(p,isx.point,maxDistSquared);
                glm::vec3 pWiLocal = isx.ToLocalNormalCoordinate(p->wi);
                L += (k / (nPaths * maxDistSquared)) *
                        p->alpha *
                        isx.object_hit->material->EvaluateScatteredEnergy(isx,woLocal,pWiLocal,pdf);
            }
        }
        else
        {
            //* compute exitant radiance form photons for diffuse surface *//
            glm::vec3 Lr(0),Lt(0);
            for(int i=0;i<nFound;i++)
            {
                if(glm::dot(Nf,photons[i].photon->wi) > 0)
                {
                    float k = kernel(photons[i].photon, isx.point, maxDistSquared);
                    Lr += (k / (nPaths * maxDistSquared)) * photons[i].photon->alpha;
                }
                else
                {
                    float k = kernel(photons[i].photon, isx.point, maxDistSquared);
                    Lt += (k / (nPaths * maxDistSquared)) * photons[i].photon->alpha;
                }
            }


            L += Lr * isx.object_hit->material->rho(woLocal, rndDis, rndGenerator, BSDF_ALL_REFLECTION) * INV_PI +
                 Lt * isx.object_hit->material->rho(woLocal, rndDis, rndGenerator, BSDF_ALL_TRANSMISSION) * INV_PI ;
        }
    }
    return L;
}

bool IsBlack(const glm::vec3 &color)
{
    float r = glm::dot(glm::vec3(1),glm::abs(color));
    return (r < 1e-6f);
}


PhotonIntegrator::PhotonIntegrator()
{
    //std::cout<<"Constructor in Photon Integrator!\n";

    // initialize parameters
#define TEST_SCENE_INPUT 0
#if TEST_SCENE_INPUT
    nCausticPhotonsWanted = 1;
    nIndirectPhotonsWanted = 1;
#else
    nCausticPhotonsWanted = 200000;
    nIndirectPhotonsWanted = 1000000;
#endif

    nLookup = 100;
    maxSpecularDepth = 5;
    maxPhotonDepth = 5;
    finalGather = false;
    gatherSamples = 32;
    maxDistSquared = 0.3f * 0.3f;
    cosGatherAngle = cos(10.0f * DEG2RAD);

    nCausticPaths = nIndirectPaths = 0;
    causticMap = indirectMap = NULL;
    radianceMap = NULL;

}

PhotonIntegrator::~PhotonIntegrator()
{

    std::cout<<"Deconstructor in Photon Integrator!\n";
    /*
    std::cout
            <<causticMap<<","
            <<indirectMap<<","
            <<radianceMap<<std::endl;
    */

    if(causticMap) delete causticMap;
    if(indirectMap) delete indirectMap;
    if(radianceMap) delete radianceMap;
}

glm::vec3 PhotonIntegrator::TraceRay(Ray r, unsigned int depth)
{
    if(depth > maxPhotonDepth)
        return glm::vec3(0);

    Intersection isx = intersection_engine->GetIntersection(r);
    if(isx.t<0)
        return glm::vec3(0,0,0);
    if(isx.object_hit->material->is_light_source)
        return isx.object_hit->material->base_color;

    glm::vec3 L(0);
    glm::vec3 woWorld = -r.direction;
    glm::vec3 woLocal = isx.ToLocalNormalCoordinate(woWorld);

    glm::vec3 &p = isx.point;
    glm::vec3 &n = isx.normal;

    ClosePhoton* lookupBuf = new ClosePhoton[nLookup];

    //* evaluate direct light *//
    L += EstimateDirectLight(isx, r, scene->lights[0]);

    //* caustic lighting *//
    L += LPhoton(causticMap, nCausticPaths, nLookup, lookupBuf, isx,woWorld, maxDistSquared);

    //* indirect lighting *//
#define INDIRECT_LIGHTING 0
#if INDIRECT_LIGHTING
    if(finalGather && indirectMap != NULL)
    {
        BxDFType nonSpecular = BxDFType(BSDF_REFLECTION |
                                        BSDF_TRANSMISSION |
                                        BSDF_DIFFUSE |
                                        BSDF_GLOSSY);
        if(isx.object_hit->material->NumComponents(nonSpecular) > 0)
        {
            const uint32_t nIndirSamplePhotons = 50;
            PhotonProcess proc(nIndirSamplePhotons, lookupBuf);
            float searchDist2 = maxDistSquared;
            while(proc.nFound < nIndirSamplePhotons)
            {
                float md2 = searchDist2;
                proc.nFound = 0;
                indirectMap->Lookup(p,proc,md2);
                searchDist2 *= 2.0f;
            }

            //copy directions
            glm::vec3 *photonDirs = new glm::vec3[nIndirSamplePhotons];
            for(uint32_t i=0;i<nIndirSamplePhotons;++i)
            {
                photonDirs[i] = proc.photons[i].photon->wi;
            }

            //use bsdf to do Final Gathering
            glm::vec3 Li(0);
            for(int i=0;i<gatherSamples;++i)
            {
                glm::vec3 wiWorld,wiLocal;
                float pdf;

                glm::vec3 fr = isx.object_hit->material->SampleAndEvaluateScatteredEnergy(
                            isx,
                            woLocal,wiLocal,pdf,
                            BxDFType(BSDF_ALL & ~BSDF_SPECULAR));
                wiWorld = isx.ToWorldNormalCoordinate(wiLocal);

                if(IsBlack(fr) || pdf < 1e-6)
                    continue;

                // trace BSDF
                Ray bounceRay(p + wiWorld*(1e-3f), wiWorld);
                Intersection gatherIntersect = intersection_engine->GetIntersection(bounceRay);
                if(gatherIntersect.t > 0)
                {
                    // compute exitant radiance
                    glm::vec3 Lindir(0.0f);
                    glm::vec3 nGather = gatherIntersect.normal;
                    nGather = Faceforward(nGather, -bounceRay.direction);

                    RadiancePhotonProcess proc(nGather);
                    float md2 = INFINITY;
                    radianceMap->Lookup(gatherIntersect.point, proc, md2);
                    if(proc.photon != NULL)
                        Lindir = proc.photon->Lo;


                    //compute MIS weight
                    //compute PDF for photon-sampling
                    float photonPdf = 0.0f;
                    float conePdf = UniformConePdf(cosGatherAngle);

                    for(uint32_t j=0;j<nIndirSamplePhotons;++j)
                    {
                        if(glm::dot(photonDirs[j], wiWorld) > 0.999f * cosGatherAngle)
                            photonPdf += conePdf;
                    }
                    photonPdf /= nIndirSamplePhotons;
                    float wt = PowerHeuristic(pdf,gatherSamples,photonPdf,gatherSamples);
                    Li += fr * Lindir * (glm::abs(glm::dot(wiWorld,n)) * wt / pdf);

                }
            }
            L += Li / float(gatherSamples);

            // use nearby photons to do final gathering
            Li = glm::vec3(0);
            for(int i=0;i<gatherSamples;i++)
            {
                int photonNum = glm::min((int)nIndirSamplePhotons - 1,
                                         (int)(uniform_distribution(generator)*nIndirectPaths+1));

                // sample gather ray direction from photonNum
                glm::vec3 vx,vy;
                CoordinateSystem(photonDirs[photonNum], &vx, &vy);
                glm::vec3 wi = UniformSampleCone(uniform_distribution(generator),uniform_distribution(generator),
                                                 cosGatherAngle, vx,vy,photonDirs[photonNum]);

                //trace photon-sampled final gather ray and accumulate radiance
                float bsdfPdf;
                glm::vec3 fr = isx.object_hit->material->EvaluateScatteredEnergy(isx,woWorld,wi,bsdfPdf);
                Ray bounceRay(p + 1e-3f * wi, wi);
                Intersection gatherIsect = intersection_engine->GetIntersection(bounceRay);
                if(gatherIsect.t > 0)
                {
                    glm::vec3 Lindir(0.0f);
                    glm::vec3 nGather = gatherIsect.normal;
                    nGather = Faceforward(nGather, -bounceRay.direction);

                    RadiancePhotonProcess proc(nGather);
                    float md2 = INFINITY;
                    radianceMap->Lookup(gatherIsect.point, proc, md2);
                    if(proc.photon != NULL)
                        Lindir = proc.photon->Lo;

                    // compute pdf for photon-sampling of direction wi
                    float photonPdf = 0.0f;
                    float conePdf = UniformConePdf(cosGatherAngle);
                    for(uint32_t j=0;j<nIndirSamplePhotons;j++)
                    {
                        if(glm::dot(photonDirs[j], wi) > 0.999f * cosGatherAngle)
                            photonPdf += conePdf;
                    }
                    photonPdf /= nIndirSamplePhotons;

                    // compute MIS weight
                    //float bsdfPdf;
                    //isx.object_hit->material->EvaluateScatteredEnergy(isx,woWorld,wi,bsdfPdf);
                    float wt = PowerHeuristic(photonPdf,gatherSamples,bsdfPdf,gatherSamples);
                    Li += fr * Lindir * glm::abs(glm::dot(wi,n)) * wt / photonPdf;
                }
            }
            L += Li / float(gatherSamples);
        }
    }
    else
    {
        L += LPhoton(indirectMap, nIndirectPaths,
                     nLookup, lookupBuf,
                     isx,woWorld,maxDistSquared);
    }
#endif

    //* specular reflect and transmit *//
    L += SpecularTransmit(r, depth, isx);
    L += SpecularReflect(r, depth, isx);

    delete[] lookupBuf;
    float check = glm::dot(L,glm::vec3(1,1,1));
    return (glm::isinf(check) || glm::isnan(check)) ? glm::vec3(0) : L ;

}

void PhotonIntegrator::Preprocessing()
{
    std::cout<<"Photon integrator preprocessing starts!\n";

    // shared variables for shooting photons
    int nDirectPaths = 0;
    std::vector<Photon> directPhotons, indirectPhotons, causticPhotons;
    std::vector<RadiancePhoton> radiancePhotons;

    bool abortTasks = false;
    causticPhotons.reserve(nCausticPhotonsWanted);
    indirectPhotons.reserve(nIndirectPhotonsWanted);
    
    uint32_t nshot=0;

    // rpReflectances and rpTransmittances
    std::vector<glm::vec3> rpReflectances, rpTransmittances;

    // light distribution ???
    int nTasks = 16; // max 16 threads
    QThread** photonShootingTasks = new QThread*[nTasks];
    for(int i=0;i<nTasks;i++)
    {
        photonShootingTasks[i] =
                new PhotonShootingTask(i,0.f,this,abortTasks,nDirectPaths,
                                       directPhotons,indirectPhotons,causticPhotons,
                                       radiancePhotons,
                                       nshot,this->scene,
                                       rpReflectances, rpTransmittances);
        photonShootingTasks[i]->start();
    }
    
    // wait for photon threads running and clean up threads
    WaitForAllThreads(photonShootingTasks, nTasks);

    // build kd-tree for direct , indirect and caustic photons
    KdTree<Photon> *directMap = NULL;
    if(!directPhotons.empty())
        directMap = new KdTree<Photon>(directPhotons);
    if(!causticPhotons.empty())
        causticMap = new KdTree<Photon>(causticPhotons);
    if(!indirectPhotons.empty())
        indirectMap = new KdTree<Photon>(indirectPhotons);

    // precomute radiance at a subset of the photons
    if(this->finalGather && radiancePhotons.size())
    {
        uint32_t numTasks = 64;
        QThread** radianceTasks = new QThread*[numTasks];

        for (uint32_t i = 0; i < numTasks; ++i)
        {
            radianceTasks[i] =
                    new ComputeRadianceTask(i, numTasks,
                                            radiancePhotons,
                                            rpReflectances, rpTransmittances,
                                            nLookup, maxDistSquared,
                                            nDirectPaths, directMap,
                                            nIndirectPaths, indirectMap,
                                            nCausticPaths, causticMap);
            radianceTasks[i]->start();
        }

        WaitForAllThreads(radianceTasks, numTasks);
        radianceMap = new KdTree<RadiancePhoton>(radiancePhotons);
    }

    delete directMap;
}

void PhotonIntegrator::WaitForAllThreads(QThread** threads, int nTasks)
{
    bool is_stillRunning;
    do
    {
        is_stillRunning = false;
        for(int i=0;i<nTasks;++i)
        {
            if(threads[i]->isRunning())
            {
                is_stillRunning = true;
                break;
            }
        }
        if(is_stillRunning)
        {
            QThread::yieldCurrentThread();
        }
    }
    while(is_stillRunning);

    // clean up
    for(int i=0;i<nTasks;i++)
    {
        delete threads[i];
    }
    delete []threads;
}

void PhotonShootingTask::run()
{
    const uint32_t blockSize = 4096;

    std::uniform_real_distribution<float> &rndDistribution = integrator->uniform_distribution;
    std::mt19937 &rndGenerator = integrator->generator;

    // local photons buffer
    std::vector<Photon> localDirectPhotons,localIndirectPhotons,localCausticPhotons;
    std::vector<RadiancePhoton> localRadiancePhotons;
    std::vector<glm::vec3> localRpReflectances, localRpTransmittances;

    // stop signals
    bool causticDone = (integrator->nCausticPhotonsWanted == 0);
    bool indirectDone = (integrator->nIndirectPhotonsWanted == 0);

    Geometry* light = scene->lights[0];// only one light

    while(true)
    {
        for(uint32_t i=0;i<blockSize;++i)
        {
            Ray photonRay;
            float pdf;
            glm::vec3 normal;
            float u[2] = {rndDistribution(rndGenerator), rndDistribution(rndGenerator)};
            glm::vec3 Le = light->Sample_L(scene,u,photonRay,normal,pdf);

            glm::vec3 alpha = Le / (pdf);

            // start photon path tracing
            bool specularPath = true;
            int nIntersection = 0;

            Intersection photonIsect;
            photonIsect = integrator->intersection_engine->GetIntersection(photonRay);
            while(photonIsect.t > 0)
            {
                ++nIntersection;
                QVector<BxDF*> photonBxdfs = photonIsect.object_hit->material->bxdfs.toVector();
                BxDFType specularType = BxDFType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_SPECULAR);
                int numSpecularType = 0;
                for(int i=0;i<photonBxdfs.size();++i)
                {
                    numSpecularType += int(photonBxdfs[i]->MatchesFlags(specularType));
                }
                bool hasNonSpecular = (photonBxdfs.size() > numSpecularType);

                glm::vec3 wo = -photonRay.direction;
                if(hasNonSpecular)
                {
                    Photon photon(photonIsect.point, alpha, wo);
                    bool depositedPhoton = false;

                    if(specularPath && nIntersection > 1) // caustic photon
                    {
                        if(!causticDone)
                        {
                            depositedPhoton = true;
                            localCausticPhotons.push_back(photon);
                        }
                    }
                    else // direct or indirect photon
                    {
                        if(nIntersection == 1 && !indirectDone && integrator->finalGather)
                        {
                            depositedPhoton = true;
                            localDirectPhotons.push_back(photon);
                        }
                        else if(nIntersection > 1 && !indirectDone)
                        {
                            depositedPhoton = true;
                            localIndirectPhotons.push_back(photon);
                        }
                    }

                    //possibly create radiance photon at photon intersection point
                    if(depositedPhoton && integrator->finalGather && rndDistribution(rndGenerator) < 0.125f)
                    {
                        glm::vec3 n = photonIsect.normal;
                        n = Faceforward(n, -photonRay.direction);
                        localRadiancePhotons.push_back(RadiancePhoton(photonIsect.point, n));

                        // rpReflectances and rpTransmittances
                        glm::vec3 rho_r = photonIsect.object_hit->material->rho(BSDF_ALL_REFLECTION,rndDistribution,rndGenerator);
                        localRpReflectances.push_back(rho_r);

                        glm::vec3 rho_t = photonIsect.object_hit->material->rho(BSDF_ALL_TRANSMISSION,rndDistribution,rndGenerator);
                        localRpTransmittances.push_back(rho_t);
                    }

                }

                if(nIntersection >= integrator->maxPhotonDepth)
                    break;

                // sample new photon ray direction
                glm::vec3 wiLocal;
                float pdfBxdf;
                BxDFType flags;
                glm::vec3 woLocal = photonIsect.ToLocalNormalCoordinate(wo);
                float u[]={rndDistribution(rndGenerator),rndDistribution(rndGenerator),rndDistribution(rndGenerator)};
                glm::vec3 fr = photonIsect.object_hit->material->BSDF_Sample_f(woLocal, u, wiLocal, pdfBxdf, flags);
                glm::vec3 wiWorld = photonIsect.ToWorldNormalCoordinate(wiLocal);

                if(pdf == 0.0f) break;

                glm::vec3 anew = alpha * fr * glm::abs(glm::dot(wiWorld, photonIsect.normal)) / pdfBxdf;

                // possibly terminate photon path
                //float continueProb = fmin(1.0f, anew.y / alpha.y);
                //if(rndDistribution(rndGenerator) > continueProb)
                //    break;
                //alpha = anew / continueProb;
                alpha = anew;

                specularPath &= ((flags&BSDF_SPECULAR)!=0);

                if(indirectDone && !specularPath) break;

                photonRay = Ray(photonIsect.point + wiWorld * (1e-3f), wiWorld);

                photonIsect = integrator->intersection_engine->GetIntersection(photonRay);
            }

        }

        if(abortTasks)
        {
            break;
        }

        //merge local photons
        mutex.lock();

        nshot += blockSize; // stop after trying so many times
        if(nshot > 5000000)
            abortTasks = true;

        // merge indirect photons
        if(!indirectDone)
        {
            integrator->nIndirectPaths += blockSize;
            for(uint32_t i=0;i<localIndirectPhotons.size();++i)
            {
                indirectPhotons.push_back(localIndirectPhotons[i]);
            }
            localIndirectPhotons.erase(localIndirectPhotons.begin(),localIndirectPhotons.end());

            if(indirectPhotons.size() >= integrator->nIndirectPhotonsWanted)
                indirectDone = true;


            nDirectPaths += blockSize;
            for(uint32_t i=0;i<localDirectPhotons.size();i++)
            {
                directPhotons.push_back(localDirectPhotons[i]);
            }
            localDirectPhotons.erase(localDirectPhotons.begin(),localDirectPhotons.end());
        }


        // merge direct , caustic photons and radiance photons
        if(!causticDone)
        {
            integrator->nCausticPaths += blockSize;
            for(uint32_t i=0;i<localCausticPhotons.size();++i)
            {
                causticPhotons.push_back(localCausticPhotons[i]);
            }
            localCausticPhotons.erase(localCausticPhotons.begin(),localCausticPhotons.end());
            if(causticPhotons.size()>= integrator->nCausticPhotonsWanted)
                causticDone = true;
        }

        for(uint32_t i=0;i<localRadiancePhotons.size();++i)
        {
            radiancePhotons.push_back(localRadiancePhotons[i]);
        }
        localRadiancePhotons.erase(localRadiancePhotons.begin(),localRadiancePhotons.end());

        for(uint32_t i=0;i<localRpReflectances.size();++i)
        {
            rpReflectances.push_back(localRpReflectances[i]);
        }
        localRpReflectances.erase(localRpReflectances.begin(),localRpReflectances.end());

        for(uint32_t i=0;i<localRpTransmittances.size();++i)
        {
            rpTransmittances.push_back(localRpTransmittances[i]);
        }
        localRpTransmittances.erase(localRpTransmittances.begin(),localRpTransmittances.end());


        //debug
        std::cout
                <<taskNum<<": "
               <<"indirect photons ="<<indirectPhotons.size()<<", "
              <<"causticPhotons ="<<causticPhotons.size()<<std::endl;

        mutex.unlock();

        // exit task
        if(indirectDone && causticDone)
            break;
    }
}

void ComputeRadianceTask::run()
{
    // Compute range of radiance photons to process in task
    uint32_t taskSize = radiancePhotons.size() / numTasks;
    uint32_t excess = radiancePhotons.size() % numTasks;
    uint32_t rpStart = glm::min(taskNum, excess) * (taskSize+1) +
            glm::max(0, (int)taskNum-(int)excess) * taskSize;
    uint32_t rpEnd = rpStart + taskSize + (taskNum < excess ? 1 : 0);

    if (taskNum == numTasks-1)
        assert(rpEnd == radiancePhotons.size());

    ClosePhoton *lookupBuf = new ClosePhoton[nLookup];
    for (uint32_t i = rpStart; i < rpEnd; ++i)
    {
        // Compute radiance for radiance photon _i_
        RadiancePhoton &rp = radiancePhotons[i];
        const glm::vec3 &rho_r = rpReflectances[i], &rho_t = rpTransmittances[i];
        if (!IsBlack(rho_r))
        {
            // Accumulate outgoing radiance due to reflected irradiance
            glm::vec3 E = EPhoton(directMap, nDirectPaths, nLookup, lookupBuf,
                                  maxDistSquared, rp.p, rp.n) +
                    EPhoton(indirectMap, nIndirectPaths, nLookup, lookupBuf,
                            maxDistSquared, rp.p, rp.n) +
                    EPhoton(causticMap, nCausticPaths, nLookup, lookupBuf,
                            maxDistSquared, rp.p, rp.n);
            rp.Lo += INV_PI * rho_r * E;
        }

        if (!IsBlack(rho_t))
        {
            // Accumulate outgoing radiance due to transmitted irradiance
            glm::vec3 E = EPhoton(directMap, nDirectPaths, nLookup, lookupBuf,
                                  maxDistSquared, rp.p, -rp.n) +
                    EPhoton(indirectMap, nIndirectPaths, nLookup, lookupBuf,
                            maxDistSquared, rp.p, -rp.n) +
                    EPhoton(causticMap, nCausticPaths, nLookup, lookupBuf,
                            maxDistSquared, rp.p, -rp.n);
            rp.Lo += INV_PI * rho_t * E;
        }
    }

    delete[] lookupBuf;
}

glm::vec3 PhotonIntegrator::SpecularTransmit(Ray r, int depth, Intersection &isx)
{
    if(depth >= maxSpecularDepth)
        return glm::vec3(0);

    glm::vec3 woWorld = -r.direction;
    glm::vec3 woLocal = isx.ToLocalNormalCoordinate(woWorld);

    glm::vec3 wiLocal,wiWorld;

    glm::vec3 &p = isx.point;
    glm::vec3 &n = isx.normal;

    float pdf;
    glm::vec3 f = isx.object_hit->material->SampleAndEvaluateScatteredEnergy(isx,woLocal,wiLocal,pdf,BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
    wiWorld = isx.ToWorldNormalCoordinate(wiLocal);

    glm::vec3 L(0);
    if(pdf > 0 && !IsBlack(f) && glm::abs(glm::dot(wiWorld,n)) > 1e-6f)
    {
        Ray newRay(p + 1e-3f * wiWorld, wiWorld);
        glm::vec3 Li = TraceRay(newRay, depth + 1);
        L = f * Li * glm::abs(glm::dot(n,wiWorld)) / pdf;
    }
    return L;
}

glm::vec3 PhotonIntegrator::SpecularReflect(Ray r, int depth, Intersection &isx)
{
    if(depth >= maxSpecularDepth)
        return glm::vec3(0);

    glm::vec3 woWorld = -r.direction;
    glm::vec3 woLocal = isx.ToLocalNormalCoordinate(woWorld);

    glm::vec3 wiLocal, wiWorld;

    glm::vec3 &p = isx.point;
    glm::vec3 &n = isx.normal;

    float pdf;
    glm::vec3 f = isx.object_hit->material->SampleAndEvaluateScatteredEnergy(isx,woLocal,wiLocal,pdf,BxDFType(BSDF_REFLECTION | BSDF_SPECULAR));
    wiWorld = isx.ToWorldNormalCoordinate(wiLocal);

    glm::vec3 L(0);
    if(pdf > 0 && !IsBlack(f) && glm::abs(glm::dot(wiWorld,n)) > 1e-6f)
    {
        Ray newRay(p + 1e-3f * wiWorld, wiWorld);
        glm::vec3 Li = TraceRay(newRay, depth + 1);
        L = f * Li * glm::abs(glm::dot(n, wiWorld)) / pdf;
    }
    return L;

}
