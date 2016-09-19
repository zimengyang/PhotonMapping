#include <raytracing/bidirectionalintegrator.h>

BidirectionalIntegrator::BidirectionalIntegrator()
{
    // constructor
}
BidirectionalIntegrator::~BidirectionalIntegrator()
{
    // deconstructor
    std::cout<<"Deconstructor in " << "Bidirectional Integrator."<<std::endl;
}

std::vector<PathNode> BidirectionalIntegrator::generateEyePath(Ray r)
{
    std::vector<PathNode> eyePath;
    eyePath.clear();

    Intersection isx = intersection_engine->GetIntersection(r);
    int depth=0;
    while(depth<max_depth && isx.t > 0)
    {
        // store the path node
        PathNode node;
        node.isx = isx;
        node.dirIn_world = -r.direction;
        node.dirIn_local = isx.ToLocalNormalCoordinate(-r.direction);
        node.F = isx.object_hit->material->SampleAndEvaluateScatteredEnergy(isx,node.dirIn_local,node.dirOut_local,node.pdf);
        node.dirOut_world = isx.ToWorldNormalCoordinate(node.dirOut_local);

        if(node.pdf != 0)
            eyePath.push_back(node);
        else
            break;

        //update r
        r = Ray(isx.point + glm::sign(glm::dot(node.dirOut_world,isx.normal)) * isx.normal*1e-3f, node.dirOut_world);

        // update isx and depth info
        depth++;
        isx = intersection_engine->GetIntersection(r);

    }

    return eyePath;
}

std::vector<PathNode> BidirectionalIntegrator::generateLightPath(Geometry* &light)
{
    std::vector<PathNode> lightPath;
    lightPath.clear();

    Intersection lightSample = light->RandomSampleOnSurface(uniform_distribution(generator),uniform_distribution(generator));

    Ray r(lightSample.point, lightSample.point - light->transform.position());

    Intersection isx = intersection_engine->GetIntersection(r);
    int depth = 0;
    while(isx.t > 0 && depth < max_depth)
    {
        // store pathnode on the light path
        PathNode node;
        node.isx = isx;
        node.dirIn_world = -r.direction;
        node.dirIn_local = isx.ToLocalNormalCoordinate(-r.direction);
        node.F = isx.object_hit->material->SampleAndEvaluateScatteredEnergy(isx,node.dirIn_local,node.dirOut_local,node.pdf);
        node.dirOut_world = isx.ToWorldNormalCoordinate(node.dirOut_local);

        if(node.pdf != 0)
            lightPath.push_back(node);
        else
            break;

        //update r
        r = Ray(isx.point + glm::sign(glm::dot(node.dirOut_world,isx.normal)) * isx.normal*1e-3f, node.dirOut_world);

        // update isx and depth info
        depth++;
        isx = intersection_engine->GetIntersection(r);
    }

    return lightPath;
}

float BidirectionalIntegrator::G(PathNode &a, PathNode &b)
{
    glm::vec3 v = glm::normalize(a.isx.point- b.isx.point);

    return glm::abs(glm::dot(a.isx.normal,v) * glm::dot(b.isx.normal,v)) / glm::distance2(a.isx.point,b.isx.point);
}

bool BidirectionalIntegrator::isBlocked(const Intersection &isx1, const Intersection &isx2)
{
    glm::vec3 direction =glm::normalize(isx2.point - isx1.point);
    Ray r(isx1.point + 1e-3f*direction,direction);
    Intersection inter = intersection_engine->GetIntersection(r);
    if(inter.object_hit != isx2.object_hit)
        return true;
    else
        return false;

}
glm::vec3 BidirectionalIntegrator::EvaluatePath(std::vector<PathNode> &eyePath, int nEye, std::vector<PathNode> &lightPath, int nLight)
{

    if (isBlocked(eyePath[nEye-1].isx, lightPath[nLight-1].isx))
    {
        return glm::vec3(0.f);
    }

    glm::vec3 L(1.0f);

    // eyePath
    for (int i = 0; i < nEye-1; ++i)
    {
        L *= eyePath[i].F * glm::abs(glm::dot(eyePath[i].dirOut_world, eyePath[i].isx.normal)) / (eyePath[i].pdf);
    }

    // connecting
    L *= eyePath[nEye-1].F * G(eyePath[nEye-1], lightPath[nLight-1]) * lightPath[nLight-1].F;

    // lightPath
    for (int i = nLight-2; i >= 0; --i)
    {
        L *= lightPath[i].F * glm::abs(glm::dot(lightPath[i].dirOut_world, lightPath[i].isx.normal)) / (lightPath[i].pdf);
    }

    return L;
}

glm::vec3 BidirectionalIntegrator::TraceRay(Ray r, unsigned int depth)
{
    Intersection isx = intersection_engine->GetIntersection(r);
    if(isx.t < 0)
        return glm::vec3(0);
    else if(isx.object_hit->material->is_light_source)
    {
        return isx.object_hit->material->base_color * isx.texture_color;
    }

    glm::vec3 resultColor(0);

    for(Geometry* light : scene->lights)
    {
        std::vector<PathNode> eyePath = generateEyePath(r);
        std::vector<PathNode> lightPath = generateLightPath(light);

        if(!eyePath.empty() && !lightPath.empty())
        {
            PathNode* node = &lightPath[0];
            float lightPdf = light->RayPDF(node->isx,Ray(node->isx.point,node->dirIn_world));
            glm::vec3 Le(0);
            if(lightPdf != 0)
                Le = light->material->base_color * light->material->intensity / lightPdf;

            glm::vec3 directWt(1.0f);
            for(int i=1;i<=eyePath.size();i++)
            {
                node = &eyePath[i-1];

                Ray ray(light->transform.position(), - node->dirIn_world);
                resultColor += directWt * EstimateDirectLight(node->isx, ray, light) / WeightPath(i,0);
                directWt *= node->F * glm::abs(glm::dot(node->dirOut_world,node->isx.normal)) / node->pdf;

                for(int j=1;j<=lightPath.size();j++)
                {
                    resultColor += Le * EvaluatePath(eyePath,i,lightPath,j) / WeightPath(i,j);
                }
            }
        }
        else
        {
            continue;
        }

    }
    return resultColor;
}

glm::vec3 BidirectionalIntegrator::EstimateDirectLight(Intersection &isx, Ray &ray, Geometry* &light)
{
    glm::vec3 Ld(0);

    Ld = MIS_SampleBRDF(isx,ray,light) + MIS_SampleLight(isx,ray,light);

    return Ld;
}

float BidirectionalIntegrator::PowerHeuristic(const float &pdf_s, const float &n_s, const float &pdf_f, const float &n_f)
{
    if(isinf(pdf_s))
        return 1.0f;
    if(isinf(pdf_f))
        return 0.0f;
    else if(fequal(pdf_f,0.0f) && fequal(pdf_s,0.0f))
        return 0.f;
    else
        return glm::pow(pdf_s * n_s,2.0f) / (glm::pow(pdf_s * n_s,2.0f) + glm::pow(pdf_f * n_f,2.0f));
}

// MIS: sampling light source
glm::vec3 BidirectionalIntegrator::MIS_SampleLight(Intersection &intersection, Ray &r, Geometry* &light)
{
    if(Number_Light == 0)
        return glm::vec3(0);

    // Direct light estimator: sample Light source
    glm::vec3 sum_light_sample(0);
    for(int i = 0; i < Number_Light; i++)
    {
        float u = uniform_distribution(generator);
        float v = uniform_distribution(generator);

        Intersection lightSample = light->SampleOnGeometrySurface(u, v, intersection.point + 1e-3f * intersection.normal);
        //Intersection lightSample = light->RandomSampleOnSurface(u,v);
        glm::vec3 wj = glm::normalize(lightSample.point - intersection.point);
        glm::vec3 wo = - r.direction;
        glm::vec3 P = intersection.point;
        glm::vec3 N = intersection.normal;

        float pdf_light = light->RayPDF(intersection, Ray(P + float(1e-3)*N, wj));

        Intersection lightIntersection = intersection_engine->GetIntersection(Ray(P + float(1e-3)*N, wj));
        float temp, pdf_brdf;

        glm::vec3 wo_local = intersection.ToLocalNormalCoordinate(wo);
        glm::vec3 wj_local = intersection.ToLocalNormalCoordinate(wj);

        glm::vec3 F = intersection.object_hit->material->EvaluateScatteredEnergy(intersection, wo_local, wj_local, pdf_brdf);
        // reach light directly && pdf(wj) > 0
        if(lightIntersection.t > 0 && lightIntersection.object_hit == light && pdf_light > 0 && pdf_brdf > 0)
        {

            glm::vec3 Ld = light->material->EvaluateScatteredEnergy(lightSample, wo, -wj, temp);
            float W = PowerHeuristic(pdf_light, float(Number_Light), pdf_brdf, float(Number_BRDF)); // cause shadow in center

            sum_light_sample = sum_light_sample +
                                  W * F * Ld * float(fabs(glm::dot(wj, N))) / pdf_light;

        }
    }
    return sum_light_sample / float(Number_Light);
}

// MIS: sampling BRDF
glm::vec3 BidirectionalIntegrator::MIS_SampleBRDF(Intersection &intersection, Ray &r, Geometry* &light)
{
    if(Number_BRDF == 0)
        return glm::vec3(0);

    // Direct light estimator: sample BRDF
    glm::vec3 sum_brdf_sample(0.0f);
    for(int i = 0; i < Number_BRDF; i++)
    {
        glm::vec3 wo_local = intersection.ToLocalNormalCoordinate(-r.direction);
        glm::vec3 wj_local;
        float pdf_brdf;
        glm::vec3 F = intersection.object_hit->material->SampleAndEvaluateScatteredEnergy(intersection,wo_local,wj_local,pdf_brdf);

        glm::vec3 wj_world = intersection.ToWorldNormalCoordinate(wj_local);
        glm::vec3 wo_world = - r.direction;

        Intersection isxOnLight = intersection_engine->GetIntersection(Ray(intersection.point+float(1e-3)*intersection.normal, wj_world));

        if(isxOnLight.t > 0 && isxOnLight.object_hit == light && pdf_brdf > 0)
        {
            float temp,pdf_light = light->RayPDF(intersection, Ray(intersection.point, wj_world));
            float W = PowerHeuristic(pdf_brdf,float(Number_BRDF),pdf_light,float(Number_Light));
            glm::vec3 Ld = light->material->EvaluateScatteredEnergy(isxOnLight,wo_world,-wj_world,temp);

            if(pdf_light > 0 )
            {
                if(isinf(pdf_brdf)) // delta specular surface
                {
                    sum_brdf_sample = sum_brdf_sample +
                            F * Ld * float(fabs(glm::dot(wj_world, intersection.normal))) / pdf_light;
                }
                else
                {
                    sum_brdf_sample = sum_brdf_sample +
                            W * F * Ld * float(fabs(glm::dot(wj_world,intersection.normal))) / pdf_brdf;
                }
            }
        }

    }
    return sum_brdf_sample / float(Number_BRDF);
}

