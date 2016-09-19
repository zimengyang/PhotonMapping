#include <scene/materials/bxdfs/specularreflectionBxDF.h>
#include <raytracing/samplers/samplercommon.h>

glm::vec3 SpecularReflectionBxDF::EvaluateScatteredEnergy(const glm::vec3 &wo, const glm::vec3 &wi, float& pdf) const
{
    pdf = PDF(wo, wi);
    if(isPerfectReflective(wo,wi))
        return reflection_color;
    else
        return glm::vec3(0);
}
glm::vec3 SpecularReflectionBxDF::EvaluateHemisphereScatteredEnergy(const glm::vec3 &wo, int num_samples, const glm::vec2 *samples) const
{
    //TODO
    return glm::vec3(0);
}

glm::vec3 SpecularReflectionBxDF::SampleAndEvaluateScatteredEnergy(const glm::vec3 &wo, glm::vec3 &wi_ret, float rand1, float rand2, float &pdf_ret) const
{
    // specular reflection wi = delta(w-wo) where w is perfectly reflective direction
    glm::vec3 N(0,0,1);
    wi_ret = 2.0f*glm::dot(N,wo)*N - wo;
    wi_ret = glm::normalize(wi_ret);
    //pdf_ret = PDF(wo, wi_ret);

    return EvaluateScatteredEnergy(wo, wi_ret, pdf_ret);
}

float SpecularReflectionBxDF::PDF(const glm::vec3 &wo, const glm::vec3 &wi) const
{
    return 1.0f;
    /*
    if(isPerfectReflective(wo,wi))
        return INFINITY;
    else
        return 0.0f;
    */
}

glm::vec3 SpecularReflectionBxDF::rho(int numSamples, float *u1, float *u2)
{
    glm::vec3 r(0);
    for (int i = 0; i < numSamples; ++i) {
        // Estimate one term of $\rho_\roman{hh}$
        glm::vec3 wo, wi;
        wo = UniformSampleHemisphere(u1[i], u2[i]);
        float pdf_o = INV_2_PI, pdf_i = 0.f;
        glm::vec3 f = SampleAndEvaluateScatteredEnergy(wo, wi, u1[i], u2[i], pdf_i);
        if (pdf_i > 0.)
            r += f * glm::abs(wi.z) * glm::abs(wo.z) / (pdf_o * pdf_i);
    }
    return r / float(M_PI*numSamples);
}

glm::vec3 SpecularReflectionBxDF::rho(const glm::vec3 &wo, int nSamples, float* u1, float* u2)
{
    glm::vec3 r(0);
    for (int i = 0; i < nSamples; ++i)
    {
        // Estimate one term of $\rho_\roman{hd}$
        glm::vec3 wi;
        float pdf = 0.f;
        glm::vec3 f = SampleAndEvaluateScatteredEnergy(wo, wi, u1[i],u2[i], pdf);
        if (pdf > 0.)
            r += f * glm::abs(wi.z) / pdf;
    }
    return r / float(nSamples);
}
