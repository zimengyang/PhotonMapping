#include <scene/materials/bxdfs/lambertBxDF.h>
#include <raytracing/samplers/samplercommon.h>

glm::vec3 LambertBxDF::EvaluateScatteredEnergy(const glm::vec3 &wo, const glm::vec3 &wi,float &pdf) const
{
    pdf = PDF(wo,wi);
    // take the material's base color and divided by pi
    if(wo.z < 0)
        return glm::vec3(0);
    else
        return this->diffuse_color / PI;
}
glm::vec3 LambertBxDF::EvaluateHemisphereScatteredEnergy(const glm::vec3 &wo, int num_samples, const glm::vec2 *samples) const
{
    //TODO
    return glm::vec3(0);
}

glm::vec3 LambertBxDF::SampleAndEvaluateScatteredEnergy(const glm::vec3 &wo, glm::vec3 &wi_ret, float rand1, float rand2, float &pdf_ret) const
{
    // sample half-angle vector wi_ret
    float costheta = glm::pow(rand1, 0.5f);
    float sintheta = glm::sqrt(glm::max(0.0f, 1.0f - costheta*costheta));
    float phi = rand2 * 2* PI;
    wi_ret = SphericalDirection(sintheta,costheta,phi);
    if(wo.z * wi_ret.z < 0.0f)
        wi_ret = -wi_ret;

    return EvaluateScatteredEnergy(wo, wi_ret, pdf_ret);
}

float LambertBxDF::PDF(const glm::vec3 &wo, const glm::vec3 &wi) const
{

    if(wo.z < 0 || wo.z * wi.z < 0)
        return 0;
    else
        return 1.0f / PI * glm::abs(wo.z);

}

glm::vec3 LambertBxDF::rho(int numSamples, float *u1, float *u2)
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

glm::vec3 LambertBxDF::rho(const glm::vec3 &wo, int nSamples, float* u1, float* u2)
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
