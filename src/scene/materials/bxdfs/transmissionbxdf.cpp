#include <scene/materials/bxdfs/transmissionbxdf.h>
#include <raytracing/samplers/samplercommon.h>

glm::vec3 TransmissionBxDF::EvaluateScatteredEnergy(const glm::vec3 &wo, const glm::vec3 &wi, float &pdf) const
{
    // ?
    pdf = 0.0f;
    return glm::vec3(0);
}

glm::vec3 TransmissionBxDF::SampleAndEvaluateScatteredEnergy(const glm::vec3 &wo, glm::vec3 &wi_ret, float rand1, float rand2, float &pdf_ret) const
{
    float ei = etai;
    float et = etat;

    if(wo.z < 0)
    {
        float temp=ei;
        ei=et;
        et=temp;
    }

    float sini2 = glm::max(0.0f,1.0f-wo.z*wo.z);
    float eta = ei/et;
    float sint2 = eta * eta * sini2;
    if(sint2 >= 1.0f)
        return glm::vec3(0);
    float cost = glm::sqrt(glm::max(0.0f,1.0f-sint2));
    if(wo.z > 0)
        cost = -cost;
    float sintOverSini = eta;

    wi_ret = glm::vec3(sintOverSini*(-wo.x), sintOverSini *(-wo.y), cost);

    wi_ret = glm::normalize(wi_ret);

    pdf_ret = 1.0f;

    float F = fresnel->Evaluate(wo.z);

    return (1.0f-F) * trans_color * ((et*et)/(ei*ei))/glm::abs(wi_ret.z) ;
}

float TransmissionBxDF::PDF(const glm::vec3 &wo, const glm::vec3 &wi) const
{

    return 1.0f;
}

glm::vec3 TransmissionBxDF::rho(int numSamples, float *u1, float *u2)
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

glm::vec3 TransmissionBxDF::rho(const glm::vec3 &wo, int nSamples, float* u1, float* u2)
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
