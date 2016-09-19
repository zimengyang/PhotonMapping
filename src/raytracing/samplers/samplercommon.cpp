#include <raytracing/samplers/samplercommon.h>

glm::vec3 UniformSampleSphere(float u1,float u2)
{
    float z = 1.0f - 2.0f * u1;
    float r = glm::sqrt(glm::max(0.0f,1.0f - z*z));
    float phi = 2.0f * PI * u2;

    float x = r * glm::cos(phi);
    float y = r * glm::sin(phi);
    glm::vec3 vec = glm::vec3(x,y,z);
    return glm::normalize(vec);
}

glm::vec3 UniformSampleHemisphere(float u1,float u2)
{
    float z = u1;
    float r = glm::sqrt(glm::max(0.f, 1.f - z*z));
    float phi = 2 * PI * u2;
    float x = r * glm::cos(phi);
    float y = r * glm::sin(phi);
    glm::vec3 vec(x,y,z);
    return glm::normalize(vec);
}

float UniformSpherePdf()
{
    return 1.0f/(4.0f*PI);
}

float UniformConePdf(float cosThetaMax) {
    return 1.f / (2.f * M_PI * (1.f - cosThetaMax));
}

inline float Lerp(float t, float v1, float v2)
{
    return (1.f - t) * v1 + t * v2;
}

glm::vec3 UniformSampleCone(float u1, float u2,
                            float costhetamax,
        const glm::vec3 &x, const glm::vec3 &y, const glm::vec3 &z)
{
    float costheta = Lerp(u1, costhetamax, 1.f);
    float sintheta = glm::sqrt(1.f - costheta*costheta);
    float phi = u2 * 2.f * M_PI;
    return glm::cos(phi) * sintheta * x +
           glm::sin(phi) * sintheta * y +
           costheta * z;
}
