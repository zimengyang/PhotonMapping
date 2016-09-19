#ifndef SAMPLERCOMMON
#define SAMPLERCOMMON

#include <la.h>

glm::vec3 UniformSampleSphere(float u1,float u2);

glm::vec3 UniformSampleHemisphere(float u1,float u2);

float UniformSpherePdf();
float UniformConePdf(float cosThetaMax);

glm::vec3 UniformSampleCone(float u1, float u2,
                            float costhetamax,
        const glm::vec3 &x, const glm::vec3 &y, const glm::vec3 &z);

#endif // SAMPLERCOMMON

