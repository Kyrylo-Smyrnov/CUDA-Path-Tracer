#include "interactions.h"

__host__ __device__ glm::vec3 calculateRandomDirectionInHemisphere(
    glm::vec3 normal,
    thrust::default_random_engine &rng)
{
    thrust::uniform_real_distribution<float> u01(0, 1);

    float up = sqrt(u01(rng)); // cos(theta)
    float over = sqrt(1 - up * up); // sin(theta)
    float around = u01(rng) * TWO_PI;

    // Find a direction that is not the normal based off of whether or not the
    // normal's components are all equal to sqrt(1/3) or whether or not at
    // least one component is less than sqrt(1/3). Learned this trick from
    // Peter Kutz.

    glm::vec3 directionNotNormal;
    if (abs(normal.x) < SQRT_OF_ONE_THIRD)
    {
        directionNotNormal = glm::vec3(1, 0, 0);
    }
    else if (abs(normal.y) < SQRT_OF_ONE_THIRD)
    {
        directionNotNormal = glm::vec3(0, 1, 0);
    }
    else
    {
        directionNotNormal = glm::vec3(0, 0, 1);
    }

    // Use not-normal direction to generate two perpendicular directions
    glm::vec3 perpendicularDirection1 =
        glm::normalize(glm::cross(normal, directionNotNormal));
    glm::vec3 perpendicularDirection2 =
        glm::normalize(glm::cross(normal, perpendicularDirection1));

    return up * normal
        + cos(around) * over * perpendicularDirection1
        + sin(around) * over * perpendicularDirection2;
}

__host__ __device__ glm::vec3 calculateRandomSpecularDirection(
    glm::vec3 R,
    float exponent,
    thrust::default_random_engine& rng
)
{
    thrust::uniform_real_distribution<float> u01(0, 1);

    float theta = glm::acos(pow((double)(u01(rng)), (double)(1 / (exponent + 1))));
    float phi = TWO_PI * u01(rng);

    float x = cos(phi) * sin(theta);
    float y = sin(phi) * sin(theta);
    float z = cos(theta);

    glm::vec3 directionNotR;
    if (abs(R.x) < SQRT_OF_ONE_THIRD)
    {
        directionNotR = glm::vec3(1, 0, 0);
    }
    else if (abs(R.y) < SQRT_OF_ONE_THIRD)
    {
        directionNotR = glm::vec3(0, 1, 0);
    }
    else
    {
        directionNotR = glm::vec3(0, 0, 1);
    }

    glm::vec3 tangent = glm::normalize(glm::cross(R, directionNotR));
    glm::vec3 bitangent = glm::normalize(glm::cross(R, tangent));

    return x * tangent + y * bitangent + z * R;
}

__host__ __device__ void scatterRay(
    PathSegment & pathSegment,
    glm::vec3 intersect,
    glm::vec3 normal,
    const Material &m,
    thrust::default_random_engine &rng)
{
    if (m.hasReflective > 0.0f)
    {
        if (m.specular.exponent == INFINITY)
        {
            pathSegment.ray.direction = glm::reflect(pathSegment.ray.direction, normal);
            pathSegment.ray.origin = intersect + pathSegment.ray.direction * EPSILON;
        }
        else
        {
            glm::vec3 R = glm::reflect(pathSegment.ray.direction, normal);

            pathSegment.ray.direction = glm::normalize(calculateRandomSpecularDirection(R, m.specular.exponent, rng));
            pathSegment.ray.origin = intersect + pathSegment.ray.direction * EPSILON;
        }
    }
    else {
        pathSegment.ray.direction = glm::normalize(calculateRandomDirectionInHemisphere(normal, rng));
        pathSegment.ray.origin = intersect + normal * EPSILON;
    }

    pathSegment.remainingBounces--;
}
