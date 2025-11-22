
#ifndef UTILS
#define UTILS

#include "parser.h"
#include <cmath>

using namespace parser;
using namespace std;

inline float dotProduct(Vec3f vec1, Vec3f vec2)
{
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;    
}

inline Vec3f vector_multiplication(Vec3f vec1, Vec3f vec2) {
    return {vec1.x * vec2.x, vec1.y * vec2.y, vec1.z * vec2.z};  
}

inline Vec3f crossProduct(Vec3f vec1, Vec3f vec2)
{
    Vec3f result;
    result.x = vec1.y*vec2.z - vec1.z*vec2.y;
    result.y = -(vec1.x*vec2.z - vec1.z*vec2.x);
    result.z = vec1.x*vec2.y - vec1.y*vec2.x;
    return result;
}

inline Vec3f add_vectors(Vec3f vec1, Vec3f vec2) {
    return {vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z};
}

inline Vec3f scale_vector(Vec3f vec, float d) {
    return {vec.x * d, vec.y * d, vec.z * d};
}

inline float vectorMagnitude(Vec3f vec) {
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

inline Vec3f clamp(Vec3f vec) {
    return {max((float)0, min((float)255, vec.x)),
    max((float)0, min((float)255, vec.y)),
    max((float)0, min((float)255, vec.z))};
}

inline float determinant(Vec3f vec1, Vec3f vec2, Vec3f vec3) {
    float factor1 = vec1.x * (vec2.y * vec3.z - vec3.y * vec2.z);
    float factor2 = vec2.x * (vec1.y * vec3.z - vec3.y * vec1.z);
    float factor3 = vec3.x * (vec1.y * vec2.z - vec2.y * vec1.z);

    return factor1 - factor2 + factor3;
}

inline float euclidean_distance_square(Vec3f vec1, Vec3f vec2) {
    return (pow(vec1.x - vec2.x, 2) + pow(vec1.y - vec2.y, 2) + pow(vec1.z - vec2.z, 2));
}

inline Vec3f normalize(Vec3f vec) {
    return scale_vector(vec, 1/vectorMagnitude(vec));
}

#endif