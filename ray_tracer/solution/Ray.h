#ifndef RAY
#define RAY

#include "parser.h"
#include "utils.h"
#include <cfloat>

using namespace std;

struct Hit_record {
    float t=FLT_MAX;
    Vec3f p; // intersection point
    bool is_null=true;
    int material_id;
    Vec3f normal;
};

class Ray {
private:
///
public:
    Vec3f o;
    Vec3f d;
    int depth;
    
    Ray(Vec3f p1, Vec3f p2) : o(p1), d(add_vectors(p2,-p1)) {
        depth = 0;
    }

    Ray(Vec3f o, Vec3f d, bool garbage) : o(o), d(d) {
        depth = 0;
    }

    Vec3f get_point(float t) {
        return add_vectors(o, scale_vector(d, t));
    }

    void intersect_with_sphere(Hit_record &hit_record, Sphere &sphere, Scene &scene) {
        Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];
        Vec3f o_c = add_vectors(o, -center);
        float dd = dotProduct(d,d);
        float d_oc = dotProduct(d, o_c);
        float delta = pow(d_oc, 2) - dd * (dotProduct(o_c, o_c) - sphere.radius*sphere.radius);

        if(delta < 0) {
            return;
        }

        hit_record.is_null = false;

        float sqrt_delta = sqrt(delta);
        float t1 = (-d_oc + sqrt_delta) / dd;
        float t2 = (-d_oc - sqrt_delta) / dd;

        hit_record.t = min(t1, t2);
        hit_record.p = get_point(hit_record.t);
        Vec3f non_normalized_normal = add_vectors(hit_record.p, -center);
        hit_record.normal = scale_vector(non_normalized_normal, 1/vectorMagnitude(non_normalized_normal));
    }

    void intersect_with_triangle(Hit_record &hit_record, Face &face, Scene &scene) {
        float product = dotProduct(d, face.normal);
        // check >=
        if(product >= 0) {
            return;
        }

        Vec3f a = scene.vertex_data[face.v0_id-1];
        Vec3f b = scene.vertex_data[face.v1_id-1];
        Vec3f c = scene.vertex_data[face.v2_id-1];

        Vec3f a_minus_o = add_vectors(a, -o);
        Vec3f a_minus_b = add_vectors(a, -b);
        Vec3f a_minus_c = add_vectors(a, -c);

        float determinant_A = determinant(a_minus_b, a_minus_c, d);
        float t = determinant(a_minus_b, a_minus_c, a_minus_o) / determinant_A;

        // check <=
        if(t + scene.shadow_ray_epsilon < 0) {
            return;
        }

        float beta = determinant(a_minus_o, a_minus_c, d) / determinant_A;

        if(beta + scene.shadow_ray_epsilon < 0) {
            return;
        }

        float gamma = determinant(a_minus_b, a_minus_o, d) / determinant_A;

        if((gamma + scene.shadow_ray_epsilon < 0) || ((gamma + beta) > (1 + scene.shadow_ray_epsilon))) {
            return;
        }

        hit_record.t = t;
        hit_record.is_null = false;
        hit_record.p = get_point(t);
        hit_record.normal = face.normal;

    }

    // check: we may need to check t_min for multiple faces for one mesh
    // void intersect_with_mesh(Hit_record &hit_record, Mesh &mesh, Scene &scene) {
    //     int faces_size = mesh.faces.size();
    //     for(int i=0; i<faces_size; i++) {
    //         intersect_with_triangle(hit_record, mesh.faces[i], scene);
    //     }
    // }
};

#endif