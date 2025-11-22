#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "Ray.h"
#include <chrono>

using namespace std;
using namespace parser;

Vec3f compute_color(Ray &r, Scene &scene);
bool closestHit(Ray &r, Hit_record &hit_record, Scene &scene);
Vec3f apply_shading(Ray &r, Hit_record &hit_record, Scene &scene);
bool is_in_shadow(Ray &light_ray, Scene &scene);
Vec3f diffuse_shading(Ray &light_ray, PointLight &light, Hit_record &hit_record, Scene &scene);
Vec3f specular_shading(Ray &light_ray, Ray &cam_ray, PointLight &light, Hit_record &hit_record, Scene &scene);

typedef unsigned char RGB[3];

int main(int argc, char *argv[])
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // Sample usage for reading an XML scene file
    parser::Scene scene;

    cout << argv[1];

    scene.loadFromXml(argv[1]);

    int number_of_cameras = scene.cameras.size();
    for (int camera_no = 0; camera_no < number_of_cameras; camera_no++)
    {
        // this outer loop is for each camera
        // generate one image for each camera
        Vec3f camera_pos = scene.cameras[camera_no].position;
        Vec3f camera_w = -scene.cameras[camera_no].gaze; // looking off the image plane
        Vec3f camera_v = scene.cameras[camera_no].up;
        Vec3f camera_u = crossProduct(camera_v, camera_w);
        int image_height = scene.cameras[camera_no].image_height;
        int image_width = scene.cameras[camera_no].image_width;
        Vec4f near_plane = scene.cameras[camera_no].near_plane;
        float near_distance = scene.cameras[camera_no].near_distance;
        string image_name = scene.cameras[camera_no].image_name;

        float plane_l = near_plane.x;
        float plane_r = near_plane.y;
        float plane_b = near_plane.z;
        float plane_t = near_plane.w;

        Vec3f camera_w_minus = -camera_w;
        Vec3f plane_m = add_vectors(camera_pos, scale_vector(camera_w_minus, near_distance));
        Vec3f plane_q = add_vectors(plane_m, add_vectors(scale_vector(camera_u, plane_l), scale_vector(camera_v, plane_t)));

        unsigned char *image = new unsigned char[image_width * image_height * 3];

        int image_index = 0;
        for (int j = 0; j < image_height; j++)
        {
            float s_v = (j + 0.5) * (plane_t - plane_b) / (image_height);
            for (int i = 0; i < image_width; i++)
            {
                float s_u = (i + 0.5) * (plane_r - plane_l) / (image_width);

                Vec3f plane_s = add_vectors(plane_q, add_vectors(scale_vector(camera_u, s_u), -scale_vector(camera_v, s_v)));
                Ray r(camera_pos, plane_s);

                Vec3f float_color = clamp(compute_color(r, scene));

                Vec3i color = {
                    static_cast<int>(std::round(float_color.x)),
                    static_cast<int>(std::round(float_color.y)),
                    static_cast<int>(std::round(float_color.z))
                };

                image[image_index++] = color.x;
                image[image_index++] = color.y;
                image[image_index++] = color.z;
            }
        }

        write_ppm(image_name.c_str(), image, image_width, image_height);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    cout << "duration: " << duration.count() << " seconds" <<endl;;
}

Vec3f compute_color(Ray &r, Scene &scene)
{
    Hit_record hit_record;

    if (r.depth > scene.max_recursion_depth)
    {
        return {0, 0, 0};
    }

    if (closestHit(r, hit_record, scene))
    {
        Vec3f color = apply_shading(r, hit_record, scene);

        return color;
    }

    else if (r.depth == 0)
    {
        return {(float)scene.background_color.x, (float)scene.background_color.y, (float)scene.background_color.z};
    }

    else
    {
        return {0, 0, 0};
    }
}

bool closestHit(Ray &r, Hit_record &hit_record, Scene &scene)
{
    int meshes_size = scene.meshes.size();
    int spheres_size = scene.spheres.size();
    int triangles_size = scene.triangles.size();

    float t_min = FLT_MAX;

    // optimize hit_record by not defining repetitively
    for (int i = 0; i < meshes_size; i++)
    {
        int faces_size = scene.meshes[i].faces.size();
        for(int j=0; j<faces_size; j++) {
            Hit_record local_hit_record;

            r.intersect_with_triangle(local_hit_record, scene.meshes[i].faces[j], scene);
            if (local_hit_record.is_null || local_hit_record.t <= 0)
            {
                continue;
            }

            if (local_hit_record.t < t_min)
            {
                t_min = local_hit_record.t;
                local_hit_record.material_id = scene.meshes[i].material_id;

                hit_record = local_hit_record;
            }
        }
    }

    for (int i = 0; i < spheres_size; i++)
    {
        Hit_record local_hit_record;
        r.intersect_with_sphere(local_hit_record, scene.spheres[i], scene);

        if (local_hit_record.is_null || local_hit_record.t <= 0)
        {
            continue;
        }

        if (local_hit_record.t < t_min)
        {
            t_min = local_hit_record.t;
            local_hit_record.material_id = scene.spheres[i].material_id;

            hit_record = local_hit_record;
        }
    }

    for (int i = 0; i < triangles_size; i++)
    {
        Hit_record local_hit_record;
        r.intersect_with_triangle(local_hit_record, scene.triangles[i].indices, scene);

        if (local_hit_record.is_null || local_hit_record.t <= 0)
        {
            continue;
        }

        if (local_hit_record.t < t_min)
        {
            t_min = local_hit_record.t;
            local_hit_record.material_id = scene.triangles[i].material_id;

            hit_record = local_hit_record;
        }
    }

    return !hit_record.is_null;
}

Vec3f apply_shading(Ray &r, Hit_record &hit_record, Scene &scene)
{
    Material mat = scene.materials[hit_record.material_id - 1];
    Vec3f color = vector_multiplication(scene.ambient_light, mat.ambient);

    if (mat.is_mirror)
    {
        Vec3f minus_cam_dir = -r.d;
        Vec3f w_o = normalize(minus_cam_dir);
        //check
        Vec3f w_r = add_vectors(-w_o, scale_vector(hit_record.normal, 2*dotProduct(hit_record.normal, w_o)));

        Ray reflection_ray(hit_record.p, w_r, false);
        reflection_ray.depth = r.depth + 1;

        color = add_vectors(color, vector_multiplication(compute_color(reflection_ray, scene), mat.mirror));
    }

    int lights_size = scene.point_lights.size();
    for (int i = 0; i < lights_size; i++)
    {
        PointLight light = scene.point_lights[i];
        Vec3f light_position = light.position;
        Vec3f intersect_point = hit_record.p;

        Vec3f new_origin = add_vectors(intersect_point, scale_vector(hit_record.normal, scene.shadow_ray_epsilon));

        Ray light_ray(new_origin, light_position);
        if(!is_in_shadow(light_ray, scene)) {
            Vec3f diffuse_term = diffuse_shading(light_ray, light, hit_record, scene);
            Vec3f specular_term = specular_shading(light_ray, r, light, hit_record, scene);
            color = add_vectors(color, add_vectors(diffuse_term, specular_term));

       }
    }
    return color;

}

bool is_in_shadow(Ray &light_ray, Scene &scene)
{
    int meshes_size = scene.meshes.size();
    int spheres_size = scene.spheres.size();
    int triangles_size = scene.triangles.size();

    // optimize hit_record by not defining repetitively
    for (int i = 0; i < meshes_size; i++)
    {
        int faces_size = scene.meshes[i].faces.size();
        for(int j=0; j<faces_size; j++) {
            Hit_record local_hit_record;

            light_ray.intersect_with_triangle(local_hit_record, scene.meshes[i].faces[j], scene);
            //check: >0  <1
            if (!local_hit_record.is_null && local_hit_record.t > 0 && local_hit_record.t < 1)
            {
                return true;
            }
        }
    }

    for (int i = 0; i < spheres_size; i++)
    {
        Hit_record local_hit_record;
        light_ray.intersect_with_sphere(local_hit_record, scene.spheres[i], scene);

        if (!local_hit_record.is_null && local_hit_record.t > 0 && local_hit_record.t < 1)
        {
            return true;
        }
    }

    for (int i = 0; i < triangles_size; i++)
    {
        Hit_record local_hit_record;
        light_ray.intersect_with_triangle(local_hit_record, scene.triangles[i].indices, scene);

        if (!local_hit_record.is_null && local_hit_record.t > 0 && local_hit_record.t < 1)
        {
            return true;
        }
    }

    return false;
}

Vec3f diffuse_shading(Ray &light_ray, PointLight &light, Hit_record &hit_record, Scene &scene) {
    Material mat = scene.materials[hit_record.material_id - 1];
    Vec3f unit_light_dir = scale_vector(light_ray.d, 1/vectorMagnitude(light_ray.d));
    float cos_theta = max((float)0, dotProduct(unit_light_dir, hit_record.normal));
    float distance_2 = euclidean_distance_square(light.position, hit_record.p);

    Vec3f I_r_2 = scale_vector(light.intensity, 1/distance_2);
    Vec3f diffuse_term = vector_multiplication(scale_vector(I_r_2, cos_theta), mat.diffuse);

    return diffuse_term;
}

Vec3f specular_shading(Ray &light_ray, Ray &cam_ray, PointLight &light, Hit_record &hit_record, Scene &scene) {
    Vec3f unit_light_dir = scale_vector(light_ray.d, 1/vectorMagnitude(light_ray.d));
    float product = dotProduct(unit_light_dir, hit_record.normal);
    
    if(product <= 0) {
        return {0,0,0};
    }

    Material mat = scene.materials[hit_record.material_id - 1];
    Vec3f minus_cam_dir = -cam_ray.d;
    Vec3f w_o = scale_vector(minus_cam_dir, 1/vectorMagnitude(minus_cam_dir));
    Vec3f w_i = unit_light_dir;
    Vec3f w_o_w_i = add_vectors(w_i, w_o);
    Vec3f half_vector = scale_vector(w_o_w_i, 1/vectorMagnitude(w_o_w_i));

    float cos_alpha = max((float)0, dotProduct(hit_record.normal, half_vector));
    float cos_with_phong = pow(cos_alpha, mat.phong_exponent);

    float distance_2 = euclidean_distance_square(light.position, hit_record.p);
    Vec3f I_r_2 = scale_vector(light.intensity, 1/distance_2);
    Vec3f specular_term = vector_multiplication(scale_vector(I_r_2, cos_with_phong), mat.specular);
    
    return specular_term;
}