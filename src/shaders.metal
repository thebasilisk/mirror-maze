#include <metal_stdlib>

using namespace metal;

struct ColorInOut {
    float4 position [[position]];
    float4 color;
};

struct ray {
    float3 ori;
    float3 dir;
};

struct rect {
    packed_float3 ori;
    packed_float3 v;
    packed_float3 u;
    packed_float3 color;
};

struct sphere {
    packed_float3 ori;
    float rad;
};

struct camera {
    packed_float3 camera_center;
    float focal_length;
    packed_float2 viewport;
};

/*
    Materials:
        0 - Matte
        1 - Mirror
        2 - Light Source
*/

float4 ray_rect_intersect (const ray beam, const rect mirror) {
    float3 rect_norm = cross(mirror.v, mirror.u);
    float norm_check = dot(beam.dir, rect_norm);

    float a = dot((mirror.ori - beam.ori), rect_norm) / norm_check;
    float3 intersection = beam.ori + (a * beam.dir);

    float3 rect_vect = intersection - mirror.ori;

    float d1 = dot(rect_vect, mirror.v) / length(mirror.v);
    float d2 = dot(rect_vect, mirror.u) / length(mirror.u);

    if ((0 <= d1 && d1 <= length(mirror.v)) && (0 <= d2 && d2 <= length(mirror.u)) && norm_check != 0.0) {
        return float4(intersection, a);
    } else {
        return float4(-1, -1, -1, -1);
    }
};

float4 ray_sphere_intersect (const ray beam, const sphere light) {
    float distance = dot(light.ori - beam.ori, beam.dir);
    float3 projection = beam.ori + (distance * beam.dir);
    float projection_distance = length(projection - light.ori);
    if (distance < 0) {
        return float4(-1, -1, -1, -1);
    } else if (projection_distance == light.rad) {
        return float4(projection, distance);
    } else if (projection_distance < light.rad) {
        float i_to_projection = sqrt(pow(light.rad, 2.0) + pow(projection_distance, 2.0));
        float idistance = distance - i_to_projection;
        float3 intersection = beam.ori + (idistance * beam.dir);
        return float4(intersection, idistance);
    } else {
        return float4(-1, -1, -1, -1);
    }
};

float random (float2 st) {
    return (fract(sin(dot(st.xy,
                         float2(12.9898,78.233)))*
        43758.5453123) - 0.5) * 2.0;
};

//fragment shader
//every frame, add new data to existing image
//
    //calculate ray direction and origin based on camera.
    //check intersection against all world-objects
    //apply lighting based on intersection point
    //calculate new ray direction with origin of intersection point
        //this is based on material diffuse/glossiness
    //continue for N bounces.


vertex ColorInOut vertex_shader (
    const device float3 *vert [[ buffer(0) ]],
    unsigned int vid [[ vertex_id ]]
) {
    ColorInOut out;
    // auto device const &pos = vert[vid];

    switch (vid) {
        case 0:
        out.position = float4(-1.0, 1.0, 0.0, 1.0);
        break;
        case 1:
        out.position = float4(1.0, 1.0, 0.0, 1.0);
        break;
        case 2:
        out.position = float4(-1.0, -1.0, 0.0, 1.0);
        break;
        case 3:
        out.position = float4(1.0, -1.0, 0.0, 1.0);
        break;
    }
    // out.position = float4(pos, 1.0,);
    out.color = float4 (0.0, 0.5, 0.5, 1.0);
    return out;
}

fragment float4 fragment_shader (
    ColorInOut in [[stage_in]],
    //const device unsigned long *tex_dimensions [[ buffer(0) ]],
    texture2d<float, access::sample> image [[ texture(0) ]]
) {
    float4 color = image.read(uint2(in.position.xy));
    if (color.x == 0.0 && color.y == 0.0 && color.z == 0.0) {
        color = (image.read(uint2(in.position.xy) + uint2(1,0)) + image.read(uint2(in.position.xy) + uint2(-1,0))) / 2.0;
        color = (color / 2.0) + (image.read(uint2(in.position.xy) + uint2(0,1)) + image.read(uint2(in.position.xy) + uint2(0,-1))) / 4.0;
    }
    return float4(color.xyz, 1.0);
}

fragment float4 fragment_shader_deprecated (
    ColorInOut in [[stage_in]],
    const device rect *mirrors [[ buffer(0) ]],
    const device camera *camera [[ buffer(1) ]],
    const device float *screen [[ buffer(2) ]],
    const device sphere *lights [[ buffer(3) ]],
    const device bool *materials [[ buffer(4) ]],
    texture2d<float, access::sample> noise [[ texture(0) ]]
) {

    auto device const &cam = camera[0];

    float2 in_pos_norm = float2(in.position.x / screen[0], in.position.y / screen[1]);
    float3 viewport_corner = cam.camera_center - float3(cam.viewport.x / 2.0, cam.viewport.y / 2.0, -cam.focal_length);
    float3 ray_dir = normalize((viewport_corner + float3(in_pos_norm.x * cam.viewport.x, in_pos_norm.y * cam.viewport.y, 0.0)) - cam.camera_center);

    ray beam;
    // beam.ori = cam.camera_center;
    // beam.dir = ray_dir;

    constexpr sampler s(
                    address::repeat,
                    filter::nearest);
    int hit;
    // float4 intersection = float4(0.0, 0.0, 0.0, 999.0);
    float3 color = float3(0.0, 0.0, 0.0);
    float4 noise_sample = noise.sample(s, in_pos_norm);
    int bounce_limit = 10;
    int total_samples = 10;
    int mirror_num = 13;
    float lighting_factor = 0.15;
    for (int sample = 0; sample < total_samples; sample++) {
        beam.ori = cam.camera_center;
        beam.dir = ray_dir + (float3(random(noise_sample.xy + float2(sample, -sample)), random(noise_sample.xy + float2(1 - sample, 1 + sample)), 0.0) * 0.001);
        float4 intersection = float4(0.0, 0.0, 0.0, 999.0);
        int mirror_hits = 0;
        for (int n = 0; n < bounce_limit; n++) {
            hit = 0;
            for (int i = 0; i < 12; i++) {
                float4 new_intersection = ray_rect_intersect(beam, mirrors[i]);
                if (new_intersection.w < intersection.w && new_intersection.w > 0.1) {
                    intersection = new_intersection;
                    mirror_num = i;
                    hit = 1;
                }
            }
            for (int j = 0; j < 3; j++) {
                float4 new_intersection = ray_sphere_intersect(beam, lights[j]);
                if (new_intersection.w < intersection.w && new_intersection.w > 0.1) {
                    intersection = new_intersection;
                    hit = 2;
                }
            }
            if (hit == 1) {
                float3 mirror_norm = cross(mirrors[mirror_num].v, mirrors[mirror_num].u);
                float beam_side = -sign(dot(beam.dir, mirror_norm));
                if (materials[mirror_num] == false || beam_side == -1.0) {
                    color += mirrors[mirror_num].color * pow(lighting_factor, float(n - mirror_hits));
                    // float3 random_dir = normalize((noise_sample.xyz - float3(0.5, 0.5, 0.5)) * 2.0);
                    float3 random_dir = normalize(float3(random(in_pos_norm + float2(sample,sample)), random(noise_sample.xy + float2(sample,sample)), random(noise_sample.zx + float2(sample,sample))));
                    float flip = sign(dot(random_dir, mirror_norm * beam_side));
                    random_dir = random_dir * flip;
                    beam.dir = normalize(random_dir + mirror_norm * beam_side);
                } else {
                    mirror_hits++;
                    color += mirrors[mirror_num].color * 0.05;
                    beam.dir = normalize(reflect(beam.dir, mirror_norm));
                }
                beam.ori = intersection.xyz;

            } else if (hit == 2) {
                color = float3(10.0, 10.0, 10.0);
                break;
            } else {
                color += float3(0.3, 0.6, 0.8) * pow(lighting_factor, float(n - mirror_hits));
                color *= 0.5;
                break;
            }
        }
    }
    color = color / float(total_samples);
    return float4(sqrt(color.x), sqrt(color.y), sqrt(color.z), 1.0);
}


kernel void compute_shader (
    texture2d<float, access::write> texout [[ texture(0) ]],
    texture2d<float, access::sample> noise [[ texture(1) ]],
    const device uint2 *pixel_update_buffer [[ buffer(0) ]],
    const device rect *mirrors [[ buffer(1) ]],
    const device camera *camera [[ buffer(2) ]],
    const device bool *materials [[ buffer(3) ]],
    //threadgroup atomic_int *shared [[ threadgroup(0) ]],
    uint2 tgid [[ threadgroup_position_in_grid ]],
    uint2 gid [[ thread_position_in_threadgroup ]],
    uint2 texid [[ thread_position_in_grid ]]
) {
    //96 is constant for view_height divided by 8
    //Double check this code, maybe supposed to be view_width / 8
    uint pixel_buffer_index = tgid.x + tgid.y * 32;
    uint2 pixel = pixel_update_buffer[pixel_buffer_index];

    uint flat_index = gid.x + 16 * gid.y;
    uint pixel_number = flat_index / 224;
    uint pixel_y_add = pixel_number % 2;
    uint pixel_x_add = pixel_number / 2;
    pixel = pixel + uint2(pixel_x_add, pixel_y_add);

    auto device const &cam = camera[0];

    //threadgroup atomic_int &pixel_red = shared[0];
    //threadgroup atomic_int &pixel_green = shared[1];
    //threadgroup atomic_int &pixel_blue = shared[2];

    //atomic_store_explicit(&pixel_red, 0, memory_order_relaxed);
    //atomic_store_explicit(&pixel_green, 0, memory_order_relaxed);
    //atomic_store_explicit(&pixel_blue, 0, memory_order_relaxed);

    //threadgroup float3 pixel_colors[32*32];

    float2 pos_norm = float2(pixel.x / 512.0, pixel.y / 384.0);
    float3 viewport_corner = cam.camera_center - float3(cam.viewport.x / 2.0, cam.viewport.y / 2.0, -cam.focal_length);
    float3 ray_dir = normalize((viewport_corner + float3(pos_norm.x * cam.viewport.x, pos_norm.y * cam.viewport.y, 0.0)) - cam.camera_center);

    ray beam;

    constexpr sampler s(address::repeat, filter::nearest);
    int hit;
    float3 color = float3(0.0, 0.0, 0.0);
    float4 noise_sample = noise.sample(s, float2(gid));
    int bounce_limit = 10;
    int mirror_num = 13;
    float lighting_factor = 0.15;

    beam.ori = cam.camera_center;
    beam.dir = ray_dir + (float3(random(noise_sample.xy + float2(gid.xy)), random(noise_sample.xy + float2(gid.yx)), 0.0) * 0.001);
    float4 intersection = float4(0.0, 0.0, 0.0, 999.0);
    int mirror_hits = 0;
    for (int n = 0; n < bounce_limit; n++) {
        hit = 0;
        for (int i = 0; i < 12; i++) {
            float4 new_intersection = ray_rect_intersect(beam, mirrors[i]);
            if (new_intersection.w < intersection.w && new_intersection.w > 0.1) {
                intersection = new_intersection;
                mirror_num = i;
                hit = 1;
            }
        }

        if (hit == 1) {
            float3 mirror_norm = cross(mirrors[mirror_num].v, mirrors[mirror_num].u);
            float beam_side = -sign(dot(beam.dir, mirror_norm));
            if (materials[mirror_num] == false || beam_side == -1.0) {
                color += mirrors[mirror_num].color * pow(lighting_factor, float(n - mirror_hits));
                // float3 random_dir = normalize((noise_sample.xyz - float3(0.5, 0.5, 0.5)) * 2.0);
                float3 random_dir = normalize(float3(random(pos_norm + float2(gid.x + n, gid.y + n)), random(noise_sample.xy + float2(gid.x + n, gid.y + n)), random(noise_sample.zx + float2(gid.x + n, gid.y + n))));
                float flip = sign(dot(random_dir, mirror_norm * beam_side));
                random_dir = random_dir * flip;
                beam.dir = normalize(random_dir + mirror_norm * beam_side);
            } else {
                mirror_hits++;
                color += mirrors[mirror_num].color * 0.05;
                beam.dir = normalize(reflect(beam.dir, mirror_norm));
            }
            beam.ori = intersection.xyz;
        } else {
            color += float3(0.3, 0.6, 0.8) * pow(lighting_factor, float(n - mirror_hits));
            color *= 0.5;
            break;
        }
    }
    //int3 write_color = int3(color * (32 * 32));
    //atomic_fetch_add_explicit(&pixel_red, write_color.x, memory_order_relaxed);
    //atomic_fetch_add_explicit(&pixel_green, write_color.y, memory_order_relaxed);
    //atomic_fetch_add_explicit(&pixel_blue, write_color.z, memory_order_relaxed);
    const int max_index = 16 * 56 / 4;
    //int flat_index = gid.x + gid.y * 16;
    threadgroup float3 test[max_index];
    test[flat_index] = color;
    threadgroup_barrier(mem_flags::mem_none);
    if (flat_index < (max_index * pixel_number) + (max_index / 2)) {
        test[flat_index] = (test[flat_index] + test[(max_index * (pixel_number + 1)) - flat_index]) / 2.0;
    }
    threadgroup_barrier(mem_flags::mem_none);
    if (flat_index < (max_index * pixel_number) + (max_index / 4)) {
        test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 2) - flat_index]) / 2.0;
    }
    threadgroup_barrier(mem_flags::mem_none);
    if (flat_index < max_index / 8) {
        test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 4) - flat_index]) / 2.0;
    }
    threadgroup_barrier(mem_flags::mem_none);
    if (flat_index < max_index / 16) {
        test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 8) - flat_index]) / 2.0;
    }
    threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < max_index / 32) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 16) - flat_index]) / 2.0;
    //}
    //threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < max_index / 64) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 32) - flat_index]) / 2.0;
    //}
    //threadgroup_barrier(mem_flags::mem_none);
    if (flat_index == pixel_number * max_index) {
        for (int i = 1; i < max_index / 16; i++) {
            test[pixel_number * max_index] = (test[pixel_number * max_index] + test[pixel_number * max_index + i]) / 2.0;
        }
        texout.write(float4(test[pixel_number * max_index], 1.0), pixel);
    }
}
