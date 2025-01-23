#include <metal_stdlib>

using namespace metal;

struct ColorInOut {
    float4 position [[position]];
    float4 color;
};

struct ray {
    float3 ori;
    float3 dir;
    float t = 1e30f;
    uint index;
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
struct bvh_node {
    packed_float3 aabb_min;
    packed_float3 aabb_max;
    uint left_first;
    uint plane_count;
};

struct camera {
    packed_float3 camera_center;
    float focal_length;
    packed_float4 rotation;
    packed_float2 viewport;
};

/*
    Materials:
        0 - Matte
        1 - Mirror
        2 - Light Source
*/

void ray_rect_intersect (thread ray& beam, const rect mirror, const int index) {
    float3 rect_norm = normalize(cross(mirror.v, mirror.u));
    float norm_check = dot(beam.dir, rect_norm);

    float a = dot((mirror.ori - beam.ori), rect_norm) / norm_check;
    float3 intersection = beam.ori + (a * beam.dir);

    float3 rect_vect = intersection - mirror.ori;

    float d1 = dot(rect_vect, mirror.v) / length(mirror.v);
    float d2 = dot(rect_vect, mirror.u) / length(mirror.u);

    if ((0 <= d1 && d1 <= length(mirror.v)) && (0 <= d2 && d2 <= length(mirror.u)) && norm_check != 0.0 && a > 0.001 && a < beam.t) {
        beam.t = a;
        beam.index = index;
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

float intersect_aabb(thread ray& beam, const float3 bmin, const float3 bmax) {
    float tx1 = (bmin.x - beam.ori.x) / beam.dir.x, tx2 = (bmax.x - beam.ori.x) / beam.dir.x;
    float tmin = min( tx1, tx2 ), tmax = max( tx1, tx2 );
    float ty1 = (bmin.y - beam.ori.y) / beam.dir.y, ty2 = (bmax.y - beam.ori.y) / beam.dir.y;
    tmin = max( tmin, min( ty1, ty2 ) ), tmax = min( tmax, max( ty1, ty2 ) );
    float tz1 = (bmin.z - beam.ori.z) / beam.dir.z, tz2 = (bmax.z - beam.ori.z) / beam.dir.z;
    tmin = max( tmin, min( tz1, tz2 ) ), tmax = min( tmax, max( tz1, tz2 ) );
    if (tmax >= tmin && tmin < beam.t && tmax > 0.0) return tmin; else return 1e30f;
}

void intersect_bvh (
    thread ray& beam,
    const device bvh_node* nodes,
    const device rect* mirrors,
    const device uint* mirror_indices,
    const uint node_index
) {
    const device bvh_node &node = nodes[node_index];

    if (!intersect_aabb(beam, node.aabb_min, node.aabb_max)) return;
    if (node.plane_count == 1) {
        ray_rect_intersect(beam, mirrors[mirror_indices[node.left_first]], mirror_indices[node.left_first]);
    } else {
        intersect_bvh(beam, nodes, mirrors, mirror_indices, node.left_first);
        intersect_bvh(beam, nodes, mirrors, mirror_indices, node.left_first + 1);
    }
}

void intersect_bvh_iterative(
    thread ray& beam,
    const device bvh_node* nodes,
    const device rect* mirrors,
    const device uint* mirror_indices
) {
    const device bvh_node* node = &nodes[0];

    const device bvh_node* stack[50];
    uint head = 0;

    while (true) {
        if (node->plane_count > 0) {
            for (uint i = 0; i < node->plane_count; i++) ray_rect_intersect(beam, mirrors[mirror_indices[node->left_first + i]], mirror_indices[node->left_first + i]);
            if (head == 0) break; else node = stack[--head];

            continue;
        }

        const device bvh_node* left_node = &nodes[node->left_first];
        const device bvh_node* right_node = &nodes[node->left_first + 1];

        float dist1 = intersect_aabb(beam, left_node->aabb_min, left_node->aabb_max);
        float dist2 = intersect_aabb(beam, right_node->aabb_min, right_node->aabb_max);

        if (dist1 > dist2) {
            float temp = dist1;
            dist1 = dist2;
            dist2 = temp;

            const device bvh_node* nemp = left_node;
            left_node = right_node;
            right_node = nemp;
        }
        if (dist1 == 1e30f) {
            if (head == 0) break; else node = stack[--head];
        } else {
            node = left_node;
            if (dist2 != 1e30f) stack[head++] = right_node;
        }
    }
}


float4 quat_inv(const float4 quat) {
    return float4(-quat.xyz, quat.w);
}

float4 quat_dot (const float4 q1, const float4 q2) {
    float s = q1.w * q2.w - dot(q1.xyz, q2.xyz);
    float3 v = cross(q1.xyz, q2.xyz) + q1.w * q2.xyz + q2.w * q1.xyz;
    return float4(v, s);
}

float3 quat_mult (const float3 vec, const float4 quat) {
    float4 r = quat_dot(quat_dot(quat_inv(quat), float4(vec.xyz, 0)), quat);
    return r.xyz;
}


float random (float2 st) {
    return (fract(sin(dot(st.xy,
                         float2(12.9898,78.233)))*
        43758.5453123) - 0.5) * 2.0;
};


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


kernel void compute_shader (
    texture2d<float, access::write> texout [[ texture(0) ]],
    texture2d<float, access::sample> noise [[ texture(1) ]],
    const device uint2 *pixel_update_buffer [[ buffer(0) ]],
    const device rect *mirrors [[ buffer(1) ]],
    const device bvh_node *nodes [[ buffer(2) ]],
    const device uint *indices [[ buffer(3) ]],
    const device camera *camera [[ buffer(4) ]],
    const device bool *materials [[ buffer(5) ]],
    uint2 tgid [[ threadgroup_position_in_grid ]],
    uint2 gid [[ thread_position_in_threadgroup ]],
    uint2 texid [[ thread_position_in_grid ]],
    uint2 dimensions [[ threads_per_threadgroup ]]
) {
    //tgid.y * grid width
    uint pixel_buffer_index = tgid.x + tgid.y * ((1024 / 2) / 16);
    uint2 pixel = pixel_update_buffer[pixel_buffer_index];

    uint total_threads = dimensions.x * dimensions.y;

    uint flat_index = gid.x + dimensions.x * gid.y;
    uint pixel_number = flat_index / (total_threads / 16);
    uint pixel_y_add = pixel_number % 4;
    uint pixel_x_add = pixel_number / 4;
    pixel = pixel + uint2(pixel_x_add, pixel_y_add);

    auto device const &cam = camera[0];

    //threadgroup float3 pixel_colors[32*32];

    float2 pos_norm = float2(pixel.x / 1024.0, pixel.y / 768.0);
    float3 viewport_corner = cam.camera_center - float3(cam.viewport.x / 2.0, cam.viewport.y / 2.0, -cam.focal_length);
    float3 ray_dir = normalize((viewport_corner + float3(pos_norm.x * cam.viewport.x, pos_norm.y * cam.viewport.y, 0.0)) - cam.camera_center);
    ray_dir = quat_mult(ray_dir, cam.rotation);

    ray beam;

    constexpr sampler s(address::repeat, filter::nearest);
    float3 color = float3(0.0, 0.0, 0.0);
    float4 noise_sample = noise.sample(s, float2(gid));
    int bounce_limit = 10;
    float lighting_factor = 0.25;

    beam.ori = cam.camera_center;
    beam.dir = ray_dir + (float3(random(noise_sample.xy + float2(gid.xy)), random(noise_sample.xy + float2(gid.yx)), 0.0) * 0.001);
    beam.t = 1e30f;
    int mirror_hits = 0;
    for (int n = 0; n < bounce_limit; n++) {
        intersect_bvh_iterative(beam, nodes, mirrors, indices);
        if (beam.t < 1e30f) {
            float3 mirror_norm = normalize(cross(mirrors[beam.index].v, mirrors[beam.index].u));
            float beam_side = -sign(dot(beam.dir, mirror_norm));
            if (materials[beam.index] == false || beam_side == -1.0) {
                color += mirrors[beam.index].color * pow(lighting_factor, float(n - mirror_hits));
                float3 random_dir = normalize(float3(random(pos_norm + float2(gid.x + n, gid.y + n)), random(noise_sample.xy + float2(gid.x + n, gid.y + n)), random(noise_sample.zx + float2(gid.x + n, gid.y + n))));
                float flip = sign(dot(random_dir, mirror_norm * beam_side));
                random_dir = random_dir * flip;
                beam.ori = beam.ori + beam.dir * beam.t;
                beam.dir = normalize(random_dir + mirror_norm * beam_side);
                beam.t = 1e30f;
            } else {
                mirror_hits++;
                color += mirrors[beam.index].color * pow(lighting_factor, float(n - mirror_hits)) * 0.05;
                beam.ori = beam.ori + beam.dir * beam.t;
                beam.dir = normalize(reflect(beam.dir, mirror_norm));
                beam.t = 1e30f;
            }
        } else {
            color += float3(0.3, 0.6, 0.8) * pow(lighting_factor, float(n - mirror_hits));
            break;
        }
    }
    //texout.write(float4(color, 1.0), pixel);
    const int max_index = 16 * 56 / 16;
    threadgroup float3 test[max_index];
    test[flat_index] = color;
    threadgroup_barrier(mem_flags::mem_none);

    if (flat_index % 2 == 0) {
        test[flat_index] += test[flat_index + 1];
    }
    threadgroup_barrier(mem_flags::mem_none);
    if (flat_index % 4 == 0) {
        test[flat_index] += test[flat_index + 2];
    }
    threadgroup_barrier(mem_flags::mem_none);
    if (flat_index % 8 == 0) {
        test[flat_index] += test[flat_index + 4];
    }
    threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < (max_index * pixel_number) + (max_index / 2)) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * (pixel_number + 1)) - flat_index - 1]);
    //}
    //threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < (max_index * pixel_number) + (max_index / 4)) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 2) - flat_index]);
    //}
    //threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < max_index / 8) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 4) - flat_index]);
    //}
    //threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < max_index / 16) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 8) - flat_index]);
    //}
    //threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < max_index / 32) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 16) - flat_index]);
    //}
    threadgroup_barrier(mem_flags::mem_none);
    //if (flat_index < max_index / 64) {
    //    test[flat_index] = (test[flat_index] + test[(max_index * pixel_number) + (max_index / 32) - flat_index]);
    //}
    //threadgroup_barrier(mem_flags::mem_none);

    if (flat_index == pixel_number * max_index) {
        for (int i = 1; i < max_index / 8; i++) {
            test[pixel_number * max_index] += test[pixel_number * max_index + (8 * i)];
        }
        test[pixel_number * max_index] = test[pixel_number * max_index] / max_index;
        texout.write(float4(test[pixel_number * max_index], 1.0), pixel);
    }
}
