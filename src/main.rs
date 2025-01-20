mod utils;
mod maths;

use std::{ffi::c_float, mem};

use rand::{random, seq::SliceRandom, thread_rng};
use objc::rc::autoreleasepool;
use objc2_app_kit::{NSAnyEventMask, NSApp, NSApplication, NSApplicationActivationPolicy, NSBitmapImageRep, NSEventType, NSImage, NSWindowStyleMask};
use objc2_foundation::{MainThreadMarker, NSComparisonResult, NSDate, NSDefaultRunLoopMode};
use utils::{copy_to_buf, get_library, get_next_frame, init_nsstring, initialize_window, make_buf, new_metal_layer, new_render_pass_descriptor, prepare_pipeline_state, set_window_layer};
use maths::{calculate_quaternion, float3_add, float3_subtract, scale3, update_quat_angle, Float2, Float3, Float4};

use metal::*;

#[repr(C)]
struct Camera {
    camera_center : Float3,
    focal_length : c_float,
    rotation : Float4,
    viewport : Float2
}

#[repr(C)]
#[derive(Clone, Debug)]
struct Plane {
    origin : Float3,
    v : Float3,
    u : Float3,
    color : Float3
}

impl Plane {
    fn new (origin : Float3, side1 : Float3, side2 : Float3, color : Float3) -> Self {
        Plane {
            origin,
            v : side1,
            u : side2,
            color
        }
    }
    fn get_center(&self) -> Float3 {
        float3_add(self.origin, scale3(float3_add(self.u, self.v), 0.5))
    }
}

#[derive(Debug, Clone)]
#[repr(C)]
struct BVHNode {
    aabb_min : Float3,
    aabb_max : Float3,
    left_first : u32,
    tri_count : u32,
}
impl BVHNode {
    fn new(left_first : u32, tri_count : u32) -> Self {
        BVHNode { aabb_min: Float3::single(1e30f32), aabb_max: Float3::single(-1e30f32), left_first, tri_count}
    }
    fn update_node_bounds(&mut self, planes : &Vec<Plane>, plane_indices : &Vec<usize>) {
        for i in self.left_first..(self.left_first + self.tri_count) {
            let plane = &planes[plane_indices[i as usize]];
            self.aabb_min = self.aabb_min.fminf(plane.origin);
            self.aabb_max = self.aabb_max.fmaxf(plane.origin);
            self.aabb_min = self.aabb_min.fminf(float3_add(plane.origin, plane.u));
            self.aabb_max = self.aabb_max.fmaxf(float3_add(plane.origin, plane.u));
            self.aabb_min = self.aabb_min.fminf(float3_add(plane.origin, plane.v));
            self.aabb_max = self.aabb_max.fmaxf(float3_add(plane.origin, plane.v));
        }
    }
    fn subdivide (&mut self, planes : &Vec<Plane>, plane_centers : &Vec<Float3>, plane_indices : &mut Vec<usize>, nodes : &mut Vec<BVHNode>) {
        // println!("Iter count: {}", nodes_used);
        if self.tri_count == 1 {
            return;
        }
        let diag = float3_subtract(self.aabb_max, self.aabb_min);
        let mut axis = 0;
        if diag.1 > diag.0 {
            axis = 1;
        }
        if diag.2 > diag.1.max(diag.0) {
            axis = 2;
        }
        let split_dist = diag.2.max(diag.1.max(diag.0)) / 2.0;
        let split_pos = match axis {
            0 => split_dist + self.aabb_min.0,
            1 => split_dist + self.aabb_min.1,
            2 => split_dist + self.aabb_min.2,
            _ => panic!(),
        };
        // println!("{:?}", self.aabb_min);
        // println!("{:?}", self.aabb_max);
        // println!("{}", self.tri_count);
        let mut i = self.left_first as i32;
        let mut j = i + self.tri_count as i32 - 1;

        while i <= j {
            let plane_center = plane_centers[plane_indices[i as usize]];
            let axis_pos = match axis {
                0 => plane_center.0,
                1 => plane_center.1,
                2 => plane_center.2,
                _ => panic!(),
            };
            // println!("i: {i}, j: {j}");
            // print!("Compare: {}   ", split_pos);
            // println!("{}", axis_pos);
            if axis_pos < split_pos {
                i += 1;
            } else {
                // println!("swap!");
                plane_indices.swap(i as usize, j as usize);
                j -= 1;
            }
        }
        let left_count = i as u32 - self.left_first;
        if left_count == 0 || left_count == self.tri_count {
            return;
        }
        let mut new_left_child = BVHNode::new(self.left_first, left_count);
        new_left_child.update_node_bounds(planes, &plane_indices);
        self.left_first = nodes.len() as u32;
        nodes.push(new_left_child.clone());
        let mut new_right_child = BVHNode::new(i as u32, self.tri_count - left_count);
        new_right_child.update_node_bounds(planes, &plane_indices);
        nodes.push(new_right_child.clone());

        new_left_child.subdivide(planes, plane_centers, plane_indices, nodes);
        new_right_child.subdivide(planes, plane_centers, plane_indices, nodes);
        let _ = std::mem::replace(&mut nodes[self.left_first as usize], new_left_child);
        let _ = std::mem::replace(&mut nodes[self.left_first as usize + 1], new_right_child);

        // println!("should go zero");
        self.tri_count = 0;
        // println!("{}", self.left_first);
        // println!("{}", self.tri_count);


    }
}

fn build_bvh (n : usize, planes : Vec<Plane>) -> (Vec<BVHNode>, Vec<usize>) {
    let mut nodes : Vec<BVHNode> = Vec::with_capacity(2 * n - 1);
    let mut plane_centers = Vec::with_capacity(n);
    let mut plane_indices = Vec::with_capacity(n);


    for i in 0..n {
        plane_centers.push(planes[i].get_center());
        plane_indices.push(i);
    }
    let mut root = BVHNode::new(0, n as u32);
    let nodes_used = 1;
    root.update_node_bounds(&planes, &plane_indices);
    nodes.push(root.clone());
    root.subdivide(&planes, &plane_centers, &mut plane_indices, &mut nodes);
    let _ = mem::replace(&mut nodes[0], root);
    (nodes, plane_indices)
}


fn gen_pixels () -> Vec<(u32, u32)> {
    let pixel_chunk_size = 4;
    let width = 512 / pixel_chunk_size;
    let height = 384 / pixel_chunk_size;
    let mut out_pixels : Vec<(u32, u32)> = Vec::with_capacity((width*height) as usize);

    for i in 0..width {
        for j in 0..height {
            out_pixels.push((pixel_chunk_size * i, pixel_chunk_size * j));
        }
    }
    let mut rng = thread_rng();
    println!("len {}", out_pixels.len());
    out_pixels.shuffle(&mut rng);
    out_pixels
}

fn random_pixels (width : u64, height : u64, pixels : &mut Vec<(u32, u32)>, original : &Vec<(u32, u32)>) -> Vec<(u32, u32)> {
    let mut pixel_update_data : Vec<(u32,u32)> = Vec::with_capacity((height * width) as usize);
    let pixel_chunk_size = 4;
    for _ in 0..height*width / (pixel_chunk_size * pixel_chunk_size) {
        if let Some(pixel) = pixels.pop() {
            pixel_update_data.push(pixel);
        } else {
            pixels.append(&mut original.clone());
            pixel_update_data.push(pixels.pop().unwrap());
        }
    }
    pixel_update_data
}
struct ray {
    ori : Float3,
    dir : Float3
}
fn intersect_aabb(beam : &ray , bmin : Float3, bmax : Float3) -> bool{
    let tx1 = (bmin.0 - beam.ori.0) / beam.dir.0;
    let tx2 = (bmax.0 - beam.ori.0) / beam.dir.0;
    // println!("tx");
    // println!("{tx1}");
    // println!("{tx2}");
    let tmin = tx1.min(tx2);
    let tmax = tx1.max(tx2);
    // println!("tmix");
    // println!("{tmin}");
    // println!("{tmax}");
    let ty1 = (bmin.1 - beam.ori.1) / beam.dir.1;
    let ty2 = (bmax.1 - beam.ori.1) / beam.dir.1;
    // println!("ty");
    // println!("{ty1}");
    // println!("{ty2}");
    let tmin = tmin.max(ty1.min(ty2));
    let tmax = tmax.min(ty1.max(ty2));
    // println!("tmix2");
    // println!("{tmin}");
    // println!("{tmax}");
    let tz1 = (bmin.2 - beam.ori.2) / beam.dir.2;
    let tz2 = (bmax.2 - beam.ori.2) / beam.dir.2;
    // println!("tz");
    // println!("{tz1}");
    // println!("{tz2}");
    let tmin = tmin.max(tz1.min(tz2));
    let tmax = tmax.min(tz1.max(tz2));
    // println!("tmix3");
    // println!("{tmin}");
    // println!("{tmax}");
    return tmax >= tmin && tmin < 1e30f32 && tmax > 0.0;
}

fn main() {

    let mut mirrors : Vec<Plane> = Vec::new();
    let mut materials : Vec<bool> = Vec::new();
    for i in 0..10 {
        if i % 2 == 0 {
            // continue;
        }
        mirrors.push(Plane::new(
            Float3(-10.0 + (i as f32 * 2.0), 0.0, 15.0 - (5i8 - i).abs() as f32),
            Float3(3.0, 0.0, (random::<f32>() - 0.5) * 2.0),
            Float3(0.0, 3.0, (random::<f32>() - 0.5) * 2.0),
            Float3(i as f32 / 10.0, (10.0 - i as f32) / 10.0, (5.0 - i as f32).abs() / 10.0)
        ));
        if i % 3 == 0 {
            materials.push(true);
        } else {
            materials.push(false);
        }
    }
    // for i in -10..10 {
    //     for j in -10..10 {
    //         mirrors.push(Plane::new(
    //             Float3((i * 5) as f32 + random::<f32>(), random::<f32>(), (j * 5) as f32 +  random::<f32>()),
    //             Float3(random::<f32>() * 3.0, 0.0, random::<f32>() * 3.0),
    //             Float3(0.0, random::<f32>() * 3.0, random::<f32>() * 3.0),
    //             Float3(random::<f32>(), random::<f32>(), random::<f32>())
    //         ));
    //     }
    //     if i % 2 == 0 {
    //         materials.push(true);
    //     } else {
    //         materials.push(false);
    //     }
    // }
    // mirrors.push(Plane::new(
    //     Float3(-50.0, -40.0, -50.0),
    //     Float3(100.0, 0.0, 0.0),
    //     Float3(0.0, 80.0, 10.0),
    //     Float3(0.0, 0.4, 0.1)
    // ));
    // materials.push(true);
    // mirrors.push(Plane::new(
    //     Float3(-50.0, -40.0, 50.0),
    //     Float3(0.0, 80.0, 0.0),
    //     Float3(100.0, 0.0, 0.0),
    //     Float3(0.0, 0.4, 0.1)
    // ));
    // materials.push(true);

    println!("Total: {:?}", mirrors.len());
    let (nodes, indices) = build_bvh(mirrors.len(), mirrors.clone());
    println!("Nodes length, {}", nodes.len());
    // println!("{}", nodes.len());
    // println!("{:?}", nodes[2].aabb_min);
    // println!("{:?}", nodes[2].aabb_max);
    // let beam = ray {
    //     ori : Float3::single(0.5),
    //     dir : Float3(0.0001, 0.0001, 1.0)
    // };
    for (i, node) in nodes.iter().enumerate() {
        // if node.tri_count == 1 && intersect_aabb(&beam, node.aabb_min, node.aabb_max) {
        //     println!("Yay! {:?}", mirrors[indices[node.left_first as usize]].origin);
        // }
        print!("{i}: {},   prim_count:", node.left_first);
        println!("{}", node.tri_count);
        if node.tri_count > 1 {
            for i in node.left_first..node.left_first+node.tri_count{
                println!("{:?}", mirrors[indices[i as usize]].origin);
            }
        }
    }
    //Many macOS api calls are main thread only
    let mtm = MainThreadMarker::new().expect("Current thread isn't safe?");

    //Get a reference to the shared application instance object
    let app = NSApplication::sharedApplication(mtm);
    app.setActivationPolicy(NSApplicationActivationPolicy::Regular);

    let style_mask =
        NSWindowStyleMask::Titled.union(
        NSWindowStyleMask::Closable);

    let view_width : f32 = 1024.0 / 2.0;
    let view_height : f32 = 768.0 / 2.0;

    //Initializes an NSWindow object with a size, backing color, title, and style_mask
    let window = initialize_window(
        view_width as f64,
        view_height as f64,
        (0.0, 0.0, 0.0, 1.0),
        "Traced rays!",
        style_mask,
        mtm
    );


    //Grab system GPU device
    let device = Device::system_default().expect("Default device not found!");

    //Grab new MetalLayer object from the GPU to draw to and set our window's content to that MetalLayer
    let layer = new_metal_layer(&device);
    set_window_layer(&window, &layer);

    //Create a queue of GPU command buffers to write to
    let command_queue = device.new_command_queue();

    //Get shader library file
    let shader_lib = get_library(&device);

    //Create a render pipeline consisting of a vertex and fragment function to give to the encoder
    let render_pipeline_state = prepare_pipeline_state(
        &device,
        "vertex_shader",
        "fragment_shader",
        &shader_lib
    );

    let compute_function = shader_lib.get_function("compute_shader", None).expect("Could not find compute function in library");
    let compute_descriptor = ComputePipelineDescriptor::new();
    compute_descriptor.set_compute_function(Some(&compute_function));
    //let compute_pipeline_state = device.new_compute_pipeline_state_with_function(&compute_function).expect("Error creating compute pipeline");
    let compute_pipeline_state = device.new_compute_pipeline_state(&compute_descriptor).expect("Error creating compute pipleine");
    let threadgroup_width = compute_pipeline_state.thread_execution_width();
    let threadgroup_height = compute_pipeline_state.max_total_threads_per_threadgroup() / threadgroup_width;
    let threads_per_threadgroup = MTLSize::new(threadgroup_width, threadgroup_height, 1);

    let threadgroups_per_grid = MTLSize::new((view_width / 4.0) as u64, (view_height / 4.0) as u64, 1);
    println!("{}", compute_pipeline_state.max_total_threads_per_threadgroup());
    println!("{threadgroup_width}, {threadgroup_height}");

    unsafe {
        app.finishLaunching();
        //The method they recommend doesn't exist when I try to use it
        app.activateIgnoringOtherApps(true);
        window.makeKeyAndOrderFront(None);
    };

    //load noise for rng and put into metal texture
    let noise_path = init_nsstring("/Users/basil/rust-projects/mirror-maze/textures/noiseTexture.png", mtm);
    let noise_image = unsafe {
        NSImage::initWithContentsOfFile(mtm.alloc::<NSImage>(), &noise_path).unwrap()
    };

    let noise_data = unsafe { noise_image.TIFFRepresentation().unwrap() };

    // let bitmap = unsafe { NSBitmapImageRep::init };
    let bitmap = unsafe { NSBitmapImageRep::initWithData(mtm.alloc::<NSBitmapImageRep>(), &noise_data).unwrap() };

    let noise_width = 512;
    let noise_height = 512;
    let tex_descriptor = TextureDescriptor::new();
    tex_descriptor.set_pixel_format(MTLPixelFormat::RGBA8Unorm);
    tex_descriptor.set_width(noise_width);
    tex_descriptor.set_height(noise_height);

    let noise_tex = device.new_texture(&tex_descriptor);
    let region = MTLRegion::new_2d(0, 0, noise_width, noise_height);
    unsafe {noise_tex.replace_region(region, 0, bitmap.bitmapData() as *mut _, 4 * noise_width)};


    let tex_usage = MTLTextureUsage::ShaderRead.union(MTLTextureUsage::ShaderWrite);

    // let screen_tex_width = view_width as u64 * threadgroup_width;
    // let screen_tex_height = view_height as u64 * threadgroup_height;
    let screen_tex_descriptor = TextureDescriptor::new();
    screen_tex_descriptor.set_storage_mode(MTLStorageMode::Private);
    screen_tex_descriptor.set_pixel_format(MTLPixelFormat::RGBA8Unorm);
    screen_tex_descriptor.set_usage(tex_usage);
    screen_tex_descriptor.set_width(view_width as u64);
    screen_tex_descriptor.set_height(view_height as u64);


    let screen_tex = device.new_texture(&screen_tex_descriptor);

    let original_pixels = gen_pixels();
    let mut pixels = original_pixels.clone();
    let initial_pixel_data = random_pixels(threadgroups_per_grid.width, threadgroups_per_grid.height, &mut pixels, &original_pixels);
    let pixel_update_buf = make_buf(&initial_pixel_data, &device);

    let quad : Vec<Float3> = vec![
        Float3(-1.0, 1.0, 0.0),
        Float3(1.0, 1.0, 0.0),
        Float3(-1.0, -1.0, 0.0),
        Float3(1.0, -1.0, 0.0),
    ];
    let quad_buf = make_buf(&quad, &device);

    let mirror_buf = make_buf(&mirrors, &device);
    let mat_buf = make_buf(&materials, &device);

    let node_buf = make_buf(&nodes, &device);
    let index_buf = make_buf(&indices, &device);

    let viewport_height = 2.0;
    let viewport_width = viewport_height * (view_width as f32 / view_height as f32);

    let mut camera_center = Float3(0.0, 0.0, 0.0);
    let focal_length = 1.0;

    let mut quat = calculate_quaternion(&Float3(0.1, 0.0, 1.0));
    let mut half_theta = quat.3.acos();

    let init_cam : Camera = Camera { camera_center, focal_length, rotation : quat, viewport: Float2(viewport_width, viewport_height) };
    let cam_buf = make_buf(&vec![init_cam], &device);

    let mirror_count_data = vec![mirrors.len() as u32];

    let fps = 30.0;
    let mut frames = 0;
    let mut frame_time = get_next_frame(fps);

    loop {
        autoreleasepool(|| {
            if app.windows().is_empty() {
                unsafe {app.terminate(None)};
            }
            if unsafe { frame_time.compare(&NSDate::now()) } == NSComparisonResult::Ascending {
                frame_time = get_next_frame(fps);
                frames += 1;
                // if frames % 120 == 0 {
                //     original_pixels.shuffle(&mut rng);
                // }
                let pixel_data = random_pixels(threadgroups_per_grid.width, threadgroups_per_grid.height, &mut pixels, &original_pixels);
                copy_to_buf(&pixel_data, &pixel_update_buf);
                if quat.0.is_nan() || quat.1.is_nan() || quat.2.is_nan() || quat.3.is_nan() {
                    println!("Help!");
                } else {
                    let cam : Camera = Camera { camera_center, focal_length, rotation : quat, viewport: Float2(viewport_width, viewport_height) };
                    copy_to_buf(&vec![cam], &cam_buf);
                }
                //Get a texture from the MetalLayer that we can actually draw to
                let drawable = layer.next_drawable().expect("Unable to find drawable");
                let texture = drawable.texture();

                //Create a new command buffer and an encoder to write to it
                let command_buffer = command_queue.new_command_buffer();
                let render_pass = new_render_pass_descriptor(texture);
                let compute_encoder = command_buffer.new_compute_command_encoder();

                compute_encoder.set_compute_pipeline_state(&compute_pipeline_state);
                compute_encoder.set_buffer(0, Some(&pixel_update_buf), 0);
                compute_encoder.set_buffer(1, Some(&mirror_buf), 0);
                compute_encoder.set_buffer(2, Some(&node_buf), 0);
                compute_encoder.set_buffer(3, Some(&index_buf), 0);
                compute_encoder.set_buffer(4, Some(&cam_buf), 0);
                compute_encoder.set_buffer(5, Some(&mat_buf), 0);
                // compute_encoder.set_bytes(6, size_of::<u32>() as u64, mirror_count_data.as_ptr() as *const _);
                compute_encoder.set_texture(0, Some(&screen_tex));
                compute_encoder.set_texture(1, Some(&noise_tex));
                //compute_encoder.set_threadgroup_memory_length(0, 16);
                compute_encoder.dispatch_thread_groups(threadgroups_per_grid, threads_per_threadgroup);
                compute_encoder.end_encoding();

                let encoder = command_buffer.new_render_command_encoder(render_pass);
                encoder.set_render_pipeline_state(&render_pipeline_state);
                encoder.set_vertex_buffer(0, Some(&quad_buf), 0);
                encoder.set_fragment_texture(0, Some(&screen_tex));
                encoder.draw_primitives(MTLPrimitiveType::TriangleStrip, 0, 4);

                encoder.end_encoding();
                command_buffer.present_drawable(&drawable);
                command_buffer.commit();

                loop {
                    let event = unsafe {NSApp(mtm).nextEventMatchingMask_untilDate_inMode_dequeue(
                        NSAnyEventMask,
                        None,
                        NSDefaultRunLoopMode,
                        true
                    )};

                    match event {
                        Some(ref e) => {
                            unsafe{
                                match e.r#type() {
                                    NSEventType::MouseMoved => {
                                        // half_theta = (half_theta + e.deltaX() as f32 / 512.0) % (std::f32::consts::PI * 2.0);
                                        // quat = update_quat_angle(&quat, half_theta);
                                        // pixels.append(&mut original_pixels.clone());
                                    }
                                    NSEventType::KeyDown => {
                                        camera_center = float3_add(camera_center, Float3(1.0, 0.0, 0.0));
                                    }
                                    _ => {

                                    }
                                }
                                NSApp(mtm).sendEvent(&e);
                            };
                        },
                        None => break,
                    }
                }
            }
        })
    }
}
