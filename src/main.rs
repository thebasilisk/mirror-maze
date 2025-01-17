mod utils;
mod maths;

use std::{collections::{HashMap, HashSet}, ffi::c_float};

use objc2::rc::Retained;
use rand::{random, seq::SliceRandom, thread_rng};
use objc::rc::autoreleasepool;
use objc2_app_kit::{NSAnyEventMask, NSApp, NSApplication, NSApplicationActivationPolicy, NSBitmapImageFileType, NSBitmapImageRep, NSColor, NSEventType, NSImage, NSImageRep, NSWindowStyleMask};
use objc2_foundation::{MainThreadMarker, NSDefaultRunLoopMode, NSString};
use utils::{copy_to_buf, get_library, init_nsstring, initialize_window, make_buf, new_metal_layer, new_render_pass_descriptor, prepare_pipeline_state, set_window_layer};
use maths::{cross_product, dot, dot3, float2_subtract, float3_add, float3_subtract, scale3, Float2, Float3};

use metal::*;

#[repr(C)]
struct Camera {
    camera_center : Float3,
    focal_length : c_float,
    viewport : Float2
}

struct Ray {
    origin : Float3,
    dir : Float3
}

impl Ray {
    fn new(origin : Float3, dir : Float3) -> Self {
        Ray {origin, dir : dir.normalized()}
    }
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
    // fn from_points (p1 : Float3, p2 : Float3, p3 : Float3) -> Self {}
    fn normal(&self) -> Float3 {
        cross_product(&self.v, &self.u)
    }
}

#[repr(C)]
struct Sphere {
    origin : Float3,
    radius : f32
}

fn ray_rect_intersection (ray : &Ray, rect : &Plane) -> bool {
    //first check if the ray can intersect with the larger plane
    //by checking the angle against the plane's normal vector
    let angle_check = dot3(ray.dir, rect.normal());
    if angle_check != 0.0 {
        //if hitting back of plane then flip
        let sign = angle_check / angle_check.abs();

        //some voodoo to find distance across ray to larger plane
        let a = dot3(float3_subtract(rect.origin,ray.origin), rect.normal()) / (dot3(ray.dir, rect.normal()) * sign);
        // println!("{sign}");
        // println!("{a}");
        let intersection = float3_add(ray.origin, scale3(ray.dir, a));

        //find distances with respect to each rectangle side vector of the intersection point
        let p0p = float3_subtract(intersection, rect.origin);
        let d1 = dot3(p0p, rect.v) / rect.v.magnitude();
        let d2 = dot3(p0p, rect.u) / rect.u.magnitude();

        // println!("d1 : {d1}, d2 : {d2}");
        //if each distance is between 0 and respective side lengths, it's a hit!
        if (0.0 <= d1 && d1 <= rect.v.magnitude()) && (0.0 <= d2 && d2 <= rect.u.magnitude()) {
            return true;
        }
    }
    false
}

fn gen_pixels () -> Vec<(u32, u32)> {
    let pixel_chunk_size = 2;
    let width = 512 / pixel_chunk_size;
    let height = 384 / pixel_chunk_size;
    let mut out_pixels : Vec<(u32, u32)> = Vec::with_capacity((width*height) as usize);

    for i in 0..width {
        for j in 0..height {
            out_pixels.push((pixel_chunk_size * i, pixel_chunk_size * j));
        }
    }
    let mut rng = thread_rng();
    out_pixels.shuffle(&mut rng);
    out_pixels
}

fn random_pixels (width : u64, height : u64, pixels : &mut Vec<(u32, u32)>, original : &Vec<(u32, u32)>) -> Vec<(u32, u32)> {
    let mut pixel_update_data : Vec<(u32,u32)> = Vec::with_capacity((height * width) as usize);
    for _ in 0..height*width {
        if let Some(pixel) = pixels.pop() {
            pixel_update_data.push(pixel);
        } else {
            pixels.append(&mut original.clone());
            pixel_update_data.push(pixels.pop().unwrap());
        }
    }
    pixel_update_data
}

fn new_random_pixels(width : u64, height : u64, view_width : f32, view_height : f32, drawn_pixels : &mut HashSet<(u32, u32)>) -> Vec<(u32, u32)> {
    // let mut pixel_update_data : Vec<Vec<(u32, u32)>> = Vec::with_capacity(height as usize);
    let pixel_chunk_size = 2u32;
    let mut pixel_update_data : Vec<(u32,u32)> = Vec::with_capacity(((height / pixel_chunk_size as u64) * (width / pixel_chunk_size as u64)) as usize);
    for _ in 0..height {
        // pixel_update_data.push(Vec::with_capacity(width as usize));
        for _ in 0..width {
            let mut new_pixel = ((random::<f32>() * view_width / pixel_chunk_size as f32) as u32 * pixel_chunk_size, (random::<f32>() * view_height / pixel_chunk_size as f32) as u32 * pixel_chunk_size);
            while drawn_pixels.contains(&new_pixel) {
                new_pixel = ((random::<f32>() * view_width / pixel_chunk_size as f32) as u32 * pixel_chunk_size, (random::<f32>() * view_height / pixel_chunk_size as f32) as u32 * pixel_chunk_size);
            }
            if drawn_pixels.len() >= ((view_width * view_height * 0.65) / (pixel_chunk_size * pixel_chunk_size) as f32) as usize {
                drawn_pixels.clear();
            }
            drawn_pixels.insert(new_pixel);
            pixel_update_data.push(new_pixel.clone());
        }
    }
    pixel_update_data
}

fn main() {

    let mut mirrors : Vec<Plane> = Vec::new();
    let mut materials : Vec<bool> = Vec::new();
    let mut lights : Vec<Sphere> = Vec::new();
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

    mirrors.push(Plane::new(
        Float3(-50.0, -40.0, -50.0),
        Float3(100.0, 0.0, 0.0),
        Float3(0.0, 80.0, 10.0),
        Float3(0.0, 0.4, 0.1)
    ));
    materials.push(true);
    mirrors.push(Plane::new(
        Float3(-50.0, -40.0, 50.0),
        Float3(0.0, 80.0, 10.0),
        Float3(100.0, 0.0, 0.0),
        Float3(0.0, 0.4, 0.1)
    ));
    materials.push(true);

    lights.push(Sphere { origin: Float3(0.0, 7.0, 25.0), radius: 4.0 });
    println!("{:?}", mirrors.len());
    println!("{materials:?}");

    // let beam = Ray::new(
    //     Float3(0.0, 0.0, 0.0),
    //     Float3(0.0, 0.0, 1.0)
    // );
    // println!("{:?}", mirror.v);
    // println!("{}", ray_rect_intersection(&beam, &mirror));

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
    let compute_pipeline_state = device.new_compute_pipeline_state_with_function(&compute_function).expect("Error creating compute pipeline");
    let threadgroup_width = compute_pipeline_state.thread_execution_width();
    let threadgroup_height = compute_pipeline_state.max_total_threads_per_threadgroup() / threadgroup_width;
    let threads_per_threadgroup = MTLSize::new(threadgroup_width, threadgroup_height, 1);
    // let total_threads_per_threadgroup = threadgroup_width * threadgroup_height;

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
    // screen_tex_descriptor.set_width(screen_tex_width);
    // screen_tex_descriptor.set_height(screen_tex_height);
    screen_tex_descriptor.set_width(view_width as u64);
    screen_tex_descriptor.set_height(view_height as u64);

    // let store_tex_descriptor = TextureDescriptor::new();
    // store_tex_descriptor.set_pixel_format(MTLPixelFormat::RGBA32Float);
    // store_tex_descriptor.set_usage(tex_usage);
    // store_tex_descriptor.set_width(1024);
    // store_tex_descriptor.set_height(768);

    let screen_tex = device.new_texture(&screen_tex_descriptor);
    let mut chosen_pixels : HashSet<(u32, u32)> = HashSet::new();

    println!("{}", (view_width * view_height * 0.50) as usize);
    let original_pixels = gen_pixels();
    let mut pixels = original_pixels.clone();
    //let initial_pixel_data = new_random_pixels(threadgroups_per_grid.width, threadgroups_per_grid.height, view_width, view_height, &mut chosen_pixels);
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
    //let light_buf = make_buf(&lights, &device);
    // let aspect_ratio = 16.0 / 9.0;

    // let image_width = 400;
    // let image_height = (image_width as f32 / aspect_ratio).round() as i32;

    let viewport_height = 2.0;
    let viewport_width = viewport_height * (view_width as f32 / view_height as f32);

    let mut camera_center = Float3(0.0, 0.0, 0.0);
    let focal_length = 1.0;

    let cam : Camera = Camera { camera_center, focal_length, viewport: Float2(viewport_width, viewport_height) };
    let cam_buf = make_buf(&vec![cam], &device);

    //let screen_tex_buf = make_buf(&vec![screen_tex_width, screen_tex_height], &device);


    loop {
        autoreleasepool(|| {
            if app.windows().is_empty() {
                unsafe {app.terminate(None)};
            }

            // let view_rect = app.windows().first().unwrap().contentView().expect("Error unwrapping content view").frame();
            // let new_dimensions = vec![view_rect.size.width as f32, view_rect.size.height as f32];
            // copy_to_buf(&new_dimensions, &screen_buf);

            // viewport_width = viewport_height * (new_dimensions[0] as f32 / new_dimensions[1] as f32);
            // let cam : Camera = Camera { camera_center, focal_length, viewport: Float2(viewport_width, viewport_height) };
            // copy_to_buf(&vec![cam], &cam_buf);

            //let pixel_data = new_random_pixels(threadgroups_per_grid.width, threadgroups_per_grid.height, view_width, view_height, &mut chosen_pixels);
            if pixels.len() > (threadgroups_per_grid.width * threadgroups_per_grid.height) as usize {
                let pixel_data = random_pixels(threadgroups_per_grid.width, threadgroups_per_grid.height, &mut pixels, &original_pixels);
                copy_to_buf(&pixel_data, &pixel_update_buf);
            } else {
                let pixel_data = new_random_pixels(threadgroups_per_grid.width, threadgroups_per_grid.height, view_width, view_height, &mut chosen_pixels);
                copy_to_buf(&pixel_data, &pixel_update_buf);
            }

            //Get a texture from the MetalLayer that we can actually draw to
            //Happening outside the loop to use one updating texture?
            let drawable = layer.next_drawable().expect("Unable to find drawable");
            let texture = drawable.texture();

            //Create a new command buffer and an encoder to write to it
            let command_buffer = command_queue.new_command_buffer();
            let render_pass = new_render_pass_descriptor(texture);
            let compute_encoder = command_buffer.new_compute_command_encoder();

            compute_encoder.set_compute_pipeline_state(&compute_pipeline_state);
            compute_encoder.set_buffer(0, Some(&pixel_update_buf), 0);
            compute_encoder.set_buffer(1, Some(&mirror_buf), 0);
            compute_encoder.set_buffer(2, Some(&cam_buf), 0);
            compute_encoder.set_buffer(3, Some(&mat_buf), 0);
            compute_encoder.set_texture(0, Some(&screen_tex));
            compute_encoder.set_texture(1, Some(&noise_tex));
            //compute_encoder.set_threadgroup_memory_length(0, 16);
            compute_encoder.dispatch_thread_groups(threadgroups_per_grid, threads_per_threadgroup);
            compute_encoder.end_encoding();

            let encoder = command_buffer.new_render_command_encoder(render_pass);
            encoder.set_render_pipeline_state(&render_pipeline_state);
            encoder.set_vertex_buffer(0, Some(&quad_buf), 0);
            // encoder.set_fragment_buffer(0, Some(&screen_tex_buf), 0);
            // encoder.set_fragment_buffer(0, Some(&mirror_buf), 0);
            // encoder.set_fragment_buffer(1, Some(&cam_buf), 0);
            // encoder.set_fragment_buffer(2, Some(&screen_buf), 0);
            // encoder.set_fragment_buffer(3, Some(&light_buf), 0);
            // encoder.set_fragment_buffer(4, Some(&mat_buf), 0);
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
                                    // half_theta = (half_theta + e.deltaX() as f32 / 512.0).rem_euclid(std::f32::consts::PI / 2.0);
                                    // quat = update_quat_angle(&quat, half_theta);
                                }
                                NSEventType::KeyDown => {
                                    camera_center = float3_add(camera_center, Float3(1.0, 0.0, 0.0));
                                    let new_cam = Camera {camera_center, focal_length, viewport: Float2(viewport_width, viewport_height) };
                                    copy_to_buf(&vec![new_cam], &cam_buf);
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
        })
    }
}
