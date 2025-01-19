mod utils;
mod maths;

use std::ffi::c_float;

use rand::{random, seq::SliceRandom, thread_rng};
use objc::rc::autoreleasepool;
use objc2_app_kit::{NSAnyEventMask, NSApp, NSApplication, NSApplicationActivationPolicy, NSBitmapImageRep, NSEventType, NSImage, NSWindowStyleMask};
use objc2_foundation::{MainThreadMarker, NSComparisonResult, NSDate, NSDefaultRunLoopMode};
use utils::{copy_to_buf, get_library, get_next_frame, init_nsstring, initialize_window, make_buf, new_metal_layer, new_render_pass_descriptor, prepare_pipeline_state, set_window_layer};
use maths::{calculate_quaternion, float3_add, update_quat_angle, Float2, Float3, Float4};

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


fn main() {

    let mut mirrors : Vec<Plane> = Vec::new();
    let mut materials : Vec<bool> = Vec::new();
    for i in 0..10 {
        if i % 2 == 0 {
            //continue;
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
    // for i in -3..3 {
    //     for j in -3..3 {
    //         mirrors.push(Plane::new(
    //             Float3(i as f32, 0.0, j as f32),
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
    mirrors.push(Plane::new(
        Float3(-50.0, -40.0, -50.0),
        Float3(100.0, 0.0, 0.0),
        Float3(0.0, 80.0, 10.0),
        Float3(0.0, 0.4, 0.1)
    ));
    materials.push(true);
    mirrors.push(Plane::new(
        Float3(-50.0, -40.0, 50.0),
        Float3(0.0, 80.0, 0.0),
        Float3(100.0, 0.0, 0.0),
        Float3(0.0, 0.4, 0.1)
    ));
    materials.push(true);

    println!("{:?}", mirrors.len());

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
                compute_encoder.set_buffer(2, Some(&cam_buf), 0);
                compute_encoder.set_buffer(3, Some(&mat_buf), 0);
                compute_encoder.set_bytes(4, size_of::<u32>() as u64, mirror_count_data.as_ptr() as *const _);
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
                                        half_theta = (half_theta + e.deltaX() as f32 / 512.0) % (std::f32::consts::PI * 2.0);
                                        quat = update_quat_angle(&quat, half_theta);
                                        pixels.append(&mut original_pixels.clone());
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
