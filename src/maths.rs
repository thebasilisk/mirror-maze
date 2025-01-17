use std::ffi::c_float;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Float4(pub c_float,pub c_float, pub c_float, pub c_float);
impl Float4 {
    pub fn new(v1 : Float2, v2 : Float2) -> Self {
        Self(v1.0, v1.1, v2.0, v2.1)
    }
    pub fn from_float3(v : Float3, s : f32) -> Self {
        Self(v.0, v.1, v.2, s)
    }
}
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Float3(pub c_float, pub c_float, pub c_float);
impl Float3 {
    pub fn new(v : Float2, f : f32) -> Self {
        Self(v.0, v.1, f)
    }
    pub fn magnitude(&self) -> f32 {
        (self.0.powf(2.0) + self.1.powf(2.0) + self.2.powf(2.0)).sqrt()
    }
    pub fn normalized(&self) -> Float3 {
        Float3(self.0 / self.magnitude(), self.1 / self.magnitude(), self.2 / self.magnitude())
    }
}
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Float2(pub c_float, pub c_float);
impl Float2 {
    pub fn magnitude(&self) -> f32 {
        //add cached result
        (self.0.powf(2.0) + self.1.powf(2.0)).sqrt()
    }
    pub fn normalized(&self) -> Float2 {
        Float2(self.0 / self.magnitude(), self.1 / self.magnitude())
    }
    pub fn default() -> Float2 {
        Float2(0.0, 0.0)
    }
}

pub struct Float2x2 {
    row1 : Float2,
    row2 : Float2
}

#[inline]
pub fn float2_add(v1 : Float2, v2 : Float2) -> Float2 {
    Float2(v1.0 + v2.0, v1.1 + v2.1)
}

#[inline]
pub fn float2_subtract(v1 : Float2, v2 : Float2) -> Float2 {
    Float2(v1.0 - v2.0, v1.1 - v2.1)
}
#[inline]
pub fn scale2 (v : Float2, f : f32) -> Float2 {
    Float2(v.0 * f, v.1 * f)
}

#[inline]
pub fn float3_add(v1 : Float3, v2 : Float3) -> Float3 {
    Float3(v1.0 + v2.0, v1.1 + v2.1, v1.2 + v2.2)
}

#[inline]
pub fn float3_subtract(v1 : Float3, v2 : Float3) -> Float3 {
    Float3(v1.0 - v2.0, v1.1 - v2.1, v1.2 - v2.2)
}
#[inline]
pub fn scale3 (v : Float3, f : f32) -> Float3 {
    Float3(v.0 * f, v.1 * f, v.2 * f)
}

#[inline]
pub fn dot(v1 : Float2, v2 : Float2) -> f32 {
    v1.0 * v2.0 + v1.1 * v2.1
}

#[inline]
pub fn dot3(v1 : Float3, v2 : Float3) -> f32 {
    v1.0 * v2.0 + v1.1 * v2.1 + v1.2 * v2.2
}

#[inline]
pub fn matrix_mul(v : Float2, m : Float2x2) -> Float2 {
    Float2(dot(v, m.row1), dot(v, m.row2))
}

#[inline]
pub fn rotation_matrix(theta : f32) -> Float2x2 {
    let cos_theta = f32::cos(theta);
    let sin_theta = f32::sin(theta);
    Float2x2 {
        row1 : Float2(cos_theta, -sin_theta),
        row2 : Float2(sin_theta, cos_theta)
    }
}

#[inline]
pub fn apply_rotation_float2(target : Float2, theta : f32) -> Float2 {
    matrix_mul(target, rotation_matrix(theta))
}

#[inline]
pub fn cross_product(vec1 : &Float3, vec2 : &Float3) -> Float3 {
    Float3(
        vec1.1 * vec2.2 - vec1.2 * vec2.1,
        vec1.2 * vec2.0 - vec1.0 * vec2.2,
        vec1.0 * vec2.1 - vec1.1 * vec2.0
    )
}


// fn calculate_quaternion(camera_rotation_dir : &float3) -> float4 {
//     let default_rotation = float3(0.0, 0.0, 1.0);
//     let camera_rotation = normalize(&camera_rotation_dir);

//     let rotation_axis = cross_product(&default_rotation, &camera_rotation);
//     let rotation_axis_normalized = normalize(&rotation_axis);
//     let rotation_axis_mag = mag(&rotation_axis);

//     //assuming normalized camera rotation vector
//     let half_theta = rotation_axis_mag.asin() / 2.0;
//     //println!("{}", half_theta / PI);

//     float4(
//         rotation_axis_normalized.0 * half_theta.sin(),
//         rotation_axis_normalized.1 * half_theta.sin(),
//         rotation_axis_normalized.2 * half_theta.sin(),
//         half_theta.cos()
//     )
// }
