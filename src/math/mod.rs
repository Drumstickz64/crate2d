mod transform;
mod vec2_ext;

pub use transform::Transform;
pub use vec2_ext::Vec2Ext;

pub fn div_or_zero(x: f32, y: f32) -> f32 {
    if y == 0.0 {
        0.0
    } else {
        x / y
    }
}

pub fn recip_or_zero(x: f32) -> f32 {
    if x == 0.0 {
        0.0
    } else {
        1.0 / x
    }
}
