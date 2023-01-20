mod transform;
mod vec2_ext;

use approx::abs_diff_eq;
pub use transform::Transform;
pub use vec2_ext::Vec2Ext;

pub fn div_or_zero(x: f32, y: f32) -> f32 {
    if abs_diff_eq!(y, 0.0) {
        0.0
    } else {
        x / y
    }
}
