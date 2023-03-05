mod circle;
mod line_2d;
mod ray_2d;
mod rect;

pub use circle::Circle;
use glam::Vec2;
pub use line_2d::Line2D;
pub use ray_2d::{Ray2D, RaycastResult2D};
pub use rect::{Aabb, Box2D};

pub trait Convex<const N: usize> {
    fn get_vertices(&self) -> [Vec2; N];
}
