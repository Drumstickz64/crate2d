use glam::Vec2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray2D {
    pub origin: Vec2,
    pub direction: Vec2,
}

impl Ray2D {
    pub fn new(origin: Vec2, direction: Vec2) -> Self {
        Self {
            origin,
            direction: direction.normalize_or_zero(),
        }
    }
}

pub struct RaycastResult2D {
    pub point: Vec2,
    pub normal: Vec2,
    pub t: f32,
}
