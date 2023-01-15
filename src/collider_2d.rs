use crate::primitive::{Circle, Rect};

#[derive(Debug, Clone, PartialEq)]
pub enum Collider2D {
    Circle(Circle),
    Aabb(Rect),
    Obb { rect: Rect, rotation: f32 },
}
