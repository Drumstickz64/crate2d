use crate::math::{Circle, Rect};

#[derive(Debug, Clone, PartialEq)]
pub enum Collider2D {
    Circle(Circle),
    Rect(Rect),
}
