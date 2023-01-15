use glam::Vec2;

#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct Line2D {
    pub start: Vec2,
    pub end: Vec2,
}

impl Line2D {
    pub fn new(start: Vec2, end: Vec2) -> Self {
        Self { start, end }
    }
}
