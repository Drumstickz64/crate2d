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

    pub fn to_vec2(self) -> Vec2 {
        self.end - self.start
    }

    pub fn direction(self) -> Vec2 {
        (self.end - self.start).normalize_or_zero()
    }

    pub fn length_squared(&self) -> f32 {
        self.end.distance_squared(self.start)
    }

    pub fn length(&self) -> f32 {
        self.end.distance(self.start)
    }
}
