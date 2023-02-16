use glam::Vec2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle {
    pub center: Vec2,
    pub radius: f32,
}

impl Circle {
    pub const fn new(center: Vec2, radius: f32) -> Self {
        Self { center, radius }
    }

    pub fn update_position(&mut self, position: Vec2) {
        self.center = position;
    }
}
