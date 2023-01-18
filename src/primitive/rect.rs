use glam::Vec2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub min: Vec2,
    pub max: Vec2,
}

impl Rect {
    pub const fn new(min: Vec2, max: Vec2) -> Self {
        Self { min, max }
    }

    pub fn size(self) -> Vec2 {
        self.max - self.min
    }

    pub fn center(self) -> Vec2 {
        self.min + (self.size() / 2.0)
    }

    pub fn get_vertices(self, _rotation: f32) -> [Vec2; 4] {
        let vertices = [
            Vec2::new(self.min.x, self.min.y),
            Vec2::new(self.min.x, self.max.y),
            Vec2::new(self.max.x, self.min.y),
            Vec2::new(self.max.x, self.max.y),
        ];

        // TODO: implement me
        // if rotation <= f32::EPSILON {
        //     vertices
        // } else {
        // }

        vertices
    }
}
