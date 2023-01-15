use glam::Vec2;

pub trait Vec2Ex: Clone + Copy {
    fn rotate_around_point(self, rhs: Self, point: Vec2) -> Self;
}

impl Vec2Ex for Vec2 {
    fn rotate_around_point(self, rhs: Self, point: Vec2) -> Self {
        let point_to_rhs = rhs - point;
        let rotated = self.rotate(point_to_rhs);
        rotated + point
    }
}
