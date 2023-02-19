use glam::Vec2;

use crate::math::Vec2Ext;

use super::Convex;

macro_rules! impl_rect_common_methods {
    () => {
        pub fn size(self) -> Vec2 {
            self.max - self.min
        }

        pub fn half_size(self) -> Vec2 {
            self.size() / 2.0
        }

        pub fn set_size(&mut self, new_size: Vec2) {
            let size_delta = new_size - self.size();
            self.min -= size_delta;
            self.max += size_delta;
        }

        pub fn center(self) -> Vec2 {
            self.min + self.half_size()
        }

        pub fn set_center(&mut self, center: Vec2) {
            let half_size = self.half_size();
            self.min = center - half_size;
            self.max = center + half_size;
        }
    };
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb {
    pub const fn new(min: Vec2, max: Vec2) -> Aabb {
        Self { min, max }
    }

    impl_rect_common_methods!();

    pub fn from_center_and_half_size(center: Vec2, half_size: Vec2) -> Aabb {
        let min = center - half_size;
        let max = center + half_size;
        Self { min, max }
    }
}

impl From<Box2D> for Aabb {
    fn from(b: Box2D) -> Self {
        Self {
            min: b.min,
            max: b.max,
        }
    }
}

impl Convex<4> for Aabb {
    fn get_vertices(&self) -> [Vec2; 4] {
        [
            Vec2::new(self.min.x, self.min.y),
            Vec2::new(self.min.x, self.max.y),
            Vec2::new(self.max.x, self.min.y),
            Vec2::new(self.max.x, self.max.y),
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Box2D {
    pub min: Vec2,
    pub max: Vec2,
    pub rotation: f32,
}

impl Box2D {
    pub const fn new(min: Vec2, max: Vec2, rotation: f32) -> Self {
        Self { min, max, rotation }
    }

    impl_rect_common_methods!();
}

impl From<Aabb> for Box2D {
    fn from(b: Aabb) -> Self {
        Self {
            min: b.min,
            max: b.max,
            rotation: 0.0,
        }
    }
}

impl Convex<4> for Box2D {
    fn get_vertices(&self) -> [Vec2; 4] {
        let center = self.center();
        let rotation_vec = Vec2::from_angle(self.rotation);
        [
            rotation_vec.rotate_around_point(Vec2::new(self.min.x, self.min.y), center),
            rotation_vec.rotate_around_point(Vec2::new(self.min.x, self.max.y), center),
            rotation_vec.rotate_around_point(Vec2::new(self.max.x, self.min.y), center),
            rotation_vec.rotate_around_point(Vec2::new(self.max.x, self.max.y), center),
        ]
    }
}
