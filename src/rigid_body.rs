use glam::Vec2;

use crate::math::Transform;

#[derive(Debug, Default, Clone)]
pub struct RigidBody {
    pub is_rotation_fixed: bool,
    transform: Transform,
    linear_velocity: Vec2,
    angular_velocity: f32,
    linear_damping: f32,
    angular_damping: f32,
}

impl RigidBody {
    pub fn new(transform: Transform) -> Self {
        Self {
            transform,
            ..Default::default()
        }
    }

    pub fn transform(&self) -> Transform {
        self.transform
    }

    pub fn linear_velocity(&self) -> Vec2 {
        self.linear_velocity
    }

    pub fn angular_velocity(&self) -> f32 {
        self.angular_velocity
    }

    pub fn linear_damping(&self) -> f32 {
        self.linear_damping
    }

    pub fn angular_damping(&self) -> f32 {
        self.angular_damping
    }
}
