use glam::Vec2;

use crate::math::{self, Transform};

#[derive(Debug, Default, Clone)]
pub struct RigidBody {
    pub is_rotation_fixed: bool,
    transform: Transform,
    mass: f32,
    mass_recip: f32,
    force_accumulator: Vec2,
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

    pub fn physics_update(&mut self, dt: f32) {
        if self.mass == 0.0 {
            return;
        }

        let acceleration = self.force_accumulator * self.mass_recip;
        self.linear_velocity += acceleration * dt;
        self.transform.position += self.linear_velocity * dt;
        self.clear_accumelators()
    }

    pub fn add_force(&mut self, force: Vec2) {
        self.force_accumulator += force;
    }
    pub fn clear_accumelators(&mut self) {
        self.force_accumulator = Vec2::ZERO;
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

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        self.mass_recip = math::recip_or_zero(mass);
    }
}
