use glam::Vec2;

use crate::{
    collision::{ColliderHandle, ColliderSet},
    math::{self},
    ForceRegistrationHandle,
};

#[derive(Debug, Clone)]
pub struct RigidBody {
    pub is_rotation_fixed: bool,
    // Coefficient of restitution
    pub cor: f32,
    pub(crate) position: Vec2,
    pub(crate) rotation: f32,
    pub(crate) mass: f32,
    pub(crate) inv_mass: f32,
    pub(crate) force_accumulator: Vec2,
    pub(crate) linear_velocity: Vec2,
    pub(crate) angular_velocity: f32,
    pub(crate) linear_damping: f32,
    pub(crate) angular_damping: f32,
    pub(crate) collider: Option<ColliderHandle>,
    pub(crate) force_registrations: Vec<ForceRegistrationHandle>,
}

impl RigidBody {
    pub fn new(position: Vec2, rotation: f32) -> Self {
        Self {
            position,
            rotation,
            is_rotation_fixed: false,
            cor: 1.0,
            mass: 0.0,
            inv_mass: 0.0,
            force_accumulator: Vec2::ZERO,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            collider: None,
            force_registrations: Vec::new(),
        }
    }

    pub fn physics_update(&mut self, dt: f32, colliders: &mut ColliderSet) {
        if self.mass == 0.0 {
            return;
        }

        let acceleration = self.force_accumulator * self.inv_mass;
        self.linear_velocity += acceleration * dt;
        self.position += self.linear_velocity * dt;

        if let Some(collider) = self.collider {
            colliders[collider].update_position(self.position);
        }

        self.clear_accumelators()
    }

    pub fn add_force(&mut self, force: Vec2) {
        self.force_accumulator += force;
    }

    pub fn set_collider(&mut self, collider: ColliderHandle) {
        self.collider = Some(collider);
    }

    pub fn clear_accumelators(&mut self) {
        self.force_accumulator = Vec2::ZERO;
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

    pub fn inv_mass(&self) -> f32 {
        self.inv_mass
    }

    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        self.inv_mass = math::recip_or_zero(mass);
    }

    pub fn has_infinite_mass(&self) -> bool {
        self.mass == 0.0
    }

    pub fn position(&self) -> Vec2 {
        self.position
    }

    pub fn rotation(&self) -> f32 {
        self.rotation
    }

    pub fn collider(&self) -> Option<ColliderHandle> {
        self.collider
    }
}
