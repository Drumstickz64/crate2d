use glam::Vec2;

use crate::{ForceGenerator, RigidBody};

#[derive(Debug, Clone)]
pub struct Gravity(Vec2);

impl Gravity {
    pub fn new(force: Vec2) -> Self {
        Self(force)
    }
}

impl ForceGenerator for Gravity {
    fn update_force(&self, body: &mut RigidBody, _dt: f32) {
        body.add_force(self.0 * body.mass());
    }
}
