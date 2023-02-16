mod force_set;
mod registry;

use crate::RigidBody;

pub use force_set::*;
pub use registry::*;

pub trait ForceGenerator {
    fn update_force(&self, body: &mut RigidBody, dt: f32);
}
