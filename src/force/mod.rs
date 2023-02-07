mod gravity;
mod registry;

use crate::RigidBody;

pub use gravity::Gravity;
pub(crate) use registry::ForceRegistry;

pub trait ForceGenerator {
    fn update_force(&self, body: &mut RigidBody, dt: f32);
}
