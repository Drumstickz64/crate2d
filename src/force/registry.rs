use generational_arena::{Arena, Index};

use crate::{RigidBodyHandle, RigidBodySet};

use super::{ForceGeneratorHandle, ForceGeneratorSet};

#[derive(Default, Debug)]
pub struct ForceRegistry {
    arena: Arena<ForceRegistration>,
}

impl ForceRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(
        &mut self,
        rigid_body_handle: RigidBodyHandle,
        force_generator_handle: ForceGeneratorHandle,
    ) -> ForceRegistrationHandle {
        let registration = ForceRegistration {
            rigid_body_handle,
            force_generator_handle,
        };
        let index = self.arena.insert(registration);
        ForceRegistrationHandle(index)
    }

    pub fn remove(&mut self, handle: ForceRegistrationHandle) -> Option<ForceRegistration> {
        self.arena.remove(handle.0)
    }

    pub fn clear(&mut self) {
        self.arena.clear()
    }

    pub fn update_forces(
        &self,
        bodies: &mut RigidBodySet,
        generators: &ForceGeneratorSet,
        dt: f32,
    ) {
        for (_, registration) in &self.arena {
            let body = &mut bodies[registration.rigid_body_handle];
            let generator = &generators[registration.force_generator_handle];
            generator.update_force(body, dt);
        }
    }
}
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct ForceRegistrationHandle(pub Index);

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct ForceRegistration {
    pub rigid_body_handle: RigidBodyHandle,
    pub force_generator_handle: ForceGeneratorHandle,
}
