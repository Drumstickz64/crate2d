use generational_arena::{Arena, Index};

use crate::RigidBody;

pub trait ForceGenerator {
    fn update_force(&self, body: &mut RigidBody, dt: f32);
}

#[derive(Default, Debug)]
pub(crate) struct ForceRegistry {
    arena: Arena<ForceRegistration>,
}

impl ForceRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, rigid_body_index: Index, force_generator_index: Index) -> Index {
        let registration = ForceRegistration {
            rigid_body_index,
            force_generator_index,
        };
        self.arena.insert(registration)
    }

    pub fn remove(&mut self, i: Index) -> Option<ForceRegistration> {
        self.arena.remove(i)
    }

    pub fn clear(&mut self) {
        self.arena.clear()
    }

    pub fn update_forces(
        &self,
        bodies: &mut Arena<RigidBody>,
        generators: &mut Arena<Box<dyn ForceGenerator>>,
        dt: f32,
    ) {
        for (_, registration) in &self.arena {
            let body = &mut bodies[registration.force_generator_index];
            let generator = &generators[registration.force_generator_index];
            generator.update_force(body, dt);
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone)]
pub(crate) struct ForceRegistration {
    pub rigid_body_index: Index,
    pub force_generator_index: Index,
}
