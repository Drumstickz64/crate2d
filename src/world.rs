use generational_arena::{Arena, Index};
use glam::Vec2;

use crate::{force::ForceRegistry, ForceGenerator, Gravity, RigidBody};

pub struct PhysicsWorld {
    rigid_bodies: Arena<RigidBody>,
    force_generators: Arena<Box<dyn ForceGenerator>>,
    force_registry: ForceRegistry,
    gravity_index: Index,
    fixed_dt: f32,
}

impl PhysicsWorld {
    pub fn new(gravity: Vec2, fixed_dt: f32) -> Self {
        let gravity = Gravity::new(gravity);
        let mut force_generators: Arena<Box<dyn ForceGenerator>> = Arena::new();
        let gravity_index = force_generators.insert(Box::new(gravity));
        Self {
            rigid_bodies: Arena::new(),
            force_generators,
            force_registry: ForceRegistry::new(),
            gravity_index,
            fixed_dt,
        }
    }

    pub fn update(&mut self, dt: f32) {
        // TODO: Use dt to sync physics updates
        self.fixed_update();
    }

    pub fn fixed_update(&mut self) {
        self.force_registry.update_forces(
            &mut self.rigid_bodies,
            &self.force_generators,
            self.fixed_dt,
        );

        for (_, body) in &mut self.rigid_bodies {
            body.physics_update(self.fixed_dt);
        }
    }

    pub fn add_rigid_body(&mut self, body: RigidBody) -> Index {
        let rigid_body_index = self.rigid_bodies.insert(body);
        self.force_registry
            .insert(rigid_body_index, self.gravity_index);
        rigid_body_index
    }
}
