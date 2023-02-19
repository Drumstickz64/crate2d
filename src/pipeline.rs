use glam::Vec2;

use crate::{
    collision::ColliderSet,
    dynamics::collision::{
        algo::find_collider_collider_collision_features, Collision, CollisionManifold,
    },
    ForceGeneratorSet, ForceRegistry, RigidBody, RigidBodySet,
};

pub struct PhysicsPipeline {
    fixed_dt: f32,
    collisions: Vec<Collision>,
}

impl PhysicsPipeline {
    const IMPULSE_ITERATIONS: u32 = 6;

    pub fn new(fixed_dt: f32) -> Self {
        Self {
            fixed_dt,
            collisions: Vec::new(),
        }
    }

    pub fn step(
        &mut self,
        _dt: f32,
        gravity: Vec2,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        generators: &ForceGeneratorSet,
        force_registry: &ForceRegistry,
    ) {
        // TODO: Use dt to sync physics steps
        self.fixed_step(gravity, bodies, colliders, generators, force_registry);
    }

    pub fn fixed_step(
        &mut self,
        gravity: Vec2,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        generators: &ForceGeneratorSet,
        force_registry: &ForceRegistry,
    ) {
        // Update forces
        force_registry.update_forces(bodies, generators, self.fixed_dt);
        for (_, body) in bodies.iter_mut() {
            body.add_force(gravity * body.mass());
        }

        // Find collisions
        self.collisions.clear();
        for (i, (body_handle1, body1)) in bodies.iter().enumerate() {
            for (body_handle2, body2) in bodies.iter().skip(i + 1) {
                if body1.has_infinite_mass() && body2.has_infinite_mass() {
                    continue;
                }

                let (Some(coll_handle_1), Some(coll_handle_2)) = (body1.collider, body2.collider) else {
                    continue;
                };

                let coll1 = &colliders[coll_handle_1];
                let coll2 = &colliders[coll_handle_2];

                let Some(manifold) = find_collider_collider_collision_features(coll1, coll2) else {
                    continue;
                };

                let collision = Collision {
                    body_handle1,
                    body_handle2,
                    manifold,
                };

                self.collisions.push(collision);
            }
        }

        // Resolve collisions via iterative impulse resolution
        for _ in 0..Self::IMPULSE_ITERATIONS {
            for collision in self.collisions.iter() {
                let (Some(rb1), Some(rb2)) = bodies.get2_mut(collision.body_handle1, collision.body_handle2) else {
                        panic!("Rigid body not found");
                    };
                self.apply_impulse(rb1, rb2, &collision.manifold);
            }
        }

        // Update velocities
        for (_, body) in bodies.iter_mut() {
            body.physics_update(self.fixed_dt, colliders);
        }
    }

    pub fn fixed_dt(&self) -> f32 {
        self.fixed_dt
    }

    pub fn set_fixed_dt(&mut self, fixed_dt: f32) {
        self.fixed_dt = fixed_dt;
    }

    fn apply_impulse(
        &self,
        rb1: &mut RigidBody,
        rb2: &mut RigidBody,
        manifold: &CollisionManifold,
    ) {
        let inv_mass_sum = rb1.inv_mass + rb2.inv_mass;
        if inv_mass_sum == 0.0 {
            return;
        }
        let relative_vel = rb2.linear_velocity - rb1.linear_velocity;
        let are_moving_apart = relative_vel.dot(manifold.normal) > 0.0;
        if are_moving_apart {
            return;
        }
        let e = rb1.cor.min(rb2.cor);
        let impulse_vel = -((1.0 + e) * relative_vel.dot(manifold.normal));
        let impulse = impulse_vel / inv_mass_sum;

        rb1.linear_velocity -= rb1.inv_mass * impulse * manifold.normal;
        rb2.linear_velocity += rb2.inv_mass * impulse * manifold.normal;
    }
}
