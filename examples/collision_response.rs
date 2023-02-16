use crate2d::{
    collision::{Collider, ColliderSet, ColliderShape},
    glam::Vec2,
    Circle, ForceGenerator, ForceGeneratorSet, ForceRegistry, PhysicsPipeline, RigidBody,
    RigidBodySet,
};
use macroquad::prelude::*;

const CIRCLE_RADIUS: f32 = 0.75;
const COLORS: [Color; 2] = [RED, BLUE];
const WIND_SPEED: Vec2 = Vec2::new(-31.25, 0.0);
const FIXED_DT: f32 = 1.0 / 60.0;

#[macroquad::main("Dynamics demo")]
async fn main() {
    let mut demo = Demo::new();
    demo.run().await
}

struct Demo {
    pipeline: PhysicsPipeline,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    generators: ForceGeneratorSet,
    force_registry: ForceRegistry,
}

impl Demo {
    pub fn new() -> Self {
        let pipeline = PhysicsPipeline::new(FIXED_DT);
        let mut bodies = RigidBodySet::new();
        let position1 = Vec2::new(12.0, 12.0);
        let mut rb1 = RigidBody::new(position1, 0.0);
        rb1.set_mass(5.0);
        let rb1_handle = bodies.insert(rb1);
        let position2 = Vec2::new(25.0, 12.0);
        let mut rb2 = RigidBody::new(position2, 0.0);
        rb2.set_mass(10.0);
        let rb2_handle = bodies.insert(rb2);

        let mut generators = ForceGeneratorSet::new();
        let right_wind_handle = generators.insert(Box::new(Wind { speed: -WIND_SPEED }));
        let left_wind_handle = generators.insert(Box::new(Wind { speed: WIND_SPEED }));
        let mut force_registry = ForceRegistry::new();
        force_registry.insert(rb1_handle, right_wind_handle);
        force_registry.insert(rb2_handle, left_wind_handle);
        let mut colliders = ColliderSet::new();
        let shape = ColliderShape::Circle(Circle::new(Vec2::ZERO, CIRCLE_RADIUS));
        let coll1 = Collider::new(shape);
        let coll2 = Collider::new(shape);
        colliders.insert_with_parent(coll1, rb1_handle, &mut bodies);
        colliders.insert_with_parent(coll2, rb2_handle, &mut bodies);

        Self {
            pipeline,
            bodies,
            colliders,
            generators,
            force_registry,
        }
    }

    pub fn update(&mut self) {
        let dt = get_frame_time();
        self.pipeline.step(
            dt,
            Vec2::ZERO,
            &mut self.bodies,
            &mut self.colliders,
            &self.generators,
            &self.force_registry,
        );
    }

    pub fn draw(&self) {
        for (color_i, (_, body)) in self.bodies.iter().enumerate() {
            draw_circle(
                body.position().x * 16.0,
                body.position().y * 16.0,
                CIRCLE_RADIUS * 16.0,
                COLORS[color_i],
            )
        }
    }

    pub async fn run(&mut self) {
        loop {
            self.update();
            self.draw();
            next_frame().await
        }
    }
}

struct Wind {
    speed: Vec2,
}

impl ForceGenerator for Wind {
    fn update_force(&self, body: &mut RigidBody, _dt: f32) {
        body.add_force(self.speed);
    }
}
