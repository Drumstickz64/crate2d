use crate2d::{
    collision::ColliderSet, glam::Vec2, ForceGeneratorSet, ForceRegistry, PhysicsPipeline,
    RigidBody, RigidBodySet,
};
use macroquad::prelude::*;

const BOX_EXTENTS: Vec2 = Vec2::splat(5.0);
const BOX_COLORS: [Color; 2] = [RED, BLUE];
const GRAVITY: Vec2 = Vec2::new(0.0, 10.0);
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
        let position1 = Vec2::new(12.5, 12.5);
        let mut rb1 = RigidBody::new(position1, 0.0);
        rb1.set_mass(30.0);
        bodies.insert(rb1);
        let position2 = Vec2::new(25.0, 12.5);
        let mut rb2 = RigidBody::new(position2, 0.0);
        rb2.set_mass(60.0);
        bodies.insert(rb2);

        let generators = ForceGeneratorSet::new();
        let force_registry = ForceRegistry::new();
        let colliders = ColliderSet::new();

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
            GRAVITY,
            &mut self.bodies,
            &mut self.colliders,
            &self.generators,
            &self.force_registry,
        );
    }

    pub fn draw(&self) {
        for (color_i, (_, body)) in self.bodies.iter().enumerate() {
            draw_rectangle(
                body.position().x * 16.0,
                body.position().y * 16.0,
                BOX_EXTENTS.x,
                BOX_EXTENTS.y,
                BOX_COLORS[color_i],
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
