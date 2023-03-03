use crate2d::{
    collision::{Collider, ColliderSet, ColliderShape},
    glam::Vec2,
    Box2D, Circle, ForceGenerator, ForceGeneratorSet, ForceRegistry, PhysicsPipeline, RigidBody,
    RigidBodySet,
};
use macroquad::prelude::*;

const CIRCLE_RADIUS: f32 = 1.25;
const BOX_SIZE: f32 = 2.0;
const COLORS: [Color; 2] = [RED, BLUE];
const WIND_SPEED: Vec2 = Vec2::new(-20.0, 0.0);
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
        let mut generators = ForceGeneratorSet::new();
        let mut force_registry = ForceRegistry::new();
        let mut colliders = ColliderSet::new();
        let right_wind_handle = generators.insert(Box::new(Wind { speed: -WIND_SPEED }));
        let left_wind_handle = generators.insert(Box::new(Wind { speed: WIND_SPEED }));

        // Circle vs Circle

        let position = Vec2::new(12.0, 12.0);
        let mut circle1 = RigidBody::new(position, 0.0);
        circle1.set_mass(5.0);
        let circle1_handle = bodies.insert(circle1);
        let position = Vec2::new(25.0, 12.0);
        let mut circle2 = RigidBody::new(position, 0.0);
        circle2.set_mass(10.0);
        let circle2_handle = bodies.insert(circle2);
        force_registry.insert(circle1_handle, right_wind_handle);
        force_registry.insert(circle2_handle, left_wind_handle);
        let shape = ColliderShape::Circle(Circle::new(Vec2::ZERO, CIRCLE_RADIUS));
        let coll1 = Collider::new(shape);
        let coll2 = Collider::new(shape);
        colliders.insert_with_parent(coll1, circle1_handle, &mut bodies);
        colliders.insert_with_parent(coll2, circle2_handle, &mut bodies);

        // Circle vs Box2D

        let position = Vec2::new(12.0, 16.0);
        let mut circle = RigidBody::new(position, 0.0);
        circle.set_mass(5.0);
        let circle_handle = bodies.insert(circle);
        let position = Vec2::new(25.0, 16.0);
        let mut cbox = RigidBody::new(position, f32::to_radians(30.0));
        cbox.set_mass(10.0);
        let cbox_handle = bodies.insert(cbox);
        force_registry.insert(circle_handle, right_wind_handle);
        force_registry.insert(cbox_handle, left_wind_handle);
        let coll1 = Collider::new(ColliderShape::Circle(Circle::new(
            Vec2::ZERO,
            CIRCLE_RADIUS,
        )));
        let coll2 = Collider::new(ColliderShape::Box2D(Box2D::new(
            Vec2::ZERO,
            Vec2::splat(BOX_SIZE),
            0.0,
        )));
        colliders.insert_with_parent(coll1, circle_handle, &mut bodies);
        colliders.insert_with_parent(coll2, cbox_handle, &mut bodies);

        // Box2D vs Box2D

        let position = Vec2::new(12.0, 20.0);
        let mut box1 = RigidBody::new(position, f32::to_radians(45.0));
        box1.set_mass(5.0);
        let box1_handle = bodies.insert(box1);
        let position = Vec2::new(25.0, 20.0);
        let mut box2 = RigidBody::new(position, f32::to_radians(45.0));
        box2.set_mass(10.0);
        let box2_handle = bodies.insert(box2);
        force_registry.insert(box1_handle, right_wind_handle);
        force_registry.insert(box2_handle, left_wind_handle);
        let shape = ColliderShape::Box2D(Box2D::new(Vec2::ZERO, Vec2::splat(BOX_SIZE), 0.0));
        let coll1 = Collider::new(shape);
        let coll2 = Collider::new(shape);
        colliders.insert_with_parent(coll1, box1_handle, &mut bodies);
        colliders.insert_with_parent(coll2, box2_handle, &mut bodies);

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
        for (i, (_, body)) in self.bodies.iter().enumerate() {
            let collider = &self.colliders[body.collider().unwrap()];
            let color = COLORS[i % 2];
            match collider.shape {
                ColliderShape::Circle(c) => draw_circle(
                    c.center.x * 16.0,
                    c.center.y * 16.0,
                    CIRCLE_RADIUS * 16.0,
                    color,
                ),
                ColliderShape::Box2D(b) => {
                    let radius = b.half_size().length();
                    let bcenter = b.center();
                    draw_poly(
                        bcenter.x * 16.0,
                        bcenter.y * 16.0,
                        4,
                        radius * 16.0,
                        45.0 - b.rotation.to_degrees(),
                        color,
                    );
                }
            }
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
