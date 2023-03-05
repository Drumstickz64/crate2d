use crate2d::{collision::ColliderShape, glam::Vec2, Box2D, Circle, ForceGenerator, RigidBody};

use macroquad::prelude::*;

#[macroquad::main("Dynamics demo")]
async fn main() {
    let mut demo = Demo::new();
    demo.run().await
}

struct Demo {
    circle1: ColliderShape,
    circle2: ColliderShape,
    box1: ColliderShape,
    box2: ColliderShape,
    colliding: Vec<ColliderShape>,
}

impl Demo {
    pub fn new() -> Self {
        Self {
            circle1: ColliderShape::Circle(Circle::new(Vec2::new(200.0, 200.0), 7.5)),
            circle2: ColliderShape::Circle(Circle::new(Vec2::new(400.0, 200.0), 10.0)),
            box1: ColliderShape::Box2D(Box2D::new(
                Vec2::new(175.0, 325.0),
                Vec2::new(225.0, 375.0),
                f32::to_radians(30.0),
            )),
            box2: ColliderShape::Box2D(Box2D::new(
                Vec2::new(375.0, 325.0),
                Vec2::new(425.0, 375.0),
                f32::to_radians(45.0),
            )),
            colliding: Vec::new(),
        }
    }

    pub fn update(&mut self) {
        self.colliding.clear();
        let mouse_pos = Vec2::from(mouse_position());
        let offset_vec = Vec2::new(0.0, 50.0);
        self.circle2.update_position(mouse_pos - offset_vec);
        self.box2.update_position(mouse_pos + offset_vec);
        let shapes = [self.circle1, self.box1, self.circle2, self.box2];
        for (i, shape1) in shapes.into_iter().enumerate() {
            for shape2 in shapes.into_iter().skip(i + 1) {
                if shape1.test_collision(shape2).is_some() {
                    self.colliding.extend([shape1, shape2]);
                }
            }
        }
    }

    pub fn draw(&self) {
        for shape in [self.circle1, self.circle2, self.box1, self.box2] {
            let color = if self.colliding.contains(&shape) {
                GREEN
            } else {
                RED
            };
            match shape {
                ColliderShape::Circle(c) => draw_circle(c.center.x, c.center.y, c.radius, color),
                ColliderShape::Box2D(b) => {
                    let bcenter = b.center();
                    let radius = b.half_size().length();
                    draw_poly(
                        bcenter.x,
                        bcenter.y,
                        4,
                        radius,
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
