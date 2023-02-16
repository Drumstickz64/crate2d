use crate2d::{collision, glam::Vec2, Box2D, Circle, ForceGenerator, RigidBody};

use macroquad::prelude::*;

const COLLISION_COLOR: Color = GREEN;
const COLORS: [Color; 2] = [RED, BLUE];

#[macroquad::main("Dynamics demo")]
async fn main() {
    let mut demo = Demo::new();
    demo.run().await
}

struct Demo {
    circle1: Circle,
    circle2: Circle,
    box1: Box2D,
    box2: Box2D,
    are_circles_colliding: bool,
    are_boxes_colliding: bool,
}

impl Demo {
    pub fn new() -> Self {
        Self {
            circle1: Circle::new(Vec2::new(200.0, 200.0), 7.5),
            circle2: Circle::new(Vec2::new(400.0, 200.0), 10.0),
            box1: Box2D::new(Vec2::new(175.0, 375.0), Vec2::new(225.0, 325.0), 0.0),
            box2: Box2D::new(Vec2::new(375.0, 375.0), Vec2::new(425.0, 325.0), 0.0),
            are_circles_colliding: false,
            are_boxes_colliding: false,
        }
    }

    pub fn update(&mut self) {
        let mouse_x = mouse_position().0;
        self.circle2
            .update_position(Vec2::new(mouse_x, self.circle2.center.y));
        self.box2
            .update_position(Vec2::new(mouse_x, self.box2.center().y));
        self.are_circles_colliding = collision::algo::circle_and_circle(self.circle1, self.circle2);
        self.are_boxes_colliding = collision::algo::box2d_and_box2d(self.box1, self.box2);
    }

    pub fn draw(&self) {
        for (i, circle) in [self.circle1, self.circle2].into_iter().enumerate() {
            let circle_color = if self.are_circles_colliding {
                COLLISION_COLOR
            } else {
                COLORS[i]
            };
            draw_circle(
                circle.center.x,
                circle.center.y,
                circle.radius,
                circle_color,
            );
        }
        for (i, box2d) in [self.box1, self.box2].into_iter().enumerate() {
            let box_color = if self.are_boxes_colliding {
                COLLISION_COLOR
            } else {
                COLORS[i]
            };

            let center = box2d.center();
            let size = box2d.size();
            draw_rectangle(center.x, center.y, size.x, size.y, box_color);
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
