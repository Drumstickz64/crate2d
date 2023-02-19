use glam::Vec2;

use super::algo;
use crate::{
    primitive::{Box2D, Circle},
    RigidBodyHandle,
};

#[derive(Debug, Clone, PartialEq)]
pub struct Collider {
    pub shape: ColliderShape,
    pub(crate) parent: Option<RigidBodyHandle>,
}

impl Collider {
    pub fn new(shape: ColliderShape) -> Self {
        Self {
            shape,
            parent: None,
        }
    }

    pub fn test_collision(&self, other: &Self) -> bool {
        self.shape.test_collision(other.shape)
    }

    pub fn update_position(&mut self, position: Vec2) {
        self.shape.update_position(position);
    }

    pub fn update_rotation(&mut self, rotation: f32) {
        self.shape.update_rotation(rotation);
    }

    pub fn set_parent(&mut self, parent: RigidBodyHandle) {
        self.parent = Some(parent);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ColliderShape {
    Circle(Circle),
    Box2D(Box2D),
}

impl ColliderShape {
    pub fn test_collision(self, other: Self) -> bool {
        match (self, other) {
            (ColliderShape::Circle(c1), ColliderShape::Circle(c2)) => {
                algo::circle_and_circle(c1, c2)
            }
            (ColliderShape::Circle(circle), ColliderShape::Box2D(box2d)) => {
                algo::circle_and_box2d(circle, box2d)
            }
            (ColliderShape::Box2D(box2d), ColliderShape::Circle(circle)) => {
                algo::circle_and_box2d(circle, box2d)
            }
            (ColliderShape::Box2D(b1), ColliderShape::Box2D(b2)) => algo::box2d_and_box2d(b1, b2),
        }
    }

    pub fn update_position(&mut self, position: Vec2) {
        match self {
            ColliderShape::Circle(circle) => circle.center = position,
            ColliderShape::Box2D(box2d) => box2d.set_center(position),
        }
    }

    fn update_rotation(&mut self, rotation: f32) {
        match self {
            ColliderShape::Circle(_) => (),
            ColliderShape::Box2D(box2d) => box2d.rotation = rotation,
        }
    }

    pub fn center(self) -> Vec2 {
        match self {
            ColliderShape::Circle(c) => c.center,
            ColliderShape::Box2D(b) => b.center(),
        }
    }
}
