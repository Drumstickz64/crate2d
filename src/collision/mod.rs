pub mod algo;

use crate::primitive::{Aabb, Circle, Obb};

use self::algo::circle_and_obb;

#[derive(Debug, Clone, PartialEq)]
pub enum Collider2D {
    Circle(Circle),
    Aabb(Aabb),
    Obb(Obb),
}

impl Collider2D {
    pub fn test_collision(&self, other: &Self) -> bool {
        match (self, other) {
            (&Collider2D::Circle(c1), &Collider2D::Circle(c2)) => algo::circle_and_circle(c1, c2),
            (&Collider2D::Circle(circle), &Collider2D::Aabb(aabb)) => {
                algo::circle_and_aabb(circle, aabb)
            }
            (&Collider2D::Circle(circle), &Collider2D::Obb(obb)) => circle_and_obb(circle, obb),
            (&Collider2D::Aabb(aabb), &Collider2D::Circle(circle)) => {
                algo::circle_and_aabb(circle, aabb)
            }
            (&Collider2D::Aabb(aabb1), &Collider2D::Aabb(aabb2)) => {
                algo::aabb_and_aabb(aabb1, aabb2)
            }
            (&Collider2D::Aabb(aabb), &Collider2D::Obb(obb)) => algo::aabb_and_obb(aabb, obb),
            (&Collider2D::Obb(obb), &Collider2D::Circle(circle)) => {
                algo::circle_and_obb(circle, obb)
            }
            (&Collider2D::Obb(obb), &Collider2D::Aabb(aabb)) => algo::aabb_and_obb(aabb, obb),
            (&Collider2D::Obb(obb1), &Collider2D::Obb(obb2)) => algo::obb_and_obb(obb1, obb2),
        }
    }
}
