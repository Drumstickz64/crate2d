pub mod algo;

use crate::primitive::{Circle, Rect};

#[derive(Debug, Clone, PartialEq)]
pub enum Collider2D {
    Circle(Circle),
    Aabb(Rect),
    Obb { rect: Rect, rotation: f32 },
}

impl Collider2D {
    pub fn test_collision(&self, other: &Self) -> bool {
        match (self, other) {
            (&Collider2D::Circle(c1), &Collider2D::Circle(c2)) => algo::circle_and_circle(c1, c2),
            (&Collider2D::Circle(circle), &Collider2D::Aabb(rect)) => {
                algo::circle_and_aabb(circle, rect)
            }
            (Collider2D::Circle(_), Collider2D::Obb { rect, rotation }) => todo!(),
            (Collider2D::Aabb(_), Collider2D::Circle(_)) => todo!(),
            (Collider2D::Aabb(_), Collider2D::Aabb(_)) => todo!(),
            (Collider2D::Aabb(_), Collider2D::Obb { rect, rotation }) => todo!(),
            (Collider2D::Obb { rect, rotation }, Collider2D::Circle(_)) => todo!(),
            (Collider2D::Obb { rect, rotation }, Collider2D::Aabb(_)) => todo!(),
            (
                Collider2D::Obb {
                    rect: rect1,
                    rotation: rotation1,
                },
                Collider2D::Obb {
                    rect: rect2,
                    rotation: rotation2,
                },
            ) => todo!(),
        }
    }
}
