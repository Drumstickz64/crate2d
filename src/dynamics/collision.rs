use glam::Vec2;

use crate::RigidBodyHandle;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Collision {
    pub body_handle1: RigidBodyHandle,
    pub body_handle2: RigidBodyHandle,
    pub manifold: CollisionManifold,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CollisionManifold {
    pub normal: Vec2,
    pub contact_points: Vec<Vec2>,
    pub depth: f32,
}

pub(crate) mod algo {
    use crate::{
        collision::{Collider, ColliderShape},
        Circle,
    };

    use super::CollisionManifold;

    pub(crate) fn find_collider_collider_collision_features(
        coll1: &Collider,
        coll2: &Collider,
    ) -> Option<CollisionManifold> {
        match (coll1.shape, coll2.shape) {
            (ColliderShape::Circle(c1), ColliderShape::Circle(c2)) => {
                find_circle_circle_collision_features(c1, c2)
            }
            (ColliderShape::Circle(_), ColliderShape::Box2D(_)) => todo!(),
            (ColliderShape::Box2D(_), ColliderShape::Circle(_)) => todo!(),
            (ColliderShape::Box2D(_), ColliderShape::Box2D(_)) => todo!(),
        }
    }

    pub(crate) fn find_circle_circle_collision_features(
        c1: Circle,
        c2: Circle,
    ) -> Option<CollisionManifold> {
        let sum_radii = c1.radius + c2.radius;
        let c1_to_c2 = c2.center - c1.center;
        if c1_to_c2.length_squared() > sum_radii * sum_radii {
            return None;
        }

        let normal = c1_to_c2.normalize_or_zero();
        // Multiply by 0.5 because we want the contact point to be in the middle
        // of the two circles to seperate the two circles the same amount
        let depth = (c1_to_c2.length() - sum_radii).abs() * 0.5;
        let distance_to_point = c1.radius - depth;
        let contact_point = c1.center + normal * distance_to_point;

        Some(CollisionManifold {
            normal,
            contact_points: vec![contact_point],
            depth,
        })
    }
}
