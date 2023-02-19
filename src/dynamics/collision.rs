use glam::Vec2;

use crate::RigidBodyHandle;

#[derive(Debug, Clone, PartialEq)]
pub struct Collision {
    pub body_handle1: RigidBodyHandle,
    pub body_handle2: RigidBodyHandle,
    pub manifold: CollisionManifold,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CollisionManifold {
    pub normal: Vec2,
    // TODO: Actually use contant points and depth
    pub contact_point_a: Vec2,
    pub contact_point_b: Vec2,
    pub depth: f32,
}

pub mod algo {
    use glam::Vec2;

    use crate::{
        collision::{self, Collider, ColliderShape},
        math::Vec2Ext,
        Aabb, Box2D, Circle,
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
            (ColliderShape::Circle(circle), ColliderShape::Box2D(box2d)) => {
                find_box2d_circle_collision_features(box2d, circle).map(|manifold| {
                    CollisionManifold {
                        normal: -manifold.normal,
                        ..manifold
                    }
                })
            }
            (ColliderShape::Box2D(box2d), ColliderShape::Circle(circle)) => {
                find_box2d_circle_collision_features(box2d, circle)
            }
            (ColliderShape::Box2D(b1), ColliderShape::Box2D(b2)) => {
                find_box2d_box2d_collision_features(b1, b2)
            }
        }
    }

    pub fn find_circle_circle_collision_features(
        c1: Circle,
        c2: Circle,
    ) -> Option<CollisionManifold> {
        let sum_radii = c1.radius + c2.radius;
        let c1_to_c2 = c2.center - c1.center;
        if c1_to_c2.length_squared() > sum_radii * sum_radii {
            return None;
        }

        let normal = c1_to_c2.normalize_or_zero();
        let depth = (c1_to_c2.length() - sum_radii).abs();
        let contact_point_a = c1.center + normal * c1.radius;
        let contact_point_b = c2.center + normal * c2.radius;

        Some(CollisionManifold {
            normal,
            contact_point_a,
            contact_point_b,
            depth,
        })
    }

    pub fn find_aabb_circle_collision_features(
        aabb: Aabb,
        circle: Circle,
    ) -> Option<CollisionManifold> {
        let aabb_center = aabb.center();
        let aabb_to_circle = circle.center - aabb_center;
        let half_size = aabb.half_size();
        let closest_point_on_box = aabb_to_circle.clamp(-half_size, half_size);
        let local_point = aabb_to_circle - closest_point_on_box;
        let distance_squared = local_point.length_squared();
        if distance_squared > circle.radius * circle.radius {
            return None;
        }

        let normal = local_point.normalize_or_zero();
        let distance = distance_squared.sqrt();
        let depth = circle.radius - distance;
        let contact_point_a = aabb_center;
        let contact_point_b = -normal * circle.radius;
        Some(CollisionManifold {
            normal,
            contact_point_a,
            contact_point_b,
            depth,
        })
    }

    pub fn find_box2d_circle_collision_features(
        box2d: Box2D,
        circle: Circle,
    ) -> Option<CollisionManifold> {
        // Bring circle to local box2d rotation
        let local_center =
            Vec2::from_angle(box2d.rotation).rotate_around_point(circle.center, box2d.center());
        let local_circle = Circle::new(local_center, circle.radius);
        find_aabb_circle_collision_features(Aabb::from(box2d), local_circle).map(|manifold| {
            CollisionManifold {
                normal: Vec2::from_angle(-box2d.rotation).rotate(manifold.normal),
                ..manifold
            }
        })
    }

    pub fn find_aabb_aabb_collision_features(b1: Aabb, b2: Aabb) -> Option<CollisionManifold> {
        if !collision::algo::aabb_and_aabb(b1, b2) {
            return None;
        }

        let faces = [Vec2::NEG_X, Vec2::X, Vec2::NEG_Y, Vec2::Y];
        let distances = [
            b2.max.x - b1.min.x, // distance from b2 to 'left' of b1
            b1.max.x - b2.min.x, // distance from b2 to 'right' of b1
            b2.max.y - b1.min.y, // distance from b2 to 'bottom' of b2
            b1.max.y - b2.min.y, // distance from b2 to 'top' of b2
        ];

        let mut depth = f32::MAX;
        let mut best_axis = faces[0];
        for i in 0..4 {
            if distances[i] < depth {
                depth = distances[i];
                best_axis = faces[i];
            }
        }

        Some(CollisionManifold {
            normal: best_axis,
            contact_point_a: b1.center(),
            contact_point_b: b2.center(),
            depth,
        })
    }

    pub fn find_box2d_box2d_collision_features(b1: Box2D, b2: Box2D) -> Option<CollisionManifold> {
        let rotation_vec1 = Vec2::from_angle(b1.rotation);
        let b1_axes = [rotation_vec1.rotate(Vec2::X), rotation_vec1.rotate(Vec2::Y)];

        let rotation_vec2 = Vec2::from_angle(b2.rotation);
        let b2_axes = [rotation_vec2.rotate(Vec2::X), rotation_vec2.rotate(Vec2::Y)];

        let mut have_collided = false;
        let mut best_axis = b1_axes[0];
        let mut depth = f32::MAX;
        for axis in b1_axes.into_iter().chain(b2_axes.into_iter()) {
            let interval1 = collision::algo::get_interval(b1, axis);
            let interval2 = collision::algo::get_interval(b2, axis);
            if !(interval1.x <= interval2.y && interval2.x <= interval1.y) {
                continue;
            }

            have_collided = true;
            let overlap = f32::min(interval1.y, interval2.y) - f32::max(interval1.x, interval2.x);
            if overlap < depth {
                depth = overlap;
                best_axis = axis;
            }
        }

        if !have_collided {
            return None;
        }

        Some(CollisionManifold {
            normal: best_axis,
            contact_point_a: b1.center(),
            contact_point_b: b2.center(),
            depth,
        })
    }
}
