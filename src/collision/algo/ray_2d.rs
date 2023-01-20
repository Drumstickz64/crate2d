// TODO: add tests

use approx::abs_diff_eq;
use glam::Vec2;

use crate::{math::div_or_zero, Circle, Collider2D, Ray2D, RaycastResult2D, Rect};

pub fn raycast(ray: Ray2D, collider: Collider2D) -> Option<RaycastResult2D> {
    match collider {
        Collider2D::Circle(circle) => raycast_circle(ray, circle),
        Collider2D::Aabb(rect) => raycast_aabb(ray, rect),
        Collider2D::Obb { rect, rotation } => raycast_obb(ray, rect, rotation),
    }
}

fn raycast_circle(ray: Ray2D, circle: Circle) -> Option<RaycastResult2D> {
    let origin_to_center = circle.center - ray.origin;
    let radius_squared = circle.radius * circle.radius;
    let origin_to_center_length_squared = origin_to_center.length_squared();

    // Project the vector from the ray origin onto the direction of the ray
    let a = origin_to_center.dot(ray.direction);
    let b_squared = origin_to_center_length_squared - (a * a);
    if radius_squared - b_squared < 0.0 {
        return None;
    }

    let f = (radius_squared - b_squared).sqrt();
    let t = if origin_to_center_length_squared < radius_squared {
        // ray starts inside the circle
        a + f
    } else {
        a - f
    };

    let point = ray.origin + (ray.direction * t);
    let normal = (point - circle.center).normalize_or_zero();

    Some(RaycastResult2D { point, normal, t })
}

fn raycast_aabb(ray: Ray2D, rect: Rect) -> Option<RaycastResult2D> {
    let origin_to_max = rect.max - ray.origin;
    let origin_to_min = rect.min - ray.origin;
    let direction = ray.direction;

    let step_x_to_max = div_or_zero(origin_to_max.x, direction.x);
    let step_y_to_max = div_or_zero(origin_to_max.y, direction.y);
    let step_x_to_min = div_or_zero(origin_to_min.x, direction.x);
    let step_y_to_min = div_or_zero(origin_to_min.y, direction.y);

    let t_min = f32::max(step_x_to_min, step_y_to_min);
    let t_max = f32::min(step_x_to_max, step_y_to_max);

    if t_max < 0.0 || t_min > t_max {
        return None;
    }

    let t = if t_min < 0.0 { t_max } else { t_min };
    let hit = t > 0.0; // && t * t < ray.maximum
    if !hit {
        return None;
    }

    let point = ray.origin + (ray.direction * t);
    let normal = (ray.origin - point).normalize_or_zero();
    Some(RaycastResult2D { point, normal, t })
}

fn raycast_obb(ray: Ray2D, rect: Rect, rotation: f32) -> Option<RaycastResult2D> {
    let rotation_vec = Vec2::from_angle(-rotation);
    let x_axis = rotation_vec.rotate(Vec2::X);
    let y_axis = rotation_vec.rotate(Vec2::Y);
    let half_size = rect.half_size();

    let p = rect.center() - ray.origin;

    // project the ray direction onto each axis of the box
    let mut f = Vec2::new(x_axis.dot(ray.direction), y_axis.dot(ray.direction));
    // project p onto each axis of the box
    let e = Vec2::new(x_axis.dot(p), y_axis.dot(p));

    let mut t_arr = [0.0; 4];
    // loop over all axes
    for i in 0..2 {
        if abs_diff_eq!(f[i], 0.0) {
            // if the ray is parallel to the current axis and the origin
            // of the ray is not inside the box, we have no hit
            if -e[i] - half_size[i] < 0.0 || -e[i] + half_size[i] > 0.0 {
                return None;
            }
            f[i] = 0.00001;
        }

        t_arr[i * 2] = (e[i] + half_size[i]) / f[i]; // tmax for this axis
        t_arr[i * 2 + 1] = (e[i] - half_size[i]) / f[i]; // tmin for this axis
    }

    let t_min = f32::max(f32::min(t_arr[0], t_arr[1]), f32::min(t_arr[2], t_arr[3]));
    let t_max = f32::min(f32::max(t_arr[0], t_arr[1]), f32::max(t_arr[2], t_arr[3]));

    if t_max < 0.0 || t_min > t_max {
        return None;
    }

    let t = if t_min < 0.0 { t_max } else { t_min };
    let hit = t > 0.0; // && t * t < ray.maximum
    if !hit {
        return None;
    }

    let point = ray.origin + (ray.direction * t);
    let normal = (ray.origin - point).normalize_or_zero();
    Some(RaycastResult2D { point, normal, t })
}
