use approx::abs_diff_eq;
use glam::Vec2;

use crate::{
    math::{div_or_zero, Vec2Ext},
    primitive::{Convex, Obb},
    Aabb, Circle, Collider2D, Line2D, Ray2D, RaycastResult2D,
};

pub fn is_point_on_line(point: Vec2, line: Line2D) -> bool {
    // Line equation: y = mx + b
    let dx = line.end.x - line.start.x;
    if dx == 0.0 {
        return abs_diff_eq!(point.x, line.start.x);
    }
    let dy = line.end.y - line.start.y;
    let slope = dy / dx;

    // any point to find b
    let y_intercept = line.end.y - (slope * line.end.x);

    // check if point is on line
    abs_diff_eq!(point.y, slope * point.x + y_intercept)
}

pub fn is_point_in_circle(point: Vec2, circle: Circle) -> bool {
    circle.center.distance_squared(point) <= circle.radius * circle.radius
}

pub fn is_point_in_aabb(point: Vec2, aabb: Aabb) -> bool {
    point.x >= aabb.min.x && point.x <= aabb.max.x && point.y >= aabb.min.y && point.y <= aabb.max.y
}

pub fn is_point_in_obb(point: Vec2, obb: Obb) -> bool {
    let rotated_point = Vec2::from_angle(obb.rotation).rotate_around_point(point, obb.center());
    rotated_point.x >= obb.min.x
        && rotated_point.x <= obb.max.x
        && rotated_point.y >= obb.min.y
        && rotated_point.y <= obb.max.y
}

pub fn line_and_circle(line: Line2D, circle: Circle) -> bool {
    if is_point_in_circle(line.start, circle) || is_point_in_circle(line.end, circle) {
        return true;
    }

    let line_start_to_center = circle.center - line.start;
    let line_vec = line.to_vec2();
    let closest_point = line.start + line_start_to_center.project_onto(line_vec);

    is_point_in_circle(closest_point, circle)
}

pub fn line_and_aabb(line: Line2D, aabb: Aabb) -> bool {
    if is_point_in_aabb(line.start, aabb) || is_point_in_aabb(line.end, aabb) {
        return true;
    }

    let start_to_max = aabb.max - line.start;
    let start_to_min = aabb.min - line.start;
    let line_direction = line.direction();
    let step_x_to_max = div_or_zero(start_to_max.x, line_direction.x);
    let step_y_to_max = div_or_zero(start_to_max.y, line_direction.y);
    let step_x_to_min = div_or_zero(start_to_min.x, line_direction.x);
    let step_y_to_min = div_or_zero(start_to_min.y, line_direction.y);

    let t_min = step_x_to_min.max(step_y_to_min);
    let t_max = step_x_to_max.min(step_y_to_max);

    if t_max < 0.0 || t_min > t_max {
        return false;
    }

    let t = if t_min < 0.0 { t_max } else { t_min };
    t > 0.0 && t * t < line.length_squared()
}

pub fn line_and_obb(line: Line2D, obb: Obb) -> bool {
    let rotation_vec = Vec2::from_angle(-obb.rotation);
    let rect_center = obb.center();
    let rotated_start = rotation_vec.rotate_around_point(line.start, rect_center);
    let rotated_end = rotation_vec.rotate_around_point(line.end, rect_center);
    let rotated_line = Line2D::new(rotated_start, rotated_end);
    line_and_aabb(rotated_line, Aabb::new(obb.min, obb.max))
}

pub fn raycast(ray: Ray2D, collider: Collider2D) -> Option<RaycastResult2D> {
    match collider {
        Collider2D::Circle(circle) => raycast_circle(ray, circle),
        Collider2D::Aabb(aabb) => raycast_aabb(ray, aabb),
        Collider2D::Obb(obb) => raycast_obb(ray, obb),
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

fn raycast_aabb(ray: Ray2D, aabb: Aabb) -> Option<RaycastResult2D> {
    let origin_to_max = aabb.max - ray.origin;
    let origin_to_min = aabb.min - ray.origin;
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

fn raycast_obb(ray: Ray2D, obb: Obb) -> Option<RaycastResult2D> {
    let rotation_vec = Vec2::from_angle(-obb.rotation);
    let x_axis = rotation_vec.rotate(Vec2::X);
    let y_axis = rotation_vec.rotate(Vec2::Y);
    let half_size = obb.half_size();

    let p = obb.center() - ray.origin;

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

pub fn circle_and_circle(c1: Circle, c2: Circle) -> bool {
    let radii_sum = c1.radius + c2.radius;
    c1.center.distance_squared(c2.center) <= radii_sum * radii_sum
}

pub fn circle_and_aabb(circle: Circle, aabb: Aabb) -> bool {
    let mut closest_point_to_circle = circle.center;

    if closest_point_to_circle.x < aabb.min.x {
        closest_point_to_circle.x = aabb.min.x;
    } else if closest_point_to_circle.x > aabb.max.x {
        closest_point_to_circle.x = aabb.max.x;
    }

    if closest_point_to_circle.y < aabb.min.y {
        closest_point_to_circle.y = aabb.min.y;
    } else if closest_point_to_circle.y > aabb.max.y {
        closest_point_to_circle.y = aabb.max.y;
    }

    closest_point_to_circle.distance_squared(circle.center) <= circle.radius * circle.radius
}

pub fn circle_and_obb(circle: Circle, obb: Obb) -> bool {
    // Treat the box as an aabb after rotating the stuff
    // create local box
    let local_rect = Aabb::new(Vec2::ZERO, obb.size());

    // create local circle
    let r = circle.center - obb.center();
    let r = Vec2::from_angle(-obb.rotation).rotate(r);
    let local_circle_pos = r + obb.half_size();
    let local_circle = Circle::new(local_circle_pos, circle.radius);

    circle_and_aabb(local_circle, local_rect)
}

pub fn aabb_and_aabb(rect1: Aabb, rect2: Aabb) -> bool {
    Vec2::AXES
        .into_iter()
        .all(|axis| is_overlapping_on_axis(rect1, rect2, axis))
}

pub fn aabb_and_obb(aabb: Aabb, obb: Obb) -> bool {
    let center = obb.center();
    let rotation_vec = Vec2::from_angle(obb.rotation);
    let obb_axes = [
        rotation_vec.rotate_around_point(Vec2::X, center),
        rotation_vec.rotate_around_point(Vec2::Y, center),
    ];

    Vec2::AXES
        .into_iter()
        .chain(obb_axes)
        .all(|axis| is_overlapping_on_axis(aabb, obb, axis))
}

pub fn obb_and_obb(obb1: Obb, obb2: Obb) -> bool {
    let center1 = obb1.center();
    let rotation_vec1 = Vec2::from_angle(obb1.rotation);
    let obb1_axes = [
        rotation_vec1.rotate_around_point(Vec2::X, center1),
        rotation_vec1.rotate_around_point(Vec2::Y, center1),
    ];

    let center2 = obb2.center();
    let rotation_vec2 = Vec2::from_angle(obb2.rotation);
    let obb2_axes = [
        rotation_vec2.rotate_around_point(Vec2::X, center2),
        rotation_vec2.rotate_around_point(Vec2::Y, center2),
    ];

    obb1_axes
        .into_iter()
        .chain(obb2_axes)
        .all(|axis| is_overlapping_on_axis(obb1, obb2, axis))
}

// ==========================================
// SAT helpers
// ==========================================

fn is_overlapping_on_axis(shape1: impl Convex, shape2: impl Convex, axis: Vec2) -> bool {
    let interval1 = get_interval(shape1, axis);
    let interval2 = get_interval(shape2, axis);
    interval1.y >= interval2.x && interval1.x <= interval2.y
}

fn get_interval(aabb: impl Convex, axis: Vec2) -> Vec2 {
    let vertices = aabb.get_vertices();

    vertices.into_iter().map(|vertex| axis.dot(vertex)).fold(
        Vec2::ZERO,
        |mut result, projection| {
            result.x = result.x.min(projection);
            result.y = result.y.max(projection);
            result
        },
    )
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use glam::Vec2;

    use super::*;

    #[test]
    fn test_is_point_on_line() {
        let point = Vec2::new(1.0, 2.0);
        let line = Line2D::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 4.0));
        assert!(is_point_on_line(point, line));

        let point = Vec2::new(3.0, 5.0);
        assert!(!is_point_on_line(point, line));

        let point = Vec2::new(0.0, 4.0);
        let line = Line2D::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 4.0));
        assert!(is_point_on_line(point, line));
    }

    #[test]
    fn test_is_point_in_circle() {
        let point = Vec2::new(1.0, 2.0);
        let circle = Circle::new(Vec2::new(0.0, 0.0), 2.0);
        assert!(!is_point_in_circle(point, circle));

        let point = Vec2::new(0.0, 1.0);
        assert!(is_point_in_circle(point, circle));
    }

    #[test]
    fn test_is_point_in_aabb() {
        let point = Vec2::new(1.0, 2.0);
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0));
        assert!(is_point_in_aabb(point, aabb));

        let point = Vec2::new(3.0, 5.0);
        assert!(!is_point_in_aabb(point, aabb));
    }

    #[test]
    fn test_is_point_in_obb() {
        let point = Vec2::new(0.2, 3.8);
        let mut obb = Obb::new(Vec2::new(0.0, 0.0), Vec2::new(4.0, 4.0), 0.0);
        assert!(is_point_in_obb(point, obb));

        obb.rotation = PI / 4.0;
        assert!(!is_point_in_obb(point, obb));

        obb.rotation = 0.0;
        let point = Vec2::new(3.0, 5.0);
        assert!(!is_point_in_obb(point, obb));
    }

    const LINE_INTERSECTION_CIRCLE: Circle = Circle::new(Vec2::splat(5.0), 3.0);

    #[test]
    fn line_intersects_circle() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(5.0, 3.0));
        assert!(line_and_circle(line, LINE_INTERSECTION_CIRCLE));
    }

    #[test]
    fn line_does_not_intersect_circle() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(10.0, 4.0));
        assert!(!line_and_circle(line, LINE_INTERSECTION_CIRCLE));
    }

    #[test]
    fn line_inside_circle_intersects_it() {
        let point_inside = Vec2::new(3.0, 4.0);
        let point_outside = Vec2::new(10.0, 5.0);
        let start_inside = Line2D::new(point_inside, point_outside);
        assert!(line_and_circle(start_inside, LINE_INTERSECTION_CIRCLE));
        let end_inside = Line2D::new(point_outside, point_inside);
        assert!(line_and_circle(end_inside, LINE_INTERSECTION_CIRCLE));
    }

    #[test]
    fn line_intersects_aabb() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(11.0, 4.0));
        let aabb = Aabb::new(Vec2::ZERO, Vec2::splat(8.0));
        assert!(line_and_aabb(line, aabb));
    }

    #[test]
    fn line_does_not_intersect_aabb() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(11.0, 0.0));
        let aabb = Aabb::new(Vec2::ZERO, Vec2::splat(8.0));
        assert!(!line_and_aabb(line, aabb));
    }

    #[test]
    fn line_inside_aabb_intersects_it() {
        let point_inside = Vec2::new(3.0, 5.0);
        let point_outside = Vec2::new(12.0, 4.0);
        let start_inside = Line2D::new(point_inside, point_outside);
        let aabb = Aabb::new(Vec2::ZERO, Vec2::splat(8.0));
        assert!(line_and_aabb(start_inside, aabb));
        let end_inside = Line2D::new(point_outside, point_inside);
        assert!(line_and_aabb(end_inside, aabb));
    }

    #[test]
    fn line_intersects_obb_when_before_rotation_and_doesnt_after_rotation() {
        let line = Line2D::new(Vec2::new(-0.2, 6.0), Vec2::new(0.5, 9.0));
        let mut obb = Obb::new(Vec2::ZERO, Vec2::splat(8.0), 0.0);
        assert!(line_and_obb(line, obb));
        obb.rotation = PI / 4.0;
        assert!(!line_and_obb(line, obb));
    }
}
