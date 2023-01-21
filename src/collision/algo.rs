use approx::abs_diff_eq;
use glam::Vec2;

use crate::{
    math::{div_or_zero, Vec2Ext},
    Circle, Collider2D, Line2D, Ray2D, RaycastResult2D, Rect,
};

pub fn is_point_on_line_2d(point: Vec2, line: Line2D) -> bool {
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

pub fn is_point_in_aabb(point: Vec2, rect: Rect) -> bool {
    point.x >= rect.min.x && point.x <= rect.max.x && point.y >= rect.min.y && point.y <= rect.max.y
}

pub fn is_point_in_obb(point: Vec2, rect: Rect, rotation: f32) -> bool {
    let rotated_point = Vec2::from_angle(rotation).rotate_around_point(point, rect.center());
    is_point_in_aabb(rotated_point, rect)
}

pub fn is_line_2d_intersecting_circle(line: Line2D, circle: Circle) -> bool {
    if is_point_in_circle(line.start, circle) || is_point_in_circle(line.end, circle) {
        return true;
    }

    let line_start_to_center = circle.center - line.start;
    let line_vec = line.to_vec2();
    let closest_point = line.start + line_start_to_center.project_onto(line_vec);

    is_point_in_circle(closest_point, circle)
}

pub fn is_line_2d_intersecting_aabb(line: Line2D, rect: Rect) -> bool {
    if is_point_in_aabb(line.start, rect) || is_point_in_aabb(line.end, rect) {
        return true;
    }

    let start_to_max = rect.max - line.start;
    let start_to_min = rect.min - line.start;
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

pub fn is_line_2d_intersecting_obb(line: Line2D, rect: Rect, rotation: f32) -> bool {
    let rotation_vec = Vec2::from_angle(-rotation);
    let rect_center = rect.center();
    let rotated_start = rotation_vec.rotate_around_point(line.start, rect_center);
    let rotated_end = rotation_vec.rotate_around_point(line.end, rect_center);
    let rotated_line = Line2D::new(rotated_start, rotated_end);
    is_line_2d_intersecting_aabb(rotated_line, rect)
}

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

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use glam::Vec2;

    use super::*;

    #[test]
    fn test_is_point_on_line_2d() {
        let point = Vec2::new(1.0, 2.0);
        let line = Line2D::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 4.0));
        assert!(is_point_on_line_2d(point, line));

        let point = Vec2::new(3.0, 5.0);
        assert!(!is_point_on_line_2d(point, line));

        let point = Vec2::new(0.0, 4.0);
        let line = Line2D::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 4.0));
        assert!(is_point_on_line_2d(point, line));
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
        let rect = Rect::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0));
        assert!(is_point_in_aabb(point, rect));

        let point = Vec2::new(3.0, 5.0);
        assert!(!is_point_in_aabb(point, rect));
    }

    #[test]
    fn test_is_point_in_obb() {
        let point = Vec2::new(0.2, 3.8);
        let rect = Rect::new(Vec2::new(0.0, 0.0), Vec2::new(4.0, 4.0));
        let rotation = 0.0;
        assert!(is_point_in_obb(point, rect, rotation));

        let rotation = PI / 4.0;
        assert!(!is_point_in_obb(point, rect, rotation));

        let rotation = 0.0;
        let point = Vec2::new(3.0, 5.0);
        assert!(!is_point_in_obb(point, rect, rotation));
    }

    const LINE_INTERSECTION_CIRCLE: Circle = Circle::new(Vec2::splat(5.0), 3.0);
    const LINE_INTERSECTION_RECT: Rect = Rect::new(Vec2::ZERO, Vec2::splat(8.0));

    #[test]
    fn line_intersects_circle() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(5.0, 3.0));
        assert!(is_line_2d_intersecting_circle(
            line,
            LINE_INTERSECTION_CIRCLE
        ));
    }

    #[test]
    fn line_does_not_intersect_circle() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(10.0, 4.0));
        assert!(!is_line_2d_intersecting_circle(
            line,
            LINE_INTERSECTION_CIRCLE
        ));
    }

    #[test]
    fn line_inside_circle_intersects_it() {
        let point_inside = Vec2::new(3.0, 4.0);
        let point_outside = Vec2::new(10.0, 5.0);
        let start_inside = Line2D::new(point_inside, point_outside);
        assert!(is_line_2d_intersecting_circle(
            start_inside,
            LINE_INTERSECTION_CIRCLE
        ));
        let end_inside = Line2D::new(point_outside, point_inside);
        assert!(is_line_2d_intersecting_circle(
            end_inside,
            LINE_INTERSECTION_CIRCLE
        ));
    }

    #[test]
    fn line_intersects_rect() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(11.0, 4.0));
        assert!(is_line_2d_intersecting_aabb(line, LINE_INTERSECTION_RECT));
    }

    #[test]
    fn line_does_not_intersect_rect() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(11.0, 0.0));
        assert!(!is_line_2d_intersecting_aabb(line, LINE_INTERSECTION_RECT));
    }

    #[test]
    fn line_inside_rect_intersects_it() {
        let point_inside = Vec2::new(3.0, 5.0);
        let point_outside = Vec2::new(12.0, 4.0);
        let start_inside = Line2D::new(point_inside, point_outside);
        assert!(is_line_2d_intersecting_aabb(
            start_inside,
            LINE_INTERSECTION_RECT
        ));
        let end_inside = Line2D::new(point_outside, point_inside);
        assert!(is_line_2d_intersecting_aabb(
            end_inside,
            LINE_INTERSECTION_RECT
        ));
    }

    #[test]
    fn line_intersects_obb_when_not_rotated_and_doesnt_when_rotated() {
        let line = Line2D::new(Vec2::new(-0.2, 6.0), Vec2::new(0.5, 9.0));
        assert!(is_line_2d_intersecting_obb(
            line,
            LINE_INTERSECTION_RECT,
            0.0
        ));
        assert!(!is_line_2d_intersecting_obb(
            line,
            LINE_INTERSECTION_RECT,
            PI / 4.0
        ));
    }
}
