use glam::Vec2;

use crate::{
    math::{div_or_zero, Vec2Ext},
    primitive::{Circle, Line2D, Rect},
};

use super::point::{is_point_in_aabb, is_point_in_circle};

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

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use glam::Vec2;

    use super::*;

    const CIRCLE: Circle = Circle::new(Vec2::splat(5.0), 3.0);
    const RECT: Rect = Rect::new(Vec2::ZERO, Vec2::splat(8.0));

    #[test]
    fn line_intersects_circle() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(5.0, 3.0));
        assert!(is_line_2d_intersecting_circle(line, CIRCLE));
    }

    #[test]
    fn line_does_not_intersect_circle() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(10.0, 4.0));
        assert!(!is_line_2d_intersecting_circle(line, CIRCLE));
    }

    #[test]
    fn line_inside_circle_intersects_it() {
        let point_inside = Vec2::new(3.0, 4.0);
        let point_outside = Vec2::new(10.0, 5.0);
        let start_inside = Line2D::new(point_inside, point_outside);
        assert!(is_line_2d_intersecting_circle(start_inside, CIRCLE));
        let end_inside = Line2D::new(point_outside, point_inside);
        assert!(is_line_2d_intersecting_circle(end_inside, CIRCLE));
    }

    #[test]
    fn line_intersects_rect() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(11.0, 4.0));
        assert!(is_line_2d_intersecting_aabb(line, RECT));
    }

    #[test]
    fn line_does_not_intersect_rect() {
        let line = Line2D::new(Vec2::new(3.0, -2.0), Vec2::new(11.0, 0.0));
        assert!(!is_line_2d_intersecting_aabb(line, RECT));
    }

    #[test]
    fn line_inside_rect_intersects_it() {
        let point_inside = Vec2::new(3.0, 5.0);
        let point_outside = Vec2::new(12.0, 4.0);
        let start_inside = Line2D::new(point_inside, point_outside);
        assert!(is_line_2d_intersecting_aabb(start_inside, RECT));
        let end_inside = Line2D::new(point_outside, point_inside);
        assert!(is_line_2d_intersecting_aabb(end_inside, RECT));
    }

    #[test]
    fn line_intersects_obb_when_not_rotated_and_doesnt_when_rotated() {
        let line = Line2D::new(Vec2::new(-0.2, 6.0), Vec2::new(0.5, 9.0));
        assert!(is_line_2d_intersecting_obb(line, RECT, 0.0));
        assert!(!is_line_2d_intersecting_obb(line, RECT, PI / 4.0));
    }
}
