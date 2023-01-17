use approx::abs_diff_eq;
use glam::Vec2;

use crate::{
    math::Vec2Ex,
    primitive::{Circle, Line2D, Rect},
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

#[cfg(test)]
mod tests {
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

        let rotation = 45.0;
        assert!(!is_point_in_obb(point, rect, rotation));

        let rotation = 0.0;
        let point = Vec2::new(3.0, 5.0);
        assert!(!is_point_in_obb(point, rect, rotation));
    }
}
