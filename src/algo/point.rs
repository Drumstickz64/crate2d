use approx::abs_diff_eq;
use glam::Vec2;

use crate::{
    math::{Transform, Vec2Ex},
    primitive::{Circle, Line2D, Rect},
};

pub fn is_point_on_line_2d(point: Vec2, line: Line2D) -> bool {
    // Line equation: y = mx + b

    let dy = line.end.y - line.start.y;
    let dx = line.end.x - line.start.x;
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
