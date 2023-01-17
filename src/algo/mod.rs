use crate::primitive::{Circle, Line2D};

use self::point::is_point_in_circle;

pub mod point;

pub fn is_line_2d_intersecting_circle(line: Line2D, circle: Circle) -> bool {
    if is_point_in_circle(line.start, circle) || is_point_in_circle(line.end, circle) {
        return true;
    }

    let line_start_to_center = circle.center - line.start;
    let line_vec = line.to_vec2();
    let closest_point = line.start + line_start_to_center.project_onto(line_vec);

    is_point_in_circle(closest_point, circle)
}
