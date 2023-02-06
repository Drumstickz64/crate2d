use glam::Vec2;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Transform {
    pub position: Vec2,
    pub rotation: f32,
}

impl Transform {
    pub fn from_position(position: Vec2) -> Self {
        Self {
            position,
            ..Default::default()
        }
    }

    pub fn from_rotation(rotation: f32) -> Self {
        Self {
            rotation,
            ..Default::default()
        }
    }
}
