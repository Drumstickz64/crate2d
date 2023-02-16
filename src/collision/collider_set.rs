use generational_arena::{Arena, Index};

use crate::{RigidBodyHandle, RigidBodySet};

use super::Collider;

#[derive(Debug, Clone, Default)]
pub struct ColliderSet {
    colliders: Arena<Collider>,
}

impl ColliderSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, collider: Collider) -> ColliderHandle {
        let index = self.colliders.insert(collider);
        ColliderHandle(index)
    }

    pub fn insert_with_parent(
        &mut self,
        mut collider: Collider,
        body_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
    ) -> ColliderHandle {
        let body = &mut bodies[body_handle];
        collider.parent = Some(body_handle);
        collider.update_position(body.position);
        let collider_handle = self.insert(collider);
        body.collider = Some(collider_handle);
        collider_handle
    }

    pub fn remove(
        &mut self,
        handle: ColliderHandle,
        bodies: &mut RigidBodySet,
    ) -> Option<Collider> {
        let collider = self.colliders.remove(handle.0)?;
        if let Some(parent_handle) = collider.parent {
            bodies[parent_handle].collider = None;
        }

        Some(collider)
    }

    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle.0)
    }

    pub fn get2_mut(
        &mut self,
        handle1: ColliderHandle,
        handle2: ColliderHandle,
    ) -> (Option<&mut Collider>, Option<&mut Collider>) {
        self.colliders.get2_mut(handle1.0, handle2.0)
    }

    pub fn iter(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter().map(|(i, rb)| (ColliderHandle(i), rb))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        self.colliders
            .iter_mut()
            .map(|(i, rb)| (ColliderHandle(i), rb))
    }

    pub fn len(&self) -> usize {
        self.colliders.len()
    }

    pub fn is_empty(&self) -> bool {
        self.colliders.is_empty()
    }
}

impl std::ops::Index<ColliderHandle> for ColliderSet {
    type Output = Collider;

    fn index(&self, handle: ColliderHandle) -> &Self::Output {
        &self.colliders[handle.0]
    }
}

impl std::ops::IndexMut<ColliderHandle> for ColliderSet {
    fn index_mut(&mut self, index: ColliderHandle) -> &mut Self::Output {
        &mut self.colliders[index.0]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColliderHandle(pub Index);
