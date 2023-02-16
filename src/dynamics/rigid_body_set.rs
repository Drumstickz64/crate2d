use generational_arena::{Arena, Index};

use crate::{collision::ColliderSet, ForceRegistry, RigidBody};

#[derive(Debug, Clone, Default)]
pub struct RigidBodySet {
    bodies: Arena<RigidBody>,
}

impl RigidBodySet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, body: RigidBody) -> RigidBodyHandle {
        let index = self.bodies.insert(body);
        RigidBodyHandle(index)
    }

    pub fn remove(
        &mut self,
        handle: RigidBodyHandle,
        colliders: &mut ColliderSet,
        registry: &mut ForceRegistry,
    ) -> Option<RigidBody> {
        let body = self.bodies.remove(handle.0)?;

        if let Some(collider) = body.collider {
            colliders[collider].parent = None;
        }

        for registration_handle in body.force_registrations.iter().copied() {
            registry.remove(registration_handle);
        }

        Some(body)
    }

    pub fn get(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle.0)
    }

    pub fn get_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(handle.0)
    }

    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyHandle, &RigidBody)> {
        self.bodies.iter().map(|(i, rb)| (RigidBodyHandle(i), rb))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (RigidBodyHandle, &mut RigidBody)> {
        self.bodies
            .iter_mut()
            .map(|(i, rb)| (RigidBodyHandle(i), rb))
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    pub fn get2_mut(
        &mut self,
        handle1: RigidBodyHandle,
        handle2: RigidBodyHandle,
    ) -> (Option<&mut RigidBody>, Option<&mut RigidBody>) {
        self.bodies.get2_mut(handle1.0, handle2.0)
    }
}

impl std::ops::Index<RigidBodyHandle> for RigidBodySet {
    type Output = RigidBody;

    fn index(&self, handle: RigidBodyHandle) -> &Self::Output {
        &self.bodies[handle.0]
    }
}

impl std::ops::IndexMut<RigidBodyHandle> for RigidBodySet {
    fn index_mut(&mut self, index: RigidBodyHandle) -> &mut Self::Output {
        &mut self.bodies[index.0]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RigidBodyHandle(pub Index);
