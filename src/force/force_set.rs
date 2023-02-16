use generational_arena::{Arena, Index};

use crate::ForceGenerator;

#[derive(Default)]
pub struct ForceGeneratorSet {
    bodies: Arena<Box<dyn ForceGenerator>>,
}

impl ForceGeneratorSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, generator: Box<dyn ForceGenerator>) -> ForceGeneratorHandle {
        let index = self.bodies.insert(generator);
        ForceGeneratorHandle(index)
    }

    pub fn remove(&mut self, handle: ForceGeneratorHandle) -> Option<Box<dyn ForceGenerator>> {
        self.bodies.remove(handle.0)
    }

    pub fn get(&self, handle: ForceGeneratorHandle) -> Option<&dyn ForceGenerator> {
        self.bodies.get(handle.0).map(|b| &**b)
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }
}

impl std::ops::Index<ForceGeneratorHandle> for ForceGeneratorSet {
    type Output = dyn ForceGenerator;

    fn index(&self, handle: ForceGeneratorHandle) -> &Self::Output {
        &*self.bodies[handle.0]
    }
}

impl std::ops::IndexMut<ForceGeneratorHandle> for ForceGeneratorSet {
    fn index_mut(&mut self, handle: ForceGeneratorHandle) -> &mut Self::Output {
        &mut *self.bodies[handle.0]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ForceGeneratorHandle(pub Index);
