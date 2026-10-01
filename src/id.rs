//! Opaque registry identities. Handles are not native Rapier handles.
use std::sync::atomic::{AtomicU64, Ordering};

/// Caller-assigned identity. Use a different value for each distinct world.
/// Reusing a value for another world cannot be detected reliably by this package.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct WorldId(pub u64);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RopeSetId(u64);

impl RopeSetId {
    pub fn value(self) -> u64 {
        self.0
    }
    pub(crate) fn fresh() -> Option<Self> {
        static NEXT: AtomicU64 = AtomicU64::new(1);
        NEXT.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_add(1))
            .ok()
            .map(Self)
    }
}

macro_rules! handle {
    ($name:ident) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name {
            pub(crate) set: RopeSetId,
            pub(crate) slot: u32,
            pub(crate) generation: u64,
        }
        impl $name {
            pub fn set_id(self) -> RopeSetId {
                self.set
            }
            pub fn slot(self) -> u32 {
                self.slot
            }
            pub fn generation(self) -> u64 {
                self.generation
            }
        }
    };
}
handle!(RopeHandle);
handle!(HarnessHandle);
handle!(AttachmentHandle);

#[derive(Debug)]
struct Slot<T> {
    generation: u64,
    value: Option<T>,
}

/// Retires an exhausted generation rather than wrapping it into a valid old ID.
#[derive(Debug)]
pub(crate) struct Arena<T> {
    slots: Vec<Slot<T>>,
    free: Vec<u32>,
    len: usize,
}

impl<T> Default for Arena<T> {
    fn default() -> Self {
        Self {
            slots: Vec::new(),
            free: Vec::new(),
            len: 0,
        }
    }
}

impl<T> Arena<T> {
    pub fn len(&self) -> usize {
        self.len
    }
    pub fn can_insert(&self, count: usize) -> bool {
        count <= self.free.len()
            || self
                .slots
                .len()
                .checked_add(count - self.free.len())
                .is_some_and(|n| n <= u32::MAX as usize)
    }
    pub fn insert(&mut self, value: T) -> Option<(u32, u64)> {
        if !self.can_insert(1) {
            return None;
        }
        let index = if let Some(index) = self.free.pop() {
            index
        } else {
            let index = self.slots.len() as u32;
            self.slots.push(Slot {
                generation: 1,
                value: None,
            });
            index
        };
        let slot = &mut self.slots[index as usize];
        slot.value = Some(value);
        self.len += 1;
        Some((index, slot.generation))
    }
    pub fn get(&self, index: u32, generation: u64) -> Option<&T> {
        self.slots
            .get(index as usize)
            .filter(|s| s.generation == generation)?
            .value
            .as_ref()
    }
    pub fn get_mut(&mut self, index: u32, generation: u64) -> Option<&mut T> {
        self.slots
            .get_mut(index as usize)
            .filter(|s| s.generation == generation)?
            .value
            .as_mut()
    }
    pub fn remove(&mut self, index: u32, generation: u64) -> Option<T> {
        let slot = self
            .slots
            .get_mut(index as usize)
            .filter(|s| s.generation == generation)?;
        let value = slot.value.take()?;
        self.len -= 1;
        if let Some(next) = slot.generation.checked_add(1) {
            slot.generation = next;
            self.free.push(index);
        }
        Some(value)
    }
    pub fn iter(&self) -> impl Iterator<Item = (u32, u64, &T)> {
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(i, s)| s.value.as_ref().map(|v| (i as u32, s.generation, v)))
    }
}
