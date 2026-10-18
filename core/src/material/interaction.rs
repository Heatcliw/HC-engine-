use std::collections::HashMap;
use serde::{Serialize, Deserialize};
use crate::material::MaterialId;


#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct MaterialPair(pub MaterialId, pub MaterialId); 
impl MaterialPair { 
	pub fn new(a: MaterialId, b: MaterialId) -> Self { 
		if a.0 <= b.0 { 
		Self(a, b) 
		} else { 
		Self(b, a) 
		} 
	} 
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MaterialInteraction {
    /// Adhesion coefficient (sticking force)
    pub adhesion: f32,

    /// Pair friction coefficient
    pub friction: f32,

    /// Pair restitution coefficient
    pub restitution: f32,
}
#[derive(Debug, Default)]
pub struct MaterialInteractionRegister {
	interactions: HashMap<MaterialPair, MaterialInteraction>,
}
impl MaterialInteractionRegister {
	pub fn new() -> Self {
		Self::default()
	}
	pub fn register(&mut self, a: MaterialId, b: MaterialId, interaction: MaterialInteraction,) {
		self.interactions.insert(MaterialPair::new(a, b), interaction); 
	} 
	pub fn get(&self, a: MaterialId, b: MaterialId) -> Option<&MaterialInteraction> { 
		self.interactions.get(&MaterialPair::new(a, b)) 
	}

    pub fn contains(&self, a: MaterialId, b: MaterialId) -> bool {
        self.interactions.contains_key(&MaterialPair::new(a, b))
    }
    pub fn remove(&mut self, a: MaterialId, b: MaterialId) {
        self.interactions.remove(&MaterialPair::new(a, b));
    }
}