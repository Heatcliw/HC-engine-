use std::collections::HashMap;

use crate::material::{Material, MaterialId};

/// Centralized storage for all material assets. 
/// Provides O(1) access by `MaterialId` and lookup by name.
#[derive(Debug, Default)] 
pub struct MaterialRegister {
    materials: Vec<Material>,
    name_to_id: HashMap<String, MaterialId>,
}

impl MaterialRegister {
	pub fn new() -> Self {
		Self::default()
	}
	pub fn register(&mut self, mut material:Material) -> MaterialId {
		
		let id = MaterialId(self.materials.len() as u32);
		material.id = id;
		self.name_to_id.insert(material.name.clone(), id); 
		self.materials.push(material); 
		id
	}
	pub fn get(&self, id: MaterialId) -> Option<&Material> { 
		self.materials.get(id.0 as usize) 
	} 
	pub fn get_by_name(&self, name: &str) -> Option<&Material> { 
		let id = self.name_to_id.get(name)?; 
		self.get(*id) 
	}

    pub fn len(&self) -> usize {
        self.materials.len()
    }

    pub fn is_empty(&self) -> bool {
        self.materials.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Material> {
        self.materials.iter()
    }
}