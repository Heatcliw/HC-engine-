use crate::body::collision::storage::*;
use std::collections::HashMap;

#[derive(Default,Debug)]
pub struct ColliderRegister {
    colliders: Vec<Collider>,
    name_to_id: HashMap<String, ColliderId>,
}

impl ColliderRegister {
    pub fn new() -> Self {
		Self::default()
	}
    pub fn register(&mut self, mut collider:Collider) -> ColliderId {
		
		let id = ColliderId(self.colliders.len() as usize);
		collider.id = id;
		self.name_to_id.insert(collider.name.clone(), id); 
		self.colliders.push(collider); 
		id
	}
    pub fn get(&self, id: ColliderId) -> Option<&Collider> { 
		self.colliders.get(id.0 as usize) 
	} 
	pub fn get_by_name(&self, name: &str) -> Option<&Collider> { 
		let id = self.name_to_id.get(name)?; 
		self.get(*id) 
	}

    pub fn len(&self) -> usize {
        self.colliders.len()
    }

    pub fn is_empty(&self) -> bool {
        self.colliders.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Collider> {
        self.colliders.iter()
    }
}