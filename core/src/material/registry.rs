use std::collections::HashMap;
use log::{debug, info, warn};
use std::path::Path;
use std::error::Error;

use crate::material::{Material, MaterialId};

/// Centralized storage for all material assets. 
/// Provides O(1) access by `MaterialId` and lookup by name.
#[derive(Debug, Default)] 
pub struct MaterialRegister {
    materials: Vec<Material>,
    name_to_id: HashMap<String, MaterialId>,
}

pub type LoaderError = Box<dyn Error + Send + Sync>;

impl MaterialRegister {
	pub fn new() -> Self {
		Self::default()
		
	}

	pub fn load(path: &Path) -> Result<Self, LoaderError> {
		let mut register= Self::new();
		for entry in std::fs::read_dir(path)? {
			let path = entry.expect("REASON").path();
			debug!("loading material file: {:?}", path);
			if path.extension().and_then(|s| s.to_str()) == Some("ron") {
				let text = std::fs::read_to_string(&path);
				let material: Material = ron::from_str(&text?).expect("REASON");
				if let Some(path) = &material.render.texture {
					debug!("Texture loaded: {:?}", path);
				} else {
					warn!("No texture");
				}

				info!("Loaded material: {}", material.name);
				register.register(material);
			}
		}

		Ok(register)
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