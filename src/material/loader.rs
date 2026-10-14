use std::path::Path;

use log::info;

use crate::material::{Material, MaterialRegister};

use bevy::{asset::AssetServer,};


pub type LoaderError = Box<dyn std::error::Error + Send + Sync>;

pub fn load_materials_from_dir( dir: &Path, registry: &mut MaterialRegister, asset_server: &AssetServer,) -> Result<(), LoaderError> {
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();

        if path.extension().and_then(|s| s.to_str()) == Some("ron") {
            let text = std::fs::read_to_string(&path)?;
            let mut material: Material = ron::from_str(&text)?;
            if let Some(path) = &material.render.texture {
                info!("Texture loaded: {:?}", path);
            } else {
                info!("No texture");
            }

            info!("Loaded material: {}", material.name);
            registry.register(material);
        }
    }

    Ok(())
}