use serde::{Deserialize, Serialize};

use crate::math::*;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SphConfig {
    pub smoothing_length: f32,
    pub rest_density: f32,
    pub stiffness: f32,
    pub viscosity: f32,
    pub gravity: Vec2,
}

impl SphConfig {
    pub fn load_from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let text = std::fs::read_to_string(path)?;
        let config = ron::from_str(&text)?;
        Ok(config)
    }

    pub fn builtin() -> Self {
        Self {
            smoothing_length: 16.0,
            rest_density: 1.0,
            stiffness: 200.0,
            viscosity: 0.15,
            gravity: Vec2 {x: 0.0, y: -9.81},
        }
    }
}

impl Default for SphConfig {
    fn default() -> Self {
        match Self::load_from_file("assets/config.ron") {
            Ok(cfg) => {
                println!("Loaded config from assets/config.ron");
                cfg
            }
            Err(err) => {
                eprintln!(
                    "Failed to load config: {err}. Using builtin defaults."
                );
                Self::builtin()
            }
        }
    }
}