use serde::{Serialize, Deserialize};
use crate::math::*;
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PhysicsProp {
    /// - Relative density.
    /// Water has a value of 1.0.
    /// This value is not measured in kg/m³.
    ///
    /// Formula:
    /// density = material_density / water_density
    ///
    /// Example:
    /// Stone = 2600 / 1000 = 2.6
    pub density: f32,

    /// - Friction coefficient
    /// Based on real-world material properties.
    /// Used as a simulation parameter.
    pub friction: f32,

    /// - Coefficient of restitution (0.0..1.0).
    /// Describes collision energy retention.
    /// 0.0 = fully inelastic collision.
    /// 1.0 = perfectly elastic collision.
    pub restitution: f32,
}
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AbsorpProb{ 
    pub low: f32,
    pub mid: f32,
    pub high: f32
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AcousticProp {
    /// - Energy absorption coefficient (0.0..1.0).
    /// Defines how much acoustic energy is absorbed by the material.
    /// 0.0 = fully reflective.
    /// 1.0 = fully absorbing.
    pub absorption:AbsorpProb,

    /// - Acoustic energy diffusion coefficient (0.0..1.0).
    /// Defines how scattered the reflected sound is.
    /// 0.0 = mirror-like reflection.
    /// 1.0 = fully diffuse reflection.
    pub diffusion: f32,

    /// - Acoustic impedance.
    /// Represents resistance of a material to acoustic wave propagation.
    /// Based on real-world acoustic properties.
    /// Used as a simulation parameter.
    pub acoustic_impedance: f32,


    pub reflectivity: f32,

    /// - Oscillation damping coefficient (0.0..1.0).
    /// Defines how quickly vibrations lose energy.
    /// 0.0 = no damping.
    /// 1.0 = complete damping.
    pub damping: f32,

    /// - Sound transmission coefficient (0.0..1.0).
    /// Defines how much acoustic energy passes through the material.
    /// 0.0 = fully blocking.
    /// 1.0 = fully transmitting.
    pub transmission: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct FluidProp {
    /// - Dynamic viscosity coefficient.
    /// Represents the resistance of a fluid to flow deformation.
    /// Based on real-world viscosity properties,
    /// but scaled for simulation stability.
    pub viscosity: f32,

    /// - Surface tension coefficient.
    /// Represents the cohesive force at the fluid surface.
    /// Based on real-world surface tension,
    /// but adapted for simulation.
    pub surface_tension: f32,

    /// - Incompressibility coefficient (0.0..1.0).
    /// Defines how resistant the fluid is to volume change.
    /// 0.0 = highly compressible fluid.
    /// 1.0 = nearly incompressible fluid.
    pub incompressibility: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct  RenderProp {
    /// - Base color in RGBA format.
    /// Each component is in the range 0.0..1.0.
    pub color: [f32; 4],

    /// - Texture asset path.
    /// Relative path to the texture file loaded by the renderer.
    pub texture: Option<String>,

    pub size: Option<Vec2>,
    /// - Mesh asset path.
    /// Relative path to the 3D mesh/model file loaded by the renderer.
    pub mesh: Option<String>,
}

#[derive(Default,Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)] 
pub struct MaterialId(pub u32);

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Material {
    // general info
    #[serde(skip)]
    pub id: MaterialId,
    pub name: String,

    // Physics Features
    pub physics: PhysicsProp,

    // Acoustics Features
    pub acoustics: AcousticProp,

    // SPH Behavior
    pub fluid: Option<FluidProp>,
    // Render Features
    pub render: RenderProp     
}