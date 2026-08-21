pub mod acoustic;
pub mod body;
pub mod fluids;
pub mod material;
pub mod math;
pub mod bake;
pub mod world;

// удобные re-export
pub use fluids::sph::*;
pub use material::*;
pub use world::*;
pub use math::{constant, geometry::{Shape, Circle,Rectangle,Rhombus,Square,Ellipse,Polygon}, kernel, vec2};