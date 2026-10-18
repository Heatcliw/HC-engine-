pub mod acoustic;
pub mod objects;
pub mod fluids;
pub mod material;
pub mod math;
pub mod bake;
pub mod world;

#[cfg(feature = "logger")]
pub mod logs;

// удобные re-export
pub use fluids::sph::*;
pub use material::*;
pub use objects::*;

pub use world::*;

#[cfg(feature = "logger")]
pub use logs::build_logger;

pub use math::{constant, geometry::{Shape, Circle,Rectangle,Rhombus,Square,Ellipse,Polygon, Point, MultiPoint}, kernel, vec2};