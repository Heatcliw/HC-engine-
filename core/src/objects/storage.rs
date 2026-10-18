use crate::{MaterialId, math::*, };
use crate::objects::collision::storage::*;
pub struct ObjectId(pub u32);

pub enum BodyType {
    Static,
    Dynamic,
    Kinematic
}

pub struct ObjectPhysicProp {
    pub mass: f32,

    pub velocity: Vec2,

    pub angular_velocity: f32,

}

pub struct Object {
    pub id: ObjectId,

    pub name: String,

    pub transform: Transform,

    pub collider: ColliderId,
    
    pub material: MaterialId,
    
    pub body_type: BodyType,

    pub physic_prop: Option<ObjectPhysicProp>,
}