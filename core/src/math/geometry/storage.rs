use rand::Rng;
use crate::math::Vec2;

#[derive(Default,Debug,Clone)]
pub struct Transform {
    pub center: Vec2,
    pub rotation: f32,
    pub scale: Vec2,
}

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShapeKind {
    Point,

    MultiPoint,

    Line,

    Circle,
    Ellipse,
    Rectangle,
    Square,
    Rhombus,
    Polygon,

    Triangle,
}

#[derive(Debug,Clone)]
pub enum Shape {
    Point(Point),

    MultiPoint(MultiPoint),
    
    Line(Line),

    Circle(Circle),
    Ellipse(Ellipse),
    Square(Square),
    Rectangle (Rectangle),
    Rhombus(Rhombus),
    Polygon(Polygon),

    Triangle(Triangle)
}
#[derive(Debug,Clone)]
pub struct Point {
    pub positions: Vec2,
}

#[derive(Debug,Clone)]
pub struct MultiPoint { 
    pub positions: Vec<Vec2>, 
}

#[derive(Debug,Clone)]
pub struct Line{
    pub start: Vec2,
    pub end: Vec2,
    pub count: usize,
}

#[derive(Debug,Clone)]
pub struct Square {
    pub side: f32,
}

#[derive(Debug,Clone)]
pub struct Rectangle {
    pub width: f32,
    pub height: f32,
}

#[derive(Debug,Clone,Copy)]
pub struct Circle {
    pub radius: f32,
}

#[derive(Debug,Clone,Copy)]
pub struct Rhombus {
    pub side: f32,
    pub angle: f32,
}

#[derive(Debug,Clone)]
pub struct Parallelogram {
    pub width: f32,
    pub height: f32,
}

#[derive(Debug,Clone)]
pub struct Ellipse {
    pub radius_x: f32,
    pub radius_y: f32,
    pub delta: Vec2
}

#[derive(Debug,Clone)]
pub struct EquilateralTriangle {
    pub side: f32,
}

#[derive(Debug,Clone)]
pub struct IsoscelesTriangle {
    pub base: f32,
    pub side: f32,
}

#[derive(Debug,Clone)]
pub struct Triangle {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

#[derive(Debug,Clone)]
pub struct Polygon {
    pub local_vectors: Vec<Vec2>
}

pub trait HasArea {
    fn area(&self) -> f32;
}

pub trait HasPerimeter {
    fn perimeter(&self) -> f32;
}

pub trait HasDiagonal {
    type Output; 
    fn diagonals(&self) -> Self::Output;
}

pub trait HasDiameter {
     type Output; 
    fn diameters(&self) -> Self::Output;
}

pub trait HasHeight {
    type Output; 
    /// Высота, опущенная на сторону A (или единственная фиксированная высота)
    fn heights(&self) -> Self::Output;
}

pub trait HasRadius {
    type Output;
    fn radius(&self) -> Self::Output;
}

pub trait HasVertices {
    type Output; 
    fn vertices(&self) -> Self::Output;
}

pub trait HasAabb {
    fn aabb(&self, transform: &Transform) -> Aabb;
}

pub trait SamplePoint {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2;
}

pub trait SampleBoundaryPoint {
    fn sample_boundary_point(&self, rng: &mut impl Rng) -> Vec2;
}