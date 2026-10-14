use crate::{body::collision::storage::Collider, math::*};

use crate::body::collision::narrow_algorithms::{
    PointPoint, PointMultiPoint, PointLine, PointCircle,
    PointEllipse, PointSquare, PointRectangle, PointRhombus,
    PointPolygon, PointTriangle, MultiPointMultiPoint, MultiPointLine,
    MultiPointCircle, MultiPointEllipse, MultiPointSquare, MultiPointRectangle,
    MultiPointRhombus, MultiPointPolygon, MultiPointTriangle, LineLine,
    LineCircle, LineEllipse, LineSquare, LineRectangle, LineRhombus,
    LinePolygon, LineTriangle, CircleCircle, CircleEllipse, CircleSquare,
    CircleRectangle, CircleRhombus, CirclePolygon, CircleTriangle,
    EllipseEllipse, EllipseSquare, EllipseRectangle, EllipseRhombus,
    EllipsePolygon, EllipseTriangle, SquareSquare, SquareRectangle,
    SquareRhombus, SquarePolygon, SquareTriangle, RectangleRectangle,
    RectangleRhombus, RectanglePolygon, RectangleTriangle, RhombusRhombus,
    RhombusPolygon, RhombusTriangle, PolygonPolygon, PolygonTriangle,
    TriangleTriangle
};

impl Shape {
    pub fn kind(&self) -> ShapeKind {
        match self {
            Shape::Point(_) => ShapeKind::Point,
            Shape::MultiPoint(_) => ShapeKind::MultiPoint,
            Shape::Line(_) => ShapeKind::Line,

            Shape::Circle(_) => ShapeKind::Circle,
            Shape::Ellipse(_) => ShapeKind::Ellipse,
            Shape::Rectangle(_) => ShapeKind::Rectangle,
            Shape::Square(_) => ShapeKind::Square,
            Shape::Rhombus(_) => ShapeKind::Rhombus,
            Shape::Polygon(_) => ShapeKind::Polygon,
            Shape::Triangle(_) => ShapeKind::Triangle,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum CollisionRelation {
    Outside,
    Touching,
    Overlapping,
    Inside,
}
#[derive(Debug, Clone)]
pub struct CollisionInfo {
    pub relation: CollisionRelation,
    pub contact: Option<Contact>,
}

#[derive(Debug, Clone)]
pub struct Contact {
    pub depth: f32,
    pub normal: Vec2,
    pub points: Vec<Vec2>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ShapePair(pub ShapeKind, pub ShapeKind);

impl ShapePair {
    pub fn new(a: ShapeKind, b: ShapeKind) -> Self {
        if (a as u8) <= (b as u8) {
            Self(a, b)
        } else {
            Self(b, a)
        }
    }
    pub fn sort<'a>(&self, a: &'a Collider, b: &'a Collider) -> (&'a Collider, &'a Collider){
        if a.kind() == self.0 {
            (a,b)
        } else {
            (b,a)  
        }
    }
}

pub trait NarrowPhaseAlgorithm<A, B> {
    fn collide(a: (&A, &Transform), b: (&B, &Transform)) -> CollisionInfo;
}

pub fn collide(a: &Collider, b: &Collider) -> CollisionInfo {
    let pair = ShapePair::new(a.geometry.kind(), b.geometry.kind());
    let (a, b) = pair.sort(a, b);
    match pair {
        ShapePair(ShapeKind::Point, ShapeKind::Point) => PointPoint::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Point(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::MultiPoint) => PointMultiPoint::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::MultiPoint(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Line) => PointLine::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Line(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Circle) => PointCircle::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Circle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Ellipse) => PointEllipse::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Ellipse(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Square) => PointSquare::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Square(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Rectangle) => PointRectangle::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rectangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Rhombus) => PointRhombus::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Polygon) => PointPolygon::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Point, ShapeKind::Triangle) => PointTriangle::collide{
            a:(match a.geometry { 
                Shape::Point(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::MultiPoint, ShapeKind::MultiPoint) => MultiPointMultiPoint::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::MultiPoint(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Line) => MultiPointLine::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Line(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Circle) => MultiPointCircle::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Circle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Ellipse) => MultiPointEllipse::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Ellipse(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Square) => MultiPointSquare::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Square(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Rectangle) => MultiPointRectangle::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rectangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Rhombus) => MultiPointRhombus::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Polygon) => MultiPointPolygon::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::MultiPoint, ShapeKind::Triangle) => MultiPointTriangle::collide{
            a:(match a.geometry { 
                Shape::MultiPoint(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::Line, ShapeKind::Line) => LineLine::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Line(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Line, ShapeKind::Circle) => LineCircle::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Circle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Line, ShapeKind::Ellipse) => LineEllipse::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Ellipse(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Line, ShapeKind::Square) => LineSquare::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Square(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Line, ShapeKind::Rectangle) => LineRectangle::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rectangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Line, ShapeKind::Rhombus) => LineRhombus::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Line, ShapeKind::Polygon) => LinePolygon::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Line, ShapeKind::Triangle) => LineTriangle::collide{
            a:(match a.geometry { 
                Shape::Line(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::Circle, ShapeKind::Circle) => CircleCircle::collide{
            a:(match a.geometry { 
                Shape::Circle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Circle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Circle, ShapeKind::Ellipse) => CircleEllipse::collide{
            a:(match a.geometry { 
                Shape::Circle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Ellipse(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Circle, ShapeKind::Square) => CircleSquare::collide{
            a:(match a.geometry { 
                Shape::Circle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Square(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Circle, ShapeKind::Rectangle) => CircleRectangle::collide{
            a:(match a.geometry { 
                Shape::Circle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rectangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Circle, ShapeKind::Rhombus) => CircleRhombus::collide{
            a:(match a.geometry { 
                Shape::Circle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Circle, ShapeKind::Polygon) => CirclePolygon::collide{
            a:(match a.geometry { 
                Shape::Circle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Circle, ShapeKind::Triangle) => CircleTriangle::collide{
            a:(match a.geometry { 
                Shape::Circle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::Ellipse, ShapeKind::Ellipse) => EllipseEllipse::collide{
            a:(match a.geometry { 
                Shape::Ellipse(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Ellipse(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Ellipse, ShapeKind::Square) => EllipseSquare::collide{
            a:(match a.geometry { 
                Shape::Ellipse(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Square(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Ellipse, ShapeKind::Rectangle) => EllipseRectangle::collide{
            a:(match a.geometry { 
                Shape::Ellipse(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rectangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Ellipse, ShapeKind::Rhombus) => EllipseRhombus::collide{
            a:(match a.geometry { 
                Shape::Ellipse(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Ellipse, ShapeKind::Polygon) => EllipsePolygon::collide{
            a:(match a.geometry { 
                Shape::Ellipse(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Ellipse, ShapeKind::Triangle) => EllipseTriangle::collide{
            a:(match a.geometry { 
                Shape::Ellipse(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::Square, ShapeKind::Square) => SquareSquare::collide{
            a:(match a.geometry { 
                Shape::Square(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Square(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Square, ShapeKind::Rectangle) => SquareRectangle::collide{
            a:(match a.geometry { 
                Shape::Square(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rectangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Square, ShapeKind::Rhombus) => SquareRhombus::collide{
            a:(match a.geometry { 
                Shape::Square(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Square, ShapeKind::Polygon) => SquarePolygon::collide{
            a:(match a.geometry { 
                Shape::Square(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Square, ShapeKind::Triangle) => SquareTriangle::collide{
            a:(match a.geometry { 
                Shape::Square(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::Rectangle, ShapeKind::Rectangle) => RectangleRectangle::collide{
            a:(match a.geometry { 
                Shape::Rectangle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rectangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Rectangle, ShapeKind::Rhombus) => RectangleRhombus::collide{
            a:(match a.geometry { 
                Shape::Rectangle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Rectangle, ShapeKind::Polygon) => RectanglePolygon::collide{
            a:(match a.geometry { 
                Shape::Rectangle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Rectangle, ShapeKind::Triangle) => RectangleTriangle::collide{
            a:(match a.geometry { 
                Shape::Rectangle(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


       ShapePair(ShapeKind::Rhombus, ShapeKind::Rhombus) => RhombusRhombus::collide{
            a:(match a.geometry { 
                Shape::Rhombus(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Rhombus(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Rhombus, ShapeKind::Polygon) => RhombusPolygon::collide{
            a:(match a.geometry { 
                Shape::Rhombus(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Rhombus, ShapeKind::Triangle) => RhombusTriangle::collide{
            a:(match a.geometry { 
                Shape::Rhombus(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::Polygon, ShapeKind::Polygon) => PolygonPolygon::collide{
            a:(match a.geometry { 
                Shape::Polygon(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Polygon(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },

        ShapePair(ShapeKind::Polygon, ShapeKind::Triangle) => PolygonTriangle::collide{
            a:(match a.geometry { 
                Shape::Polygon(p) => p, 
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform), 
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        },


        ShapePair(ShapeKind::Triangle, ShapeKind::Triangle) => TriangleTriangle::collide{
            a: (match a.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, a.transform),
            b: (match b.geometry {
                Shape::Triangle(p) => p,
                _ => return CollisionInfo { relation: CollisionRelation::Outside, contact: None }}, b.transform),
        }
    }
}



