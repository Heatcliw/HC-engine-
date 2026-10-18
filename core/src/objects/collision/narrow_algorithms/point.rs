use crate::{objects::{NarrowPhaseAlgorithm, CollisionInfo, CollisionRelation, Contact}, Point, math::*};

/// Performs narrow-phase collision detection between two points.
///
/// The point is located at the center of its transform.
/// The points collide only when the distance between them is
/// within the tolerance.
pub struct PointPoint;

impl NarrowPhaseAlgorithm<Point, Point> for PointPoint {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Point, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (_poi_b, trans_b) = (b.0, b.1);

        let distance = length(sub(trans_a.center, trans_b.center));

        if distance > EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Touching,
            contact: Some(Contact {
                depth: 0.0,
                normal: VEC2_X,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a
/// multi-point body.
///
/// The point is located at the center of its transform.
/// All points of the multi-point body share the center of its
/// transform, so the test reduces to a point-to-point distance check.
pub struct PointMultiPoint;

impl NarrowPhaseAlgorithm<Point, MultiPoint> for PointMultiPoint {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&MultiPoint, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (mul_b, trans_b) = (b.0, b.1);

        if mul_b.positions.is_empty() {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let distance = length(sub(trans_a.center, trans_b.center));

        if distance > EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Touching,
            contact: Some(Contact {
                depth: 0.0,
                normal: VEC2_X,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a line.
///
/// The point is located at the center of its transform.
/// The closest point on the segment is determined and compared with
/// the tolerance.
pub struct PointLine;

impl NarrowPhaseAlgorithm<Point, Line> for PointLine {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Line, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (lin_b, trans_b) = (b.0, b.1);

        let start = add(
            trans_b.center,
            rotate(
                Vec2 {
                    x: lin_b.start.x * trans_b.scale.x.abs(),
                    y: lin_b.start.y * trans_b.scale.y.abs(),
                },
                trans_b.rotation,
            ),
        );

        let end = add(
            trans_b.center,
            rotate(
                Vec2 {
                    x: lin_b.end.x * trans_b.scale.x.abs(),
                    y: lin_b.end.y * trans_b.scale.y.abs(),
                },
                trans_b.rotation,
            ),
        );

        let line = sub(end, start);
        let delta = sub(trans_a.center, start);

        let line_length_sq = dot(line, line);

        let t = if line_length_sq > SCALAR_ZERO {
            (dot(delta, line) / line_length_sq)
                .clamp(0.0, 1.0)
        } else {
            0.0
        };

        let closest_point = add(start, scale(line, t));

        let distance = length(sub(trans_a.center, closest_point));

        if distance > EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = if length(line) > SCALAR_ZERO {
            normalize(Vec2 {
                x: -line.y,
                y: line.x,
            })
        } else {
            VEC2_X
        };

        CollisionInfo {
            relation: CollisionRelation::Touching,
            contact: Some(Contact {
                depth: 0.0,
                normal,
                points: vec![closest_point],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a circle.
///
/// The point is located at the center of its transform.
/// The distance from the point to the circle center is compared with
/// the circle radius.
pub struct PointCircle;

impl NarrowPhaseAlgorithm<Point, Circle> for PointCircle {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Circle, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (cir_b, trans_b) = (b.0, b.1);

        let radius = cir_b.radius * trans_b.scale.x.abs();

        let delta = sub(trans_a.center, trans_b.center);
        let distance = length(delta);

        if distance > radius + EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = if distance > SCALAR_ZERO {
            div(delta, distance)
        } else {
            VEC2_X
        };

        if (distance - radius).abs() <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Inside,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and an ellipse.
///
/// The point is located at the center of its transform.
/// The distance from the point to the ellipse center is compared with
/// the ellipse radius.
pub struct PointEllipse;

impl NarrowPhaseAlgorithm<Point, Ellipse> for PointEllipse {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (eli_b, trans_b) = (b.0, b.1);

        let radius = eli_b.radius();

        let delta = sub(trans_a.center, trans_b.center);
        let distance = length(delta);

        if distance > radius + EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = if distance > SCALAR_ZERO {
            div(delta, distance)
        } else {
            VEC2_X
        };

        if (distance - radius).abs() <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Inside,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a square.
///
/// The point is located at the center of its transform.
/// The point is transformed into the square's local coordinate system
/// and its distances to the sides are used to find the penetration depth.
pub struct PointSquare;

impl NarrowPhaseAlgorithm<Point, Square> for PointSquare {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Square, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (squ_b, trans_b) = (b.0, b.1);

        let half_width = squ_b.side * trans_b.scale.x.abs() * 0.5;
        let half_height = squ_b.side * trans_b.scale.x.abs() * 0.5;

        let local_point = rotate(
            sub(trans_a.center, trans_b.center),
            -trans_b.rotation,
        );

        let gap_x = half_width - local_point.x.abs();
        let gap_y = half_height - local_point.y.abs();

        if gap_x < -EPSILON || gap_y < -EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal_local = if gap_x <= gap_y {
            Vec2 {
                x: if local_point.x < 0.0 { -1.0 } else { 1.0 },
                y: 0.0,
            }
        } else {
            Vec2 {
                x: 0.0,
                y: if local_point.y < 0.0 { -1.0 } else { 1.0 },
            }
        };

        let normal = rotate(normal_local, trans_b.rotation);

        let depth = gap_x.min(gap_y);

        if depth <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Inside,
            contact: Some(Contact {
                depth,
                normal,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a rectangle.
///
/// The point is located at the center of its transform.
/// The point is transformed into the rectangle's local coordinate
/// system and its distances to the sides are used to find the penetration depth.
pub struct PointRectangle;

impl NarrowPhaseAlgorithm<Point, Rectangle> for PointRectangle {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (rec_b, trans_b) = (b.0, b.1);

        let half_width = rec_b.width * trans_b.scale.x.abs() * 0.5;
        let half_height = rec_b.height * trans_b.scale.y.abs() * 0.5;

        let local_point = rotate(
            sub(trans_a.center, trans_b.center),
            -trans_b.rotation,
        );

        let gap_x = half_width - local_point.x.abs();
        let gap_y = half_height - local_point.y.abs();

        if gap_x < -EPSILON || gap_y < -EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal_local = if gap_x <= gap_y {
            Vec2 {
                x: if local_point.x < 0.0 { -1.0 } else { 1.0 },
                y: 0.0,
            }
        } else {
            Vec2 {
                x: 0.0,
                y: if local_point.y < 0.0 { -1.0 } else { 1.0 },
            }
        };

        let normal = rotate(normal_local, trans_b.rotation);

        let depth = gap_x.min(gap_y);

        if depth <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Inside,
            contact: Some(Contact {
                depth,
                normal,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a rhombus.
///
/// The point is located at the center of its transform.
/// The point is transformed into the rhombus's local coordinate system.
/// The closest edge gives the distance, the normal and the containment test.
pub struct PointRhombus;

impl NarrowPhaseAlgorithm<Point, Rhombus> for PointRhombus {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (rho_b, trans_b) = (b.0, b.1);

        let half_diagonal_x_b =
            rho_b.side * (rho_b.angle * 0.5).cos()
            * trans_b.scale.x.abs();

        let half_diagonal_y_b =
            rho_b.side * (rho_b.angle * 0.5).sin()
            * trans_b.scale.y.abs();

        let local_b: Vec<Vec2> = vec![
            Vec2 { x: 0.0, y: half_diagonal_y_b },
            Vec2 { x: half_diagonal_x_b, y: 0.0 },
            Vec2 { x: 0.0, y: -half_diagonal_y_b },
            Vec2 { x: -half_diagonal_x_b, y: 0.0 },
        ];

        let local_point = rotate(
            sub(trans_a.center, trans_b.center),
            -trans_b.rotation,
        );

        let count = local_b.len();

        let mut centroid = VEC2_ZERO;

        for vertex in &local_b {
            centroid = add(centroid, *vertex);
        }

        centroid = scale(centroid, 1.0 / count as f32);

        let mut closest_point = VEC2_ZERO;
        let mut closest_normal = VEC2_X;
        let mut min_distance_sq = f32::MAX;

        for i in 0..count {
            let start = local_b[i];
            let end = local_b[(i + 1) % count];

            let edge = sub(end, start);
            let point = sub(local_point, start);

            let edge_length_sq = dot(edge, edge);

            let t = if edge_length_sq > SCALAR_ZERO {
                (dot(point, edge) / edge_length_sq)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };

            let point_on_edge = add(start, scale(edge, t));

            let diff = sub(local_point, point_on_edge);
            let distance_sq = dot(diff, diff);

            if distance_sq < min_distance_sq {
                min_distance_sq = distance_sq;
                closest_point = point_on_edge;

                let edge_normal = if edge_length_sq > SCALAR_ZERO {
                    normalize(Vec2 {
                        x: edge.y,
                        y: -edge.x,
                    })
                } else {
                    VEC2_X
                };

                closest_normal = if dot(edge_normal, sub(point_on_edge, centroid)) < 0.0 {
                    negative(edge_normal)
                } else {
                    edge_normal
                };
            }
        }

        let distance = min_distance_sq.sqrt();

        let mut inside = false;

        for i in 0..count {
            let vertex_start = local_b[i];
            let vertex_end = local_b[(i + 1) % count];

            if (vertex_start.y > local_point.y) != (vertex_end.y > local_point.y) {
                let x = (vertex_end.x - vertex_start.x)
                    * (local_point.y - vertex_start.y)
                    / (vertex_end.y - vertex_start.y)
                    + vertex_start.x;

                if local_point.x < x {
                    inside = !inside;
                }
            }
        }

        if !inside && distance > EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = rotate(closest_normal, trans_b.rotation);

        if distance <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![add(
                        trans_b.center,
                        rotate(closest_point, trans_b.rotation),
                    )],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Inside,
            contact: Some(Contact {
                depth: distance,
                normal,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a polygon.
///
/// The point is located at the center of its transform.
/// The point is transformed into the polygon's local coordinate system.
/// The closest edge gives the distance and the normal, and the ray casting
/// test determines whether the point is inside the polygon.
pub struct PointPolygon;

impl NarrowPhaseAlgorithm<Point, Polygon> for PointPolygon {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (pol_b, trans_b) = (b.0, b.1);

        let local_b: Vec<Vec2> = pol_b
            .local_vectors
            .iter()
            .map(|vertex| Vec2 {
                x: vertex.x * trans_b.scale.x.abs(),
                y: vertex.y * trans_b.scale.y.abs(),
            })
            .collect();

        if local_b.len() < 3 {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let local_point = rotate(
            sub(trans_a.center, trans_b.center),
            -trans_b.rotation,
        );

        let count = local_b.len();

        let mut centroid = VEC2_ZERO;

        for vertex in &local_b {
            centroid = add(centroid, *vertex);
        }

        centroid = scale(centroid, 1.0 / count as f32);

        let mut closest_point = VEC2_ZERO;
        let mut closest_normal = VEC2_X;
        let mut min_distance_sq = f32::MAX;

        for i in 0..count {
            let start = local_b[i];
            let end = local_b[(i + 1) % count];

            let edge = sub(end, start);
            let point = sub(local_point, start);

            let edge_length_sq = dot(edge, edge);

            let t = if edge_length_sq > SCALAR_ZERO {
                (dot(point, edge) / edge_length_sq)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };

            let point_on_edge = add(start, scale(edge, t));

            let diff = sub(local_point, point_on_edge);
            let distance_sq = dot(diff, diff);

            if distance_sq < min_distance_sq {
                min_distance_sq = distance_sq;
                closest_point = point_on_edge;

                let edge_normal = if edge_length_sq > SCALAR_ZERO {
                    normalize(Vec2 {
                        x: edge.y,
                        y: -edge.x,
                    })
                } else {
                    VEC2_X
                };

                closest_normal = if dot(edge_normal, sub(point_on_edge, centroid)) < 0.0 {
                    negative(edge_normal)
                } else {
                    edge_normal
                };
            }
        }

        let distance = min_distance_sq.sqrt();

        let mut inside = false;

        for i in 0..count {
            let vertex_start = local_b[i];
            let vertex_end = local_b[(i + 1) % count];

            if (vertex_start.y > local_point.y) != (vertex_end.y > local_point.y) {
                let x = (vertex_end.x - vertex_start.x)
                    * (local_point.y - vertex_start.y)
                    / (vertex_end.y - vertex_start.y)
                    + vertex_start.x;

                if local_point.x < x {
                    inside = !inside;
                }
            }
        }

        if !inside && distance > EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = rotate(closest_normal, trans_b.rotation);

        if distance <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![add(
                        trans_b.center,
                        rotate(closest_point, trans_b.rotation),
                    )],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Inside,
            contact: Some(Contact {
                depth: distance,
                normal,
                points: vec![trans_a.center],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a point and a triangle.
///
/// The point is located at the center of its transform.
/// The triangle is reconstructed from its side lengths. The closest
/// edge gives the distance and the normal, and the ray casting test
/// determines whether the point is inside the triangle.
pub struct PointTriangle;

impl NarrowPhaseAlgorithm<Point, Triangle> for PointTriangle {
    /// Determines the collision relation between the specified point
    /// and shape and calculates contact information when applicable.
    fn collide(a: (&Point, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {
        let (_poi_a, trans_a) = (a.0, a.1);
        let (tri_b, trans_b) = (b.0, b.1);

        let sides_b = [
            tri_b.a * trans_b.scale.x.abs(),
            tri_b.b * trans_b.scale.x.abs(),
            tri_b.c * trans_b.scale.x.abs(),
        ];

        let vertex_b_c_x =
            (sides_b[1] * sides_b[1]
                + sides_b[2] * sides_b[2]
                - sides_b[0] * sides_b[0])
            / (2.0 * sides_b[2]);

        let local_b: Vec<Vec2> = vec![
            VEC2_ZERO,
            Vec2 {
                x: sides_b[2],
                y: 0.0,
            },
            Vec2 {
                x: vertex_b_c_x,
                y: (sides_b[1] * sides_b[1]
                    - vertex_b_c_x * vertex_b_c_x)
                    .max(0.0)
                    .sqrt(),
            },
        ];

        let local_point = rotate(
            sub(trans_a.center, trans_b.center),
            -trans_b.rotation,
        );

        let count = local_b.len();

        let mut centroid = VEC2_ZERO;

        for vertex in &local_b {
            centroid = add(centroid, *vertex);
        }

        centroid = scale(centroid, 1.0 / count as f32);

        let mut closest_point = VEC2_ZERO;
        let mut closest_normal = VEC2_X;
        let mut min_distance_sq = f32::MAX;

        for i in 0..count {
            let start = local_b[i];
            let end = local_b[(i + 1) % count];

            let edge = sub(end, start);
            let point = sub(local_point, start);

            let edge_length_sq = dot(edge, edge);

            let t = if edge_length_sq > SCALAR_ZERO {
                (dot(point, edge) / edge_length_sq)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };

            let point_on_edge = add(start, scale(edge, t));

            let diff = sub(local_point, point_on_edge);
            let distance_sq = dot(diff, diff);

            if distance_sq < min_distance_sq {
                min_distance_sq = distance_sq;
                closest_point = point_on_edge;

                let edge_normal = if edge_length_sq > SCALAR_ZERO {
                    normalize(Vec2 {
                        x: edge.y,
                        y: -edge.x,
                    })
                } else {
                    VEC2_X
                };

                closest_normal = if dot(edge_normal, sub(point_on_edge, centroid)) < 0.0 {
                    negative(edge_normal)
                } else {
                    edge_normal
                };
            }
        }

        let distance = min_distance_sq.sqrt();

        let mut inside = false;

        for i in 0..count {
            let vertex_start = local_b[i];
            let vertex_end = local_b[(i + 1) % count];

            if (vertex_start.y > local_point.y) != (vertex_end.y > local_point.y) {
                let x = (vertex_end.x - vertex_start.x)
                    * (local_point.y - vertex_start.y)
                    / (vertex_end.y - vertex_start.y)
                    + vertex_start.x;

                if local_point.x < x {
                    inside = !inside;
                }
            }
        }

        if !inside && distance > EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = rotate(closest_normal, trans_b.rotation);

        if distance <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![add(
                        trans_b.center,
                        rotate(closest_point, trans_b.rotation),
                    )],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Inside,
            contact: Some(Contact {
                depth: distance,
                normal,
                points: vec![trans_a.center],
            }),
        }
    }
}
