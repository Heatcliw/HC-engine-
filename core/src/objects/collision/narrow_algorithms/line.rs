use crate::{objects::{NarrowPhaseAlgorithm, CollisionInfo, CollisionRelation, Contact}, Ellipse, math::*};

pub struct LineLine;

impl NarrowPhaseAlgorithm<Line, Line> for LineLine {
    fn collide(a: (&Line, &Transform), b: (&Line, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (lin_b, trans_b) = (b.0, b.1);

        let start_a = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end_a = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let start_b = add(
            trans_b.center,
            rotate(
                Vec2 {
                    x: lin_b.start.x * trans_b.scale.x.abs(),
                    y: lin_b.start.y * trans_b.scale.y.abs(),
                },
                trans_b.rotation,
            ),
        );

        let end_b = add(
            trans_b.center,
            rotate(
                Vec2 {
                    x: lin_b.end.x * trans_b.scale.x.abs(),
                    y: lin_b.end.y * trans_b.scale.y.abs(),
                },
                trans_b.rotation,
            ),
        );

        let r = sub(end_a, start_a);
        let s = sub(end_b, start_b);

        let denominator = gaus(r, s);

        if denominator.abs() <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let qp = sub(start_b, start_a);

        let t = gaus(qp, s) / denominator;
        let u = gaus(qp, r) / denominator;

        if t < -EPSILON || t > 1.0 + EPSILON
            || u < -EPSILON || u > 1.0 + EPSILON
        {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let point = add(start_a, scale(r, t));

        let normal = if length(r) > SCALAR_ZERO {
            normalize(Vec2 {
                x: -r.y,
                y: r.x,
            })
        } else {
            VEC2_X
        };

        CollisionInfo {
            relation: CollisionRelation::Touching,
            contact: Some(Contact {
                depth: 0.0,
                normal,
                points: vec![point],
            }),
        }
    }
}

pub struct LineCircle;

impl NarrowPhaseAlgorithm<Line, Circle> for LineCircle {
    fn collide(a: (&Line, &Transform), b: (&Circle, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (cir_b, trans_b) = (b.0, b.1);

        let start = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let delta = sub(trans_b.center, start);
        let line = sub(end, start);

        let line_length_sq = dot(line, line);

        let t = if line_length_sq > SCALAR_ZERO {
            (dot(delta, line) / line_length_sq)
                .clamp(0.0, 1.0)
        } else {
            0.0
        };

        let closest_point = add(start, scale(line, t));

        let diff = sub(trans_b.center, closest_point);
        let distance = length(diff);

        let radius =
            cir_b.radius
            * trans_b.scale.x.abs();

        if distance > radius {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = if distance > SCALAR_ZERO {
            normalize(sub(closest_point, trans_b.center))
        } else {
            VEC2_X
        };

        if (distance - radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![closest_point],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![closest_point],
            }),
        }
    }
}

pub struct LineEllipse;

impl NarrowPhaseAlgorithm<Line, Ellipse> for LineEllipse {
    fn collide(a: (&Line, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (eli_b, trans_b) = (b.0, b.1);

        let start = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let delta = sub(trans_b.center, start);
        let line = sub(end, start);

        let line_length_sq = dot(line, line);

        let t = if line_length_sq > SCALAR_ZERO {
            (dot(delta, line) / line_length_sq)
                .clamp(0.0, 1.0)
        } else {
            0.0
        };

        let closest_point = add(start, scale(line, t));

        let local = rotate(
            sub(closest_point, trans_b.center),
            -trans_b.rotation,
        );

        let radius = eli_b.radius();

        let distance = length(local);

        if distance > radius {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = if distance > SCALAR_ZERO {
            normalize(sub(closest_point, trans_b.center))
        } else {
            VEC2_X
        };

        if (distance - radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![closest_point],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![closest_point],
            }),
        }
    }
}

pub struct LineSquare;

impl NarrowPhaseAlgorithm<Line, Square> for LineSquare {
    fn collide(a: (&Line, &Transform), b: (&Square, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (squ_b, trans_b) = (b.0, b.1);

        let start_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let start = rotate(
            sub(start_world, trans_b.center),
            -trans_b.rotation,
        );

        let end = rotate(
            sub(end_world, trans_b.center),
            -trans_b.rotation,
        );

        let half = squ_b.side * trans_b.scale.x.abs() * 0.5;

        let vertices = [
            Vec2 { x: -half, y: -half },
            Vec2 { x:  half, y: -half },
            Vec2 { x:  half, y:  half },
            Vec2 { x: -half, y:  half },
        ];

        let mut points = Vec::new();

        for i in 0..4 {
            let edge_start = vertices[i];
            let edge_end = vertices[(i + 1) % 4];

            let r = sub(end, start);
            let s = sub(edge_end, edge_start);

            let denominator = gaus(r, s);

            if denominator.abs() <= EPSILON {
                continue;
            }

            let qp = sub(edge_start, start);

            let t = gaus(qp, s) / denominator;
            let u = gaus(qp, r) / denominator;

            if t >= -EPSILON && t <= 1.0 + EPSILON
                && u >= -EPSILON && u <= 1.0 + EPSILON
            {
                points.push(lerp(start, end, t));
            }
        }

        if points.is_empty() {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let point = points[0];

        CollisionInfo {
            relation: CollisionRelation::Touching,
            contact: Some(Contact {
                depth: 0.0,
                normal: rotate(VEC2_X, trans_b.rotation),
                points: vec![
                    add(
                        trans_b.center,
                        rotate(point, trans_b.rotation),
                    )
                ],
            }),
        }
    }
}

pub struct LineRectangle;

impl NarrowPhaseAlgorithm<Line, Rectangle> for LineRectangle {
    fn collide(a: (&Line, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (rec_b, trans_b) = (b.0, b.1);

        let start_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let start = rotate(
            sub(start_world, trans_b.center),
            -trans_b.rotation,
        );

        let end = rotate(
            sub(end_world, trans_b.center),
            -trans_b.rotation,
        );

        let half_width =
            rec_b.width * trans_b.scale.x.abs() * 0.5;

        let half_height =
            rec_b.height * trans_b.scale.y.abs() * 0.5;

        let vertices = [
            Vec2 {
                x: -half_width,
                y: -half_height,
            },
            Vec2 {
                x: half_width,
                y: -half_height,
            },
            Vec2 {
                x: half_width,
                y: half_height,
            },
            Vec2 {
                x: -half_width,
                y: half_height,
            },
        ];

        let mut points = Vec::new();

        for i in 0..4 {
            let edge_start = vertices[i];
            let edge_end = vertices[(i + 1) % 4];

            let r = sub(end, start);
            let s = sub(edge_end, edge_start);

            let denominator = gaus(r, s);

            if denominator.abs() <= EPSILON {
                continue;
            }

            let qp = sub(edge_start, start);

            let t = gaus(qp, s) / denominator;
            let u = gaus(qp, r) / denominator;

            if t >= -EPSILON && t <= 1.0 + EPSILON
                && u >= -EPSILON && u <= 1.0 + EPSILON
            {
                points.push(lerp(start, end, t));
            }
        }

        if points.is_empty() {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let point = points[0];

        CollisionInfo {
            relation: CollisionRelation::Touching,
            contact: Some(Contact {
                depth: 0.0,
                normal: rotate(VEC2_X, trans_b.rotation),
                points: vec![
                    add(
                        trans_b.center,
                        rotate(point, trans_b.rotation),
                    )
                ],
            }),
        }
    }
}

pub struct LineRhombus;

impl NarrowPhaseAlgorithm<Line, Rhombus> for LineRhombus {
    fn collide(a: (&Line, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (rho_b, trans_b) = (b.0, b.1);

        let start_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let start = rotate(
            sub(start_world, trans_b.center),
            -trans_b.rotation,
        );

        let end = rotate(
            sub(end_world, trans_b.center),
            -trans_b.rotation,
        );

        let half_diagonal_x =
            rho_b.side * (rho_b.angle * 0.5).cos()
            * trans_b.scale.x.abs();

        let half_diagonal_y =
            rho_b.side * (rho_b.angle * 0.5).sin()
            * trans_b.scale.y.abs();

        let vertices = [
            Vec2 {
                x: 0.0,
                y: half_diagonal_y,
            },
            Vec2 {
                x: half_diagonal_x,
                y: 0.0,
            },
            Vec2 {
                x: 0.0,
                y: -half_diagonal_y,
            },
            Vec2 {
                x: -half_diagonal_x,
                y: 0.0,
            },
        ];

        for i in 0..4 {
            let edge_start = vertices[i];
            let edge_end = vertices[(i + 1) % 4];

            let r = sub(end, start);
            let s = sub(edge_end, edge_start);

            let denominator = gaus(r, s);

            if denominator.abs() <= EPSILON {
                continue;
            }

            let qp = sub(edge_start, start);

            let t = gaus(qp, s) / denominator;
            let u = gaus(qp, r) / denominator;

            if t >= -EPSILON && t <= 1.0 + EPSILON
                && u >= -EPSILON && u <= 1.0 + EPSILON
            {
                let point = lerp(start, end, t);

                return CollisionInfo {
                    relation: CollisionRelation::Touching,
                    contact: Some(Contact {
                        depth: 0.0,
                        normal: rotate(VEC2_X, trans_b.rotation),
                        points: vec![
                            add(
                                trans_b.center,
                                rotate(point, trans_b.rotation),
                            )
                        ],
                    }),
                };
            }
        }

        CollisionInfo {
            relation: CollisionRelation::Outside,
            contact: None,
        }
    }
}

pub struct LinePolygon;

impl NarrowPhaseAlgorithm<Line, Polygon> for LinePolygon {
    fn collide(a: (&Line, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (pol_b, trans_b) = (b.0, b.1);

        let start_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let start = rotate(
            sub(start_world, trans_b.center),
            -trans_b.rotation,
        );

        let end = rotate(
            sub(end_world, trans_b.center),
            -trans_b.rotation,
        );

        let count = pol_b.local_vectors.len();

        if count < 2 {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        for i in 0..count {
            let edge_start = pol_b.local_vectors[i];
            let edge_end = pol_b.local_vectors[(i + 1) % count];

            let r = sub(end, start);
            let s = sub(edge_end, edge_start);

            let denominator = gaus(r, s);

            if denominator.abs() <= EPSILON {
                continue;
            }

            let qp = sub(edge_start, start);

            let t = gaus(qp, s) / denominator;
            let u = gaus(qp, r) / denominator;

            if t >= -EPSILON && t <= 1.0 + EPSILON
                && u >= -EPSILON && u <= 1.0 + EPSILON
            {
                let point = lerp(start, end, t);

                return CollisionInfo {
                    relation: CollisionRelation::Touching,
                    contact: Some(Contact {
                        depth: 0.0,
                        normal: rotate(VEC2_X, trans_b.rotation),
                        points: vec![
                            add(
                                trans_b.center,
                                rotate(point, trans_b.rotation),
                            )
                        ],
                    }),
                };
            }
        }

        CollisionInfo {
            relation: CollisionRelation::Outside,
            contact: None,
        }
    }
}

pub struct LineTriangle;

impl NarrowPhaseAlgorithm<Line, Triangle> for LineTriangle {
    fn collide(a: (&Line, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {
        let (lin_a, trans_a) = (a.0, a.1);
        let (tri_b, trans_b) = (b.0, b.1);

        let side_a = tri_b.a * trans_b.scale.x.abs();
        let side_b = tri_b.b * trans_b.scale.x.abs();
        let side_c = tri_b.c * trans_b.scale.x.abs();

        let start_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.start.x * trans_a.scale.x.abs(),
                    y: lin_a.start.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let end_world = add(
            trans_a.center,
            rotate(
                Vec2 {
                    x: lin_a.end.x * trans_a.scale.x.abs(),
                    y: lin_a.end.y * trans_a.scale.y.abs(),
                },
                trans_a.rotation,
            ),
        );

        let start = rotate(
            sub(start_world, trans_b.center),
            -trans_b.rotation,
        );

        let end = rotate(
            sub(end_world, trans_b.center),
            -trans_b.rotation,
        );

        let vertex_a = VEC2_ZERO;

        let vertex_b = Vec2 {
            x: side_c,
            y: 0.0,
        };

        let vertex_c_x =
            (side_b.powi(2) + side_c.powi(2) - side_a.powi(2))
            / (2.0 * side_c);

        let vertex_c_y =
            (side_b.powi(2) - vertex_c_x.powi(2))
            .max(0.0)
            .sqrt();

        let vertex_c = Vec2 {
            x: vertex_c_x,
            y: vertex_c_y,
        };

        let vertices = [
            vertex_a,
            vertex_b,
            vertex_c,
        ];

        for i in 0..3 {
            let edge_start = vertices[i];
            let edge_end = vertices[(i + 1) % 3];

            let r = sub(end, start);
            let s = sub(edge_end, edge_start);

            let denominator = gaus(r, s);

            if denominator.abs() <= EPSILON {
                continue;
            }

            let qp = sub(edge_start, start);

            let t = gaus(qp, s) / denominator;
            let u = gaus(qp, r) / denominator;

            if t >= -EPSILON && t <= 1.0 + EPSILON
                && u >= -EPSILON && u <= 1.0 + EPSILON
            {
                let point = lerp(start, end, t);

                return CollisionInfo {
                    relation: CollisionRelation::Touching,
                    contact: Some(Contact {
                        depth: 0.0,
                        normal: rotate(VEC2_X, trans_b.rotation),
                        points: vec![
                            add(
                                trans_b.center,
                                rotate(point, trans_b.rotation),
                            )
                        ],
                    }),
                };
            }
        }

        CollisionInfo {
            relation: CollisionRelation::Outside,
            contact: None,
        }
    }
}