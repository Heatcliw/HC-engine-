use crate::{body::{NarrowPhaseAlgorithm, CollisionInfo, CollisionRelation, Contact}, Ellipse, math::*};

pub struct EllipseEllipse;

impl NarrowPhaseAlgorithm<Ellipse, Ellipse> for EllipseEllipse {
    fn collide(a: (&Ellipse, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {
        let (eli_a, trans_a) = (a.0, a.1);
        let (eli_b, trans_b) = (b.0, b.1);

        let delta = sub(trans_a.center, trans_b.center);
        let distance = length(delta);

        let normal = if distance > SCALAR_ZERO {
            div(delta, distance)
        } else {
            VEC2_X
        };

        let radius_a = {
            let angle = normal.y.atan2(normal.x);
            let cos = angle.cos();
            let sin = angle.sin();

            let local = Vec2 {
                x: cos,
                y: sin,
            };

            let radius = eli_a.radius();
            radius * length(local)
        };

        let radius_b = {
            let angle = (-normal.y).atan2(-normal.x);
            let cos = angle.cos();
            let sin = angle.sin();

            let local = Vec2 {
                x: cos,
                y: sin,
            };

            let radius = eli_b.radius();
            radius * length(local)
        };

        let sum_rad = radius_a + radius_b;

        if distance > sum_rad {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let point_a = add(
            trans_a.center,
            scale(normal, radius_a),
        );

        let point_b = sub(
            trans_b.center,
            scale(normal, radius_b),
        );

        let point = scale(add(point_a, point_b), 0.5);

        let contact = Contact {
            depth: sum_rad - distance,
            normal,
            points: vec![point],
        };

        if distance + radius_b <= radius_a - EPSILON
            || distance + radius_a <= radius_b - EPSILON
        {
            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(contact),
            };
        }

        if (distance - sum_rad).abs() < EPSILON {
            CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(contact),
            }
        } else {
            CollisionInfo {
                relation: CollisionRelation::Overlapping,
                contact: Some(contact),
            }
        }
    }
}

pub struct EllipseSquare;

impl NarrowPhaseAlgorithm<Ellipse, Square> for EllipseSquare {
    fn collide(a: (&Ellipse, &Transform), b: (&Square, &Transform)) -> CollisionInfo {
        let (eli_a, trans_a) = (a.0, a.1);
        let (squ_b, trans_b) = (b.0, b.1);

        let delta = sub(trans_a.center, trans_b.center);

        let cos = trans_b.rotation.cos();
        let sin = trans_b.rotation.sin();

        let local_point = Vec2 {
            x: delta.x * cos + delta.y * sin,
            y: -delta.x * sin + delta.y * cos,
        };

        let half = squ_b.side * trans_b.scale.x.abs() * 0.5;

        let closest_point = Vec2 {
            x: local_point.x.clamp(-half, half),
            y: local_point.y.clamp(-half, half),
        };

        let diff = sub(local_point, closest_point);
        let distance_sq = dot(diff, diff);

        let distance = distance_sq.sqrt();

        let direction = if distance > SCALAR_ZERO {
            normalize(diff)
        } else {
            VEC2_X
        };

        let world_direction = rotate(direction, trans_b.rotation);

        let ellipse_radius = eli_a.radius();

        if distance > ellipse_radius {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        if (distance - ellipse_radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: world_direction,
                    points: vec![
                        add(
                            trans_b.center,
                            rotate(closest_point, trans_b.rotation),
                        )
                    ],
                }),
            };
        }

        let sides = [
            local_point.x + half,
            half - local_point.x,
            local_point.y + half,
            half - local_point.y,
        ];

        let min_distance = sides[0]
            .min(sides[1])
            .min(sides[2])
            .min(sides[3]);

        if min_distance >= ellipse_radius {
            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: ellipse_radius - min_distance,
                    normal: world_direction,
                    points: vec![trans_a.center],
                }),
            };
        }

        let contact_world = add(
            trans_b.center,
            rotate(closest_point, trans_b.rotation),
        );

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: ellipse_radius - distance,
                normal: world_direction,
                points: vec![contact_world],
            }),
        }
    }
}

pub struct EllipseRectangle;

impl NarrowPhaseAlgorithm<Ellipse, Rectangle> for EllipseRectangle {
    fn collide(a: (&Ellipse, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {
        let (eli_a, trans_a) = (a.0, a.1);
        let (rec_b, trans_b) = (b.0, b.1);

        let delta = sub(trans_a.center, trans_b.center);

        let cos = trans_b.rotation.cos();
        let sin = trans_b.rotation.sin();

        let local_point = Vec2 {
            x: delta.x * cos + delta.y * sin,
            y: -delta.x * sin + delta.y * cos,
        };

        let half_width = rec_b.width * trans_b.scale.x.abs() * 0.5;
        let half_height = rec_b.height * trans_b.scale.y.abs() * 0.5;

        let closest_point = Vec2 {
            x: local_point.x.clamp(-half_width, half_width),
            y: local_point.y.clamp(-half_height, half_height),
        };

        let diff = sub(local_point, closest_point);
        let distance_sq = dot(diff, diff);
        let distance = distance_sq.sqrt();

        let direction = if distance > SCALAR_ZERO {
            normalize(diff)
        } else {
            VEC2_X
        };

        let normal = rotate(direction, trans_b.rotation);
        let ellipse_radius = eli_a.radius();

        if distance > ellipse_radius {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let contact_world = add(
            trans_b.center,
            rotate(closest_point, trans_b.rotation),
        );

        if (distance - ellipse_radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![contact_world],
                }),
            };
        }

        let sides = [
            local_point.x + half_width,
            half_width - local_point.x,
            local_point.y + half_height,
            half_height - local_point.y,
        ];

        let min_distance = sides[0]
            .min(sides[1])
            .min(sides[2])
            .min(sides[3]);

        if min_distance >= ellipse_radius {
            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: ellipse_radius - min_distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: ellipse_radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}

pub struct EllipseRhombus;

impl NarrowPhaseAlgorithm<Ellipse, Rhombus> for EllipseRhombus {
    fn collide(a: (&Ellipse, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {
        let (eli_a, trans_a) = (a.0, a.1);
        let (rho_b, trans_b) = (b.0, b.1);

        let delta = sub(trans_a.center, trans_b.center);

        let local_point = rotate(delta, -trans_b.rotation);

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

        let mut closest_point = vertices[0];
        let mut min_distance_sq = f32::MAX;

        for i in 0..4 {
            let start = vertices[i];
            let end = vertices[(i + 1) % 4];

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
            }
        }

        let distance = min_distance_sq.sqrt();

        let direction = if distance > SCALAR_ZERO {
            normalize(sub(local_point, closest_point))
        } else {
            VEC2_X
        };

        let normal = rotate(direction, trans_b.rotation);

        let ellipse_radius = eli_a.radius();

        if distance > ellipse_radius {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let contact_world = add(
            trans_b.center,
            rotate(closest_point, trans_b.rotation),
        );

        if (distance - ellipse_radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![contact_world],
                }),
            };
        }

        let inside =
            local_point.x + local_point.y <= half_diagonal_y &&
            local_point.x - local_point.y <= half_diagonal_x &&
            -local_point.x - local_point.y <= half_diagonal_y &&
            -local_point.x + local_point.y <= half_diagonal_x;

        if inside {
            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: ellipse_radius - distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: ellipse_radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}

pub struct EllipsePolygon;

impl NarrowPhaseAlgorithm<Ellipse, Polygon> for EllipsePolygon {
    fn collide(a: (&Ellipse, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {
        let (eli_a, trans_a) = (a.0, a.1);
        let (pol_b, trans_b) = (b.0, b.1);

        let delta = sub(trans_a.center, trans_b.center);

        let local_point = rotate(delta, -trans_b.rotation);

        let count = pol_b.local_vectors.len();

        if count < 2 {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let mut closest_point = VEC2_ZERO;
        let mut min_distance_sq = f32::MAX;

        for i in 0..count {
            let start = pol_b.local_vectors[i];
            let end = pol_b.local_vectors[(i + 1) % count];

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
            }
        }

        let distance = min_distance_sq.sqrt();

        let direction = if distance > SCALAR_ZERO {
            normalize(sub(local_point, closest_point))
        } else {
            VEC2_X
        };

        let normal = rotate(direction, trans_b.rotation);
        let ellipse_radius = eli_a.radius();

        if distance > ellipse_radius {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let contact_world = add(
            trans_b.center,
            rotate(closest_point, trans_b.rotation),
        );

        if (distance - ellipse_radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![contact_world],
                }),
            };
        }

        let mut inside = false;

        for i in 0..count {
            let a = pol_b.local_vectors[i];
            let b = pol_b.local_vectors[(i + 1) % count];

            if (a.y > local_point.y) != (b.y > local_point.y) {
                let x = (b.x - a.x)
                    * (local_point.y - a.y)
                    / (b.y - a.y)
                    + a.x;

                if local_point.x < x {
                    inside = !inside;
                }
            }
        }

        if inside {
            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: ellipse_radius - distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: ellipse_radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}

pub struct EllipseTriangle;

impl NarrowPhaseAlgorithm<Ellipse, Triangle> for EllipseTriangle {
    fn collide(a: (&Ellipse, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {
        let (eli_a, trans_a) = (a.0, a.1);
        let (tri_b, trans_b) = (b.0, b.1);

        let ellipse_radius = eli_a.radius();

        let side_a = tri_b.a * trans_b.scale.x.abs();
        let side_b = tri_b.b * trans_b.scale.x.abs();
        let side_c = tri_b.c * trans_b.scale.x.abs();

        let delta = sub(trans_a.center, trans_b.center);

        let local_point = rotate(delta, -trans_b.rotation);

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

        let mut closest_point = VEC2_ZERO;
        let mut min_distance_sq = f32::MAX;

        for i in 0..3 {
            let start = vertices[i];
            let end = vertices[(i + 1) % 3];

            let edge = sub(end, start);
            let point = sub(local_point, start);

            let edge_length_sq = dot(edge, edge);

            let t = if edge_length_sq > SCALAR_ZERO {
                (dot(point, edge) / edge_length_sq)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };

            let point_on_edge = lerp(start, end, t);

            let diff = sub(local_point, point_on_edge);
            let distance_sq = dot(diff, diff);

            if distance_sq < min_distance_sq {
                min_distance_sq = distance_sq;
                closest_point = point_on_edge;
            }
        }

        let distance = min_distance_sq.sqrt();

        let direction = if distance > SCALAR_ZERO {
            normalize(sub(local_point, closest_point))
        } else {
            VEC2_X
        };

        let normal = rotate(direction, trans_b.rotation);

        if distance > ellipse_radius {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let contact_world = add(
            trans_b.center,
            rotate(closest_point, trans_b.rotation),
        );

        if (distance - ellipse_radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal,
                    points: vec![contact_world],
                }),
            };
        }

        let sign_0 = gaus(
            sub(vertices[1], vertices[0]),
            sub(local_point, vertices[0]),
        );

        let sign_1 = gaus(
            sub(vertices[2], vertices[1]),
            sub(local_point, vertices[1]),
        );

        let sign_2 = gaus(
            sub(vertices[0], vertices[2]),
            sub(local_point, vertices[2]),
        );

        let inside =
            (sign_0 >= 0.0 && sign_1 >= 0.0 && sign_2 >= 0.0)
            || (sign_0 <= 0.0 && sign_1 <= 0.0 && sign_2 <= 0.0);

        if inside {
            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: ellipse_radius - distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: ellipse_radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}