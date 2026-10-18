use crate::{objects::{NarrowPhaseAlgorithm, CollisionInfo, CollisionRelation, Contact}, Rhombus, math::*};

/// Performs narrow-phase collision detection between a rhombus and a rhombus.
///
/// The vertices of both shapes are reconstructed from their parameters
/// and transformed into world coordinates. The Separating Axis Theorem is
/// then used to test the edge normals of both shapes and determine the
/// minimum penetration depth.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct RhombusRhombus;

impl NarrowPhaseAlgorithm<Rhombus, Rhombus> for RhombusRhombus {
    /// Determines the collision relation between the specified shapes
    /// and calculates contact information when applicable.
    fn collide(a: (&Rhombus, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {
        let (rho_a, trans_a) = (a.0, a.1);
        let (rho_b, trans_b) = (b.0, b.1);

        let half_diagonal_x_a =
            rho_a.side * (rho_a.angle * 0.5).cos()
            * trans_a.scale.x.abs();

        let half_diagonal_y_a =
            rho_a.side * (rho_a.angle * 0.5).sin()
            * trans_a.scale.y.abs();

        let local_a: Vec<Vec2> = vec![
            Vec2 { x: 0.0, y: half_diagonal_y_a },
            Vec2 { x: half_diagonal_x_a, y: 0.0 },
            Vec2 { x: 0.0, y: -half_diagonal_y_a },
            Vec2 { x: -half_diagonal_x_a, y: 0.0 },
        ];

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

        let vertices_a: Vec<Vec2> = local_a
            .iter()
            .map(|vertex| add(trans_a.center, rotate(*vertex, trans_a.rotation)))
            .collect();

        let vertices_b: Vec<Vec2> = local_b
            .iter()
            .map(|vertex| add(trans_b.center, rotate(*vertex, trans_b.rotation)))
            .collect();

        let count_a = vertices_a.len();
        let count_b = vertices_b.len();

        let mut axes: Vec<Vec2> = Vec::with_capacity(count_a + count_b);

        for i in 0..count_a {
            let edge_a = sub(
                vertices_a[(i + 1) % count_a],
                vertices_a[i],
            );

            axes.push(normalize(Vec2 {
                x: -edge_a.y,
                y: edge_a.x,
            }));
        }

        for i in 0..count_b {
            let edge_b = sub(
                vertices_b[(i + 1) % count_b],
                vertices_b[i],
            );

            axes.push(normalize(Vec2 {
                x: -edge_b.y,
                y: edge_b.x,
            }));
        }

        let center_delta = sub(
            trans_a.center,
            trans_b.center,
        );

        let mut min_overlap = f32::MAX;
        let mut collision_normal = VEC2_ZERO;

        for axis in axes {
            if axis.x == 0.0 && axis.y == 0.0 {
                continue;
            }

            let mut min_a = dot(vertices_a[0], axis);
            let mut max_a = min_a;

            let mut min_b = dot(vertices_b[0], axis);
            let mut max_b = min_b;

            for i in 1..count_a {
                let projection_a = dot(vertices_a[i], axis);

                min_a = min_a.min(projection_a);
                max_a = max_a.max(projection_a);
            }

            for i in 1..count_b {
                let projection_b = dot(vertices_b[i], axis);

                min_b = min_b.min(projection_b);
                max_b = max_b.max(projection_b);
            }

            let overlap =
                max_a.min(max_b) - min_a.max(min_b);

            if overlap < -EPSILON {
                return CollisionInfo {
                    relation: CollisionRelation::Outside,
                    contact: None,
                };
            }

            if overlap < min_overlap {
                min_overlap = overlap;

                collision_normal = if dot(center_delta, axis) < 0.0 {
                    negative(axis)
                } else {
                    axis
                };
            }
        }

        let mut support_a = vertices_a[0];
        let mut support_b = vertices_b[0];

        let mut min_projection_a =
            dot(support_a, collision_normal);

        let mut max_projection_b =
            dot(support_b, collision_normal);

        for i in 1..count_a {
            let projection_a =
                dot(vertices_a[i], collision_normal);

            if projection_a < min_projection_a {
                min_projection_a = projection_a;
                support_a = vertices_a[i];
            }
        }

        for i in 1..count_b {
            let projection_b =
                dot(vertices_b[i], collision_normal);

            if projection_b > max_projection_b {
                max_projection_b = projection_b;
                support_b = vertices_b[i];
            }
        }

        let contact_point = scale(
            add(support_a, support_b),
            0.5,
        );

        if min_overlap <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: collision_normal,
                    points: vec![contact_point],
                }),
            };
        }

        let inside_shape = |point: Vec2, vertices: &Vec<Vec2>| {
            let count = vertices.len();

            let mut has_positive = false;
            let mut has_negative = false;

            for i in 0..count {
                let sign = gaus(
                    sub(vertices[(i + 1) % count], vertices[i]),
                    sub(point, vertices[i]),
                );

                if sign > EPSILON {
                    has_positive = true;
                }

                if sign < -EPSILON {
                    has_negative = true;
                }
            }

            !(has_positive && has_negative)
        };

        let a_inside_b = vertices_a
            .iter()
            .all(|vertex| inside_shape(*vertex, &vertices_b));

        let b_inside_a = vertices_b
            .iter()
            .all(|vertex| inside_shape(*vertex, &vertices_a));

        let relation = if a_inside_b || b_inside_a {
            CollisionRelation::Inside
        } else {
            CollisionRelation::Overlapping
        };

        CollisionInfo {
            relation,
            contact: Some(Contact {
                depth: min_overlap,
                normal: collision_normal,
                points: vec![contact_point],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a rhombus and a polygon.
///
/// The vertices of both shapes are reconstructed from their parameters
/// and transformed into world coordinates. The Separating Axis Theorem is
/// then used to test the edge normals of both shapes and determine the
/// minimum penetration depth.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct RhombusPolygon;

impl NarrowPhaseAlgorithm<Rhombus, Polygon> for RhombusPolygon {
    /// Determines the collision relation between the specified shapes
    /// and calculates contact information when applicable.
    fn collide(a: (&Rhombus, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {
        let (rho_a, trans_a) = (a.0, a.1);
        let (pol_b, trans_b) = (b.0, b.1);

        let half_diagonal_x_a =
            rho_a.side * (rho_a.angle * 0.5).cos()
            * trans_a.scale.x.abs();

        let half_diagonal_y_a =
            rho_a.side * (rho_a.angle * 0.5).sin()
            * trans_a.scale.y.abs();

        let local_a: Vec<Vec2> = vec![
            Vec2 { x: 0.0, y: half_diagonal_y_a },
            Vec2 { x: half_diagonal_x_a, y: 0.0 },
            Vec2 { x: 0.0, y: -half_diagonal_y_a },
            Vec2 { x: -half_diagonal_x_a, y: 0.0 },
        ];

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

        let vertices_a: Vec<Vec2> = local_a
            .iter()
            .map(|vertex| add(trans_a.center, rotate(*vertex, trans_a.rotation)))
            .collect();

        let vertices_b: Vec<Vec2> = local_b
            .iter()
            .map(|vertex| add(trans_b.center, rotate(*vertex, trans_b.rotation)))
            .collect();

        let count_a = vertices_a.len();
        let count_b = vertices_b.len();

        let mut axes: Vec<Vec2> = Vec::with_capacity(count_a + count_b);

        for i in 0..count_a {
            let edge_a = sub(
                vertices_a[(i + 1) % count_a],
                vertices_a[i],
            );

            axes.push(normalize(Vec2 {
                x: -edge_a.y,
                y: edge_a.x,
            }));
        }

        for i in 0..count_b {
            let edge_b = sub(
                vertices_b[(i + 1) % count_b],
                vertices_b[i],
            );

            axes.push(normalize(Vec2 {
                x: -edge_b.y,
                y: edge_b.x,
            }));
        }

        let center_delta = sub(
            trans_a.center,
            trans_b.center,
        );

        let mut min_overlap = f32::MAX;
        let mut collision_normal = VEC2_ZERO;

        for axis in axes {
            if axis.x == 0.0 && axis.y == 0.0 {
                continue;
            }

            let mut min_a = dot(vertices_a[0], axis);
            let mut max_a = min_a;

            let mut min_b = dot(vertices_b[0], axis);
            let mut max_b = min_b;

            for i in 1..count_a {
                let projection_a = dot(vertices_a[i], axis);

                min_a = min_a.min(projection_a);
                max_a = max_a.max(projection_a);
            }

            for i in 1..count_b {
                let projection_b = dot(vertices_b[i], axis);

                min_b = min_b.min(projection_b);
                max_b = max_b.max(projection_b);
            }

            let overlap =
                max_a.min(max_b) - min_a.max(min_b);

            if overlap < -EPSILON {
                return CollisionInfo {
                    relation: CollisionRelation::Outside,
                    contact: None,
                };
            }

            if overlap < min_overlap {
                min_overlap = overlap;

                collision_normal = if dot(center_delta, axis) < 0.0 {
                    negative(axis)
                } else {
                    axis
                };
            }
        }

        let mut support_a = vertices_a[0];
        let mut support_b = vertices_b[0];

        let mut min_projection_a =
            dot(support_a, collision_normal);

        let mut max_projection_b =
            dot(support_b, collision_normal);

        for i in 1..count_a {
            let projection_a =
                dot(vertices_a[i], collision_normal);

            if projection_a < min_projection_a {
                min_projection_a = projection_a;
                support_a = vertices_a[i];
            }
        }

        for i in 1..count_b {
            let projection_b =
                dot(vertices_b[i], collision_normal);

            if projection_b > max_projection_b {
                max_projection_b = projection_b;
                support_b = vertices_b[i];
            }
        }

        let contact_point = scale(
            add(support_a, support_b),
            0.5,
        );

        if min_overlap <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: collision_normal,
                    points: vec![contact_point],
                }),
            };
        }

        let inside_shape = |point: Vec2, vertices: &Vec<Vec2>| {
            let count = vertices.len();

            let mut has_positive = false;
            let mut has_negative = false;

            for i in 0..count {
                let sign = gaus(
                    sub(vertices[(i + 1) % count], vertices[i]),
                    sub(point, vertices[i]),
                );

                if sign > EPSILON {
                    has_positive = true;
                }

                if sign < -EPSILON {
                    has_negative = true;
                }
            }

            !(has_positive && has_negative)
        };

        let a_inside_b = vertices_a
            .iter()
            .all(|vertex| inside_shape(*vertex, &vertices_b));

        let b_inside_a = vertices_b
            .iter()
            .all(|vertex| inside_shape(*vertex, &vertices_a));

        let relation = if a_inside_b || b_inside_a {
            CollisionRelation::Inside
        } else {
            CollisionRelation::Overlapping
        };

        CollisionInfo {
            relation,
            contact: Some(Contact {
                depth: min_overlap,
                normal: collision_normal,
                points: vec![contact_point],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a rhombus and a triangle.
///
/// The vertices of both shapes are reconstructed from their parameters
/// and transformed into world coordinates. The Separating Axis Theorem is
/// then used to test the edge normals of both shapes and determine the
/// minimum penetration depth.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct RhombusTriangle;

impl NarrowPhaseAlgorithm<Rhombus, Triangle> for RhombusTriangle {
    /// Determines the collision relation between the specified shapes
    /// and calculates contact information when applicable.
    fn collide(a: (&Rhombus, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {
        let (rho_a, trans_a) = (a.0, a.1);
        let (tri_b, trans_b) = (b.0, b.1);

        let half_diagonal_x_a =
            rho_a.side * (rho_a.angle * 0.5).cos()
            * trans_a.scale.x.abs();

        let half_diagonal_y_a =
            rho_a.side * (rho_a.angle * 0.5).sin()
            * trans_a.scale.y.abs();

        let local_a: Vec<Vec2> = vec![
            Vec2 { x: 0.0, y: half_diagonal_y_a },
            Vec2 { x: half_diagonal_x_a, y: 0.0 },
            Vec2 { x: 0.0, y: -half_diagonal_y_a },
            Vec2 { x: -half_diagonal_x_a, y: 0.0 },
        ];

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

        let vertices_a: Vec<Vec2> = local_a
            .iter()
            .map(|vertex| add(trans_a.center, rotate(*vertex, trans_a.rotation)))
            .collect();

        let vertices_b: Vec<Vec2> = local_b
            .iter()
            .map(|vertex| add(trans_b.center, rotate(*vertex, trans_b.rotation)))
            .collect();

        let count_a = vertices_a.len();
        let count_b = vertices_b.len();

        let mut axes: Vec<Vec2> = Vec::with_capacity(count_a + count_b);

        for i in 0..count_a {
            let edge_a = sub(
                vertices_a[(i + 1) % count_a],
                vertices_a[i],
            );

            axes.push(normalize(Vec2 {
                x: -edge_a.y,
                y: edge_a.x,
            }));
        }

        for i in 0..count_b {
            let edge_b = sub(
                vertices_b[(i + 1) % count_b],
                vertices_b[i],
            );

            axes.push(normalize(Vec2 {
                x: -edge_b.y,
                y: edge_b.x,
            }));
        }

        let center_delta = sub(
            trans_a.center,
            trans_b.center,
        );

        let mut min_overlap = f32::MAX;
        let mut collision_normal = VEC2_ZERO;

        for axis in axes {
            if axis.x == 0.0 && axis.y == 0.0 {
                continue;
            }

            let mut min_a = dot(vertices_a[0], axis);
            let mut max_a = min_a;

            let mut min_b = dot(vertices_b[0], axis);
            let mut max_b = min_b;

            for i in 1..count_a {
                let projection_a = dot(vertices_a[i], axis);

                min_a = min_a.min(projection_a);
                max_a = max_a.max(projection_a);
            }

            for i in 1..count_b {
                let projection_b = dot(vertices_b[i], axis);

                min_b = min_b.min(projection_b);
                max_b = max_b.max(projection_b);
            }

            let overlap =
                max_a.min(max_b) - min_a.max(min_b);

            if overlap < -EPSILON {
                return CollisionInfo {
                    relation: CollisionRelation::Outside,
                    contact: None,
                };
            }

            if overlap < min_overlap {
                min_overlap = overlap;

                collision_normal = if dot(center_delta, axis) < 0.0 {
                    negative(axis)
                } else {
                    axis
                };
            }
        }

        let mut support_a = vertices_a[0];
        let mut support_b = vertices_b[0];

        let mut min_projection_a =
            dot(support_a, collision_normal);

        let mut max_projection_b =
            dot(support_b, collision_normal);

        for i in 1..count_a {
            let projection_a =
                dot(vertices_a[i], collision_normal);

            if projection_a < min_projection_a {
                min_projection_a = projection_a;
                support_a = vertices_a[i];
            }
        }

        for i in 1..count_b {
            let projection_b =
                dot(vertices_b[i], collision_normal);

            if projection_b > max_projection_b {
                max_projection_b = projection_b;
                support_b = vertices_b[i];
            }
        }

        let contact_point = scale(
            add(support_a, support_b),
            0.5,
        );

        if min_overlap <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: collision_normal,
                    points: vec![contact_point],
                }),
            };
        }

        let inside_shape = |point: Vec2, vertices: &Vec<Vec2>| {
            let count = vertices.len();

            let mut has_positive = false;
            let mut has_negative = false;

            for i in 0..count {
                let sign = gaus(
                    sub(vertices[(i + 1) % count], vertices[i]),
                    sub(point, vertices[i]),
                );

                if sign > EPSILON {
                    has_positive = true;
                }

                if sign < -EPSILON {
                    has_negative = true;
                }
            }

            !(has_positive && has_negative)
        };

        let a_inside_b = vertices_a
            .iter()
            .all(|vertex| inside_shape(*vertex, &vertices_b));

        let b_inside_a = vertices_b
            .iter()
            .all(|vertex| inside_shape(*vertex, &vertices_a));

        let relation = if a_inside_b || b_inside_a {
            CollisionRelation::Inside
        } else {
            CollisionRelation::Overlapping
        };

        CollisionInfo {
            relation,
            contact: Some(Contact {
                depth: min_overlap,
                normal: collision_normal,
                points: vec![contact_point],
            }),
        }
    }
}
