use crate::{objects::{NarrowPhaseAlgorithm, CollisionInfo, CollisionRelation, Contact}, math::*};

/// Performs narrow-phase collision detection between two triangles.
///
/// The triangle vertices are reconstructed from their side lengths and
/// transformed into world coordinates. The Separating Axis Theorem is then
/// used to test the six edge normals of both triangles and determine the
/// minimum penetration depth.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct TriangleTriangle;

impl NarrowPhaseAlgorithm<Triangle, Triangle> for TriangleTriangle {
    /// Determines the collision relation between the specified triangles
    /// and calculates contact information when applicable.
    fn collide(a: (&Triangle, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {
        let (tri_a, trans_a) = (a.0, a.1);
        let (tri_b, trans_b) = (b.0, b.1);

        let scale_a = trans_a.scale.x.abs();
        let scale_b = trans_b.scale.x.abs();

        let side_a = [
            tri_a.a * scale_a,
            tri_a.b * scale_a,
            tri_a.c * scale_a,
        ];

        let side_b = [
            tri_b.a * scale_b,
            tri_b.b * scale_b,
            tri_b.c * scale_b,
        ];

        let vertex_a_a = VEC2_ZERO;

        let vertex_a_b = Vec2 {
            x: side_a[2],
            y: 0.0,
        };

        let vertex_a_c_x =
            (side_a[1] * side_a[1]
                + side_a[2] * side_a[2]
                - side_a[0] * side_a[0])
            / (2.0 * side_a[2]);

        let vertex_a_c = Vec2 {
            x: vertex_a_c_x,
            y: (side_a[1] * side_a[1]
                - vertex_a_c_x * vertex_a_c_x)
                .max(0.0)
                .sqrt(),
        };

        let vertex_b_a = VEC2_ZERO;

        let vertex_b_b = Vec2 {
            x: side_b[2],
            y: 0.0,
        };

        let vertex_b_c_x =
            (side_b[1] * side_b[1]
                + side_b[2] * side_b[2]
                - side_b[0] * side_b[0])
            / (2.0 * side_b[2]);

        let vertex_b_c = Vec2 {
            x: vertex_b_c_x,
            y: (side_b[1] * side_b[1]
                - vertex_b_c_x * vertex_b_c_x)
                .max(0.0)
                .sqrt(),
        };

        let cos_a = trans_a.rotation.cos();
        let sin_a = trans_a.rotation.sin();

        let cos_b = trans_b.rotation.cos();
        let sin_b = trans_b.rotation.sin();

        let transform_a = |point: Vec2| {
            Vec2 {
                x: trans_a.center.x
                    + point.x * cos_a
                    - point.y * sin_a,

                y: trans_a.center.y
                    + point.x * sin_a
                    + point.y * cos_a,
            }
        };

        let transform_b = |point: Vec2| {
            Vec2 {
                x: trans_b.center.x
                    + point.x * cos_b
                    - point.y * sin_b,

                y: trans_b.center.y
                    + point.x * sin_b
                    + point.y * cos_b,
            }
        };

        let vertices_a = [
            transform_a(vertex_a_a),
            transform_a(vertex_a_b),
            transform_a(vertex_a_c),
        ];

        let vertices_b = [
            transform_b(vertex_b_a),
            transform_b(vertex_b_b),
            transform_b(vertex_b_c),
        ];

        let mut axes = [VEC2_ZERO; 6];

        for i in 0..3 {
            let next = (i + 1) % 3;

            let edge_a = sub(
                vertices_a[next],
                vertices_a[i],
            );

            let edge_b = sub(
                vertices_b[next],
                vertices_b[i],
            );

            axes[i] = normalize(Vec2 {
                x: -edge_a.y,
                y: edge_a.x,
            });

            axes[i + 3] = normalize(Vec2 {
                x: -edge_b.y,
                y: edge_b.x,
            });
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

            for i in 1..3 {
                let projection_a = dot(vertices_a[i], axis);

                min_a = min_a.min(projection_a);
                max_a = max_a.max(projection_a);

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

        if min_overlap <= EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: collision_normal,
                    points: vec![
                        scale(
                            add(
                                trans_a.center,
                                trans_b.center,
                            ),
                            0.5,
                        )
                    ],
                }),
            };
        }

        let inside_triangle = |point: Vec2, vertices: [Vec2; 3]| {
            let sign_0 = gaus(
                sub(vertices[1], vertices[0]),
                sub(point, vertices[0]),
            );

            let sign_1 = gaus(
                sub(vertices[2], vertices[1]),
                sub(point, vertices[1]),
            );

            let sign_2 = gaus(
                sub(vertices[0], vertices[2]),
                sub(point, vertices[2]),
            );

            (sign_0 >= -EPSILON
                && sign_1 >= -EPSILON
                && sign_2 >= -EPSILON)
                || (sign_0 <= EPSILON
                    && sign_1 <= EPSILON
                    && sign_2 <= EPSILON)
        };

        let a_inside_b =
            inside_triangle(vertices_a[0], vertices_b)
            && inside_triangle(vertices_a[1], vertices_b)
            && inside_triangle(vertices_a[2], vertices_b);

        let b_inside_a =
            inside_triangle(vertices_b[0], vertices_a)
            && inside_triangle(vertices_b[1], vertices_a)
            && inside_triangle(vertices_b[2], vertices_a);

        let mut support_a = vertices_a[0];
        let mut support_b = vertices_b[0];

        let mut min_projection_a =
            dot(support_a, collision_normal);

        let mut max_projection_b =
            dot(support_b, collision_normal);

        for i in 1..3 {
            let projection_a =
                dot(vertices_a[i], collision_normal);

            if projection_a < min_projection_a {
                min_projection_a = projection_a;
                support_a = vertices_a[i];
            }

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