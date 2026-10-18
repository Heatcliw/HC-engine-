use crate::{objects::{NarrowPhaseAlgorithm, CollisionInfo, CollisionRelation, Contact}, Circle, math::*};

/// Performs narrow-phase collision detection between two circles.
///
/// The distance between the circle centers is compared with the sum of
/// their radii to determine whether the circles are outside, touching,
/// overlapping, or one circle is completely inside the other.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct CircleCircle;

impl NarrowPhaseAlgorithm<Circle, Circle> for CircleCircle {
    /// Determines the collision relation between the specified circles
    /// and calculates contact information when applicable.
    fn collide(a: (&Circle, &Transform), b: (&Circle, &Transform)) -> CollisionInfo {
        let (cir_a, trans_a) = (a.0, a.1);
        let (eli_b, trans_b) = (b.0, b.1);

        let delta = sub(trans_a.center, trans_b.center);
        let distance = length(delta);
        let sum_rad = cir_a.radius + eli_b.radius;

        if distance > sum_rad {
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

        let point_a = add(trans_a.center, scale(normal, cir_a.radius));

        let point_b = sub(trans_b.center, scale(normal, eli_b.radius));

        let point = scale(add(point_a, point_b), 0.5);

        let contact = Contact {
            depth: sum_rad - distance,
            normal,
            points: vec![point],
        };

        if distance + eli_b.radius <= cir_a.radius - EPSILON
            || distance + cir_a.radius <= eli_b.radius - EPSILON
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

/// Performs narrow-phase collision detection between a circle and an ellipse.
///
/// The distance between the circle and ellipse centers is used to determine
/// whether the shapes are outside, touching, or overlapping each other.
/// The ellipse radius in the direction of the circle is used to calculate
/// the combined collision distance and contact information.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct CircleEllipse;

impl NarrowPhaseAlgorithm<Circle, Ellipse> for CircleEllipse {
    /// Determines the collision relation between the specified circle
    /// and ellipse and calculates contact information when applicable.
    fn collide(a: (&Circle, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {
        let (cir_a, trans_a) = (a.0, a.1);
        let (eli_b, trans_b) = (b.0, b.1);
        
        let delta = sub(trans_b.center, trans_a.center);

        let distance = length(eli_b.delta);
        let theta = eli_b.delta.y.atan2(eli_b.delta.x);

        let cos_t = theta.cos();
        let sin_t = theta.sin();
        
        let radius_elipse = eli_b.radius();

        let sum_rad = cir_a.radius + radius_elipse;

        if distance > sum_rad {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let normal = if distance > SCALAR_ZERO {
            div(eli_b.delta, distance)
        } else {
            VEC2_X
        };

        let point_a = add(trans_a.center, scale(normal, cir_a.radius));

        let point_b = sub(trans_b.center, scale(normal, radius_elipse));

        let point = scale(add(point_a, point_b), 0.5);

        let contact = Contact {
            depth: sum_rad - distance,
            normal,
            points: vec![point],
        };

        if distance + radius_elipse <= cir_a.radius - EPSILON
            || distance + cir_a.radius <= radius_elipse - EPSILON
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

/// Performs narrow-phase collision detection between a circle and a square.
///
/// The circle center is transformed into the square's local coordinate
/// system. The closest point on the square is then determined by clamping
/// the circle center to the square boundaries.
///
/// The distance between the circle center and the closest point is used to
/// determine whether the circle is outside, touching, or overlapping the
/// square. The distances to the square sides are additionally used to
/// determine whether the circle center is located inside the square.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct CircleSquare;

impl NarrowPhaseAlgorithm<Circle, Square> for CircleSquare {
    /// Determines the collision relation between the specified circle
    /// and square and calculates contact information when applicable.
    fn collide(a: (&Circle, &Transform), b: (&Square, &Transform)) -> CollisionInfo {
        let (cir_a, trans_a) = (a.0, a.1);
        let (squ_b, trans_b) = (b.0, b.1);
        
        let radius = cir_a.radius * trans_a.scale.x.abs();

        let half = squ_b.side * trans_b.scale.x.abs() * 0.5;
    
        let delta = sub(trans_a.center, trans_b.center);

        let cos = trans_b.rotation.cos();
        let sin = trans_b.rotation.sin();

        let local_point = Vec2 {
            x: (delta.x * cos + delta.y * sin), 
            y: (-delta.x * sin + delta.y * cos)
        };

        let clossest_point = Vec2 {
            x: local_point.x.clamp(-half, half), 
            y: local_point.y.clamp(-half, half)
        };

        let diff_point = Vec2 {
            x: local_point.x - clossest_point.x ,
            y: local_point.y - clossest_point.y
        };

        let distance_sq = dot(diff_point, diff_point);

        if distance_sq > radius.powi(2) {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let distance = distance_sq.sqrt();

        if (distance - radius).abs() < f32::EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: VEC2_ZERO,
                    points: vec![trans_a.center],
                }),
            };
        };

        let sides: [f32; 4] = [
            local_point.x + half,
            half - local_point.x,
            local_point.y + half,
            half - local_point.y,
        ];

        let min_distance = sides[0]
            .min(sides[1])
            .min(sides[2])
            .min(sides[3]);

        if min_distance >= radius {
            let mut normal_local= VEC2_ZERO;

            if sides[0] == min_distance {
                normal_local = negative_absciss(VEC2_X);
            } else if sides[1] == min_distance {
                normal_local = VEC2_X;
            } else if sides[2] == min_distance {
                normal_local = negative_ordinate(VEC2_Y);
            } else {
                normal_local = VEC2_Y;
            }

            let normal = Vec2 {
                x: normal_local.x * cos - normal_local.y * sin,
                y: normal_local.x * sin + normal_local.y * cos,
        };

            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: radius - min_distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        let normal = if distance > 0.0 {
            let mut normal_local = Vec2 {
                x: diff_point.x / distance, 
                y: diff_point.y / distance
            };

            Vec2 {
                x: normal_local.x * cos - normal_local.y * sin,
                y: normal_local.x * sin + normal_local.y * cos,
            }
        } else {
            VEC2_ZERO
        };

        let contact_world = Vec2 {
           x: trans_b.center.x + local_point.x * cos - local_point.y * sin,
           y: trans_b.center.y + local_point.x * sin + local_point.y * cos,
        };

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a circle and a rectangle.
///
/// The circle center is transformed into the rectangle's local coordinate
/// system. The closest point on the rectangle is then determined by clamping
/// the circle center to the rectangle boundaries.
///
/// The distance between the circle center and the closest point is used to
/// determine whether the circle is outside, touching, or overlapping the
/// rectangle. The distances to the rectangle sides are additionally used
/// to determine whether the circle center is located inside the rectangle.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct CircleRectangle;

impl NarrowPhaseAlgorithm<Circle, Rectangle> for CircleRectangle {
    /// Determines the collision relation between the specified circle
    /// and rectangle and calculates contact information when applicable.
    fn collide(a: (&Circle, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {
        let (cir_a, trans_a) = (a.0, a.1);
        let (rec_b, trans_b) = (b.0, b.1);
        
        let radius = cir_a.radius * trans_a.scale.x.abs();

        let half_width = rec_b.width * trans_b.scale.x.abs() * 0.5;
        let half_height = rec_b.height * trans_b.scale.y.abs() * 0.5;
    
        let delta = sub(trans_a.center, trans_b.center);

        let cos = trans_b.rotation.cos();
        let sin = trans_b.rotation.sin();

        let local_point = Vec2 {
            x: (delta.x * cos + delta.y * sin), 
            y: (-delta.x * sin + delta.y * cos)
        };

        let clossest_point = Vec2 {
            x: local_point.x.clamp(-half_width, half_width),
            y: local_point.y.clamp(-half_height, half_height),
        };

        let diff_point = Vec2 {
            x: local_point.x - clossest_point.x ,
            y: local_point.y - clossest_point.y
        };

        let distance_sq = dot(diff_point, diff_point);

        if distance_sq > radius.powi(2) {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let distance = distance_sq.sqrt();

        if (distance - radius).abs() < f32::EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: VEC2_ZERO,
                    points: vec![trans_a.center],
                }),
            };
        };

        let sides: [f32; 4] = [
            local_point.x + half_width,
            half_width - local_point.x,
            local_point.y + half_height,
            half_height - local_point.y,
        ];

        let min_distance = sides[0]
            .min(sides[1])
            .min(sides[2])
            .min(sides[3]);

        if min_distance >= radius {
            let mut normal_local= VEC2_ZERO;

            if sides[0] == min_distance {
                normal_local = negative_absciss(VEC2_X);
            } else if sides[1] == min_distance {
                normal_local = VEC2_X;
            } else if sides[2] == min_distance {
                normal_local = negative_ordinate(VEC2_Y);
            } else {
                normal_local = VEC2_Y;
            }

            let normal = Vec2 {
                x: normal_local.x * cos - normal_local.y * sin,
                y: normal_local.x * sin + normal_local.y * cos,
        };

            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: radius - min_distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        let normal = if distance > 0.0 {
            let mut normal_local = Vec2 {
                x: diff_point.x / distance, 
                y: diff_point.y / distance
            };

            Vec2 {
                x: normal_local.x * cos - normal_local.y * sin,
                y: normal_local.x * sin + normal_local.y * cos,
            }
        } else {
            VEC2_ZERO
        };

        let contact_world = Vec2 {
           x: trans_b.center.x + local_point.x * cos - local_point.y * sin,
           y: trans_b.center.y + local_point.x * sin + local_point.y * cos,
        };

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a circle and a rhombus.
///
/// The rhombus diagonals are calculated from its side length and angle.
/// The circle center is transformed into the rhombus's local coordinate
/// system, after which the closest point on each rhombus edge is determined.
///
/// The minimum distance to the rhombus edges is used to determine whether
/// the circle is outside, touching, or overlapping the rhombus. The side
/// distances are additionally used to determine whether the circle center
/// is located inside the rhombus.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct CircleRhombus;

impl NarrowPhaseAlgorithm<Circle, Rhombus> for CircleRhombus {
    /// Determines the collision relation between the specified circle
    /// and rhombus and calculates contact information when applicable.
    fn collide(a: (&Circle, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {
        let (cir_a, trans_a) = (a.0, a.1);
        let (rho_b, trans_b) = (b.0, b.1);

        let radius = cir_a.radius * trans_a.scale.x.abs();

        let half_diagonal_x =
            rho_b.side * (rho_b.angle * 0.5).cos()
            * trans_b.scale.x.abs();

        let half_diagonal_y =
            rho_b.side * (rho_b.angle * 0.5).sin()
            * trans_b.scale.y.abs();

        let delta = sub(trans_a.center, trans_b.center);

        let cos = trans_b.rotation.cos();
        let sin = trans_b.rotation.sin();

        let local_point = Vec2 {
            x: delta.x * cos + delta.y * sin,
            y: -delta.x * sin + delta.y * cos,
        };

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
            let point_to_start = sub(local_point, start);

            let edge_length_sq = dot(edge, edge);

            let t = if edge_length_sq > SCALAR_ZERO {
                (dot(point_to_start, edge) / edge_length_sq)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };

            let point = add(start, scale(edge, t));

            let diff = sub(local_point, point);
            let distance_sq = dot(diff, diff);

            if distance_sq < min_distance_sq {
                min_distance_sq = distance_sq;
                closest_point = point;
            }
        }

        if min_distance_sq > radius.powi(2) {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let distance = min_distance_sq.sqrt();

        if (distance - radius).abs() < f32::EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: VEC2_ZERO,
                    points: vec![trans_a.center],
                }),
            };
        }

        let side_0 = local_point.x + local_point.y - half_diagonal_y;
        let side_1 = local_point.x - local_point.y - half_diagonal_x;
        let side_2 = -local_point.x - local_point.y - half_diagonal_y;
        let side_3 = -local_point.x + local_point.y - half_diagonal_x;

        let inside =
            side_0 <= 0.0 &&
            side_1 <= 0.0 &&
            side_2 <= 0.0 &&
            side_3 <= 0.0;

        if inside {
            let mut normal_local = VEC2_ZERO;

            let distance_top =
                half_diagonal_y - local_point.y;

            let distance_right =
                half_diagonal_x - local_point.x;

            let distance_bottom =
                half_diagonal_y + local_point.y;

            let distance_left =
                half_diagonal_x + local_point.x;

            let min_distance =
                distance_top
                    .min(distance_right)
                    .min(distance_bottom)
                    .min(distance_left);

            if min_distance == distance_top {
                normal_local = negative_ordinate(VEC2_Y);
            } else if min_distance == distance_right {
                normal_local = VEC2_X;
            } else if min_distance == distance_bottom {
                normal_local = VEC2_Y;
            } else {
                normal_local = negative_absciss(VEC2_X);
            }

            let normal = Vec2 {
                x: normal_local.x * cos - normal_local.y * sin,
                y: normal_local.x * sin + normal_local.y * cos,
            };

            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: radius - min_distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        let normal = if distance > SCALAR_ZERO {
            let normal_local = Vec2 {
                x: (local_point.x - closest_point.x) / distance,
                y: (local_point.y - closest_point.y) / distance,
            };

            Vec2 {
                x: normal_local.x * cos - normal_local.y * sin,
                y: normal_local.x * sin + normal_local.y * cos,
            }
        } else {
            VEC2_ZERO
        };

        let contact_world = Vec2 {
            x: trans_b.center.x
                + closest_point.x * cos
                - closest_point.y * sin,

            y: trans_b.center.y
                + closest_point.x * sin
                + closest_point.y * cos,
        };

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a circle and a polygon.
///
/// The circle center is transformed into the polygon's local coordinate
/// system. The closest point on each polygon edge is then determined and
/// the minimum distance to the polygon is used to detect a collision.
///
/// The polygon containment test determines whether the circle center is
/// located inside the polygon.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct CirclePolygon;

impl NarrowPhaseAlgorithm<Circle, Polygon> for CirclePolygon {
    /// Determines the collision relation between the specified circle
    /// and polygon and calculates contact information when applicable.
    fn collide(a: (&Circle, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {
        let (cir_a, trans_a) = (a.0, a.1);
        let (pol_b, trans_b) = (b.0, b.1);

        let radius = cir_a.radius * trans_a.scale.x.abs();

        let delta = sub(trans_a.center, trans_b.center);

        let cos = trans_b.rotation.cos();
        let sin = trans_b.rotation.sin();

        let local_point = Vec2 {
            x: delta.x * cos + delta.y * sin,
            y: -delta.x * sin + delta.y * cos,
        };

        let mut closest_point = VEC2_ZERO;
        let mut min_distance_sq = f32::MAX;

        let count = pol_b.local_vectors.len();

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

        if min_distance_sq > radius.powi(2) {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let distance = min_distance_sq.sqrt();

        if (distance - radius).abs() < f32::EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: VEC2_ZERO,
                    points: vec![trans_a.center],
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
            let normal = if distance > SCALAR_ZERO {
                let normal_local = Vec2 {
                    x: (closest_point.x - local_point.x) / distance,
                    y: (closest_point.y - local_point.y) / distance,
                };

                Vec2 {
                    x: -normal_local.x * cos + normal_local.y * sin,
                    y: -normal_local.x * sin - normal_local.y * cos,
                }
            } else {
                VEC2_ZERO
            };

            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: radius - distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        let normal = if distance > SCALAR_ZERO {
            let normal_local = Vec2 {
                x: (local_point.x - closest_point.x) / distance,
                y: (local_point.y - closest_point.y) / distance,
            };

            Vec2 {
                x: normal_local.x * cos - normal_local.y * sin,
                y: normal_local.x * sin + normal_local.y * cos,
            }
        } else {
            VEC2_ZERO
        };

        let contact_world = Vec2 {
            x: trans_b.center.x
                + closest_point.x * cos
                - closest_point.y * sin,

            y: trans_b.center.y
                + closest_point.x * sin
                + closest_point.y * cos,
        };

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}

/// Performs narrow-phase collision detection between a circle and a triangle.
///
/// The triangle is reconstructed from its three side lengths using the
/// law of cosines. The circle center is transformed into the triangle's
/// local coordinate system, after which the closest point on each triangle
/// edge is determined.
///
/// The minimum distance between the circle center and the triangle is used
/// to determine whether the circle is outside, touching, overlapping,
/// or completely inside the triangle.
///
/// The returned contact contains the penetration depth, collision normal,
/// and contact point when a collision is detected.
pub struct CircleTriangle;

impl NarrowPhaseAlgorithm<Circle, Triangle> for CircleTriangle {
    /// Determines the collision relation between the specified circle
    /// and triangle and calculates contact information when applicable.
    fn collide(a: (&Circle, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {
        let (cir_a, trans_a) = (a.0, a.1);
        let (tri_b, trans_b) = (b.0, b.1);

        let radius = cir_a.radius * trans_a.scale.x.abs();

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
                (dot(point, edge) / edge_length_sq).clamp(0.0, 1.0)
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

        if min_distance_sq > radius.powi(2) {
            return CollisionInfo {
                relation: CollisionRelation::Outside,
                contact: None,
            };
        }

        let distance = min_distance_sq.sqrt();

        if (distance - radius).abs() < EPSILON {
            return CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(Contact {
                    depth: 0.0,
                    normal: VEC2_ZERO,
                    points: vec![trans_a.center],
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
            let normal = if distance > SCALAR_ZERO {
                normalize(sub(closest_point, local_point))
            } else {
                VEC2_ZERO
            };

            let normal = rotate(normal, trans_b.rotation);

            return CollisionInfo {
                relation: CollisionRelation::Inside,
                contact: Some(Contact {
                    depth: radius + distance,
                    normal,
                    points: vec![trans_a.center],
                }),
            };
        }

        let normal = if distance > SCALAR_ZERO {
            let normal_local = normalize(sub(local_point, closest_point));

            rotate(normal_local, trans_b.rotation)
        } else {
            VEC2_ZERO
        };

        let contact_world = add(
            trans_b.center,
            rotate(closest_point, trans_b.rotation),
        );

        CollisionInfo {
            relation: CollisionRelation::Overlapping,
            contact: Some(Contact {
                depth: radius - distance,
                normal,
                points: vec![contact_world],
            }),
        }
    }
}