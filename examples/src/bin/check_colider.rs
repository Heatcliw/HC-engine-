//! Collision checks (no logger, no assets).
//!
//! Run:  cargo run -p hc-physics --example check_collisions
//! Exit code is 1 if at least one check fails.

use std::panic::{catch_unwind, AssertUnwindSafe};

use log::{debug, error, info};
use hc::objects::{collide, Collider, ColliderId, CollisionRelation};
use hc::math::{Transform, Vec2};
use hc::*;

fn col(name: &str, shape: Shape, x: f32, y: f32) -> Collider {
    Collider {
        id: ColliderId(0),
        name: name.into(),
        transform: Transform {
            center: Vec2 { x, y },
            rotation: 0.0,
            scale: Vec2 { x: 1.0, y: 1.0 },
        },
        geometry: shape,
        trigger: false,
    }
}

fn same(a: CollisionRelation, b: CollisionRelation) -> bool {
    std::mem::discriminant(&a) == std::mem::discriminant(&b)
}

fn rect(w: f32, h: f32) -> Shape {
    Shape::Rectangle(Rectangle { width: w, height: h })
}
fn square(s: f32) -> Shape {
    Shape::Square(Square { side: s })
}
fn circle(r: f32) -> Shape {
    Shape::Circle(Circle { radius: r })
}
fn ellipse(rx: f32, ry: f32) -> Shape {
    Shape::Ellipse(Ellipse { radius_x: rx, radius_y: ry, delta: Vec2 { x: 0.0, y: 0.0 } })
}

struct Case {
    name: &'static str,
    a: Collider,
    b: Collider,
    expected: CollisionRelation,
    depth: Option<f32>,
}

fn check_pairs() -> (u32, u32) {
    use CollisionRelation::*;
    let c = |name, a, b, expected, depth| Case { name, a, b, expected, depth };

    let cases = vec![
        c("circle-circle far",      col("a", circle(5.0), 0.0, 0.0),  col("b", circle(5.0), 20.0, 0.0), Outside,     None),
        c("circle-circle overlap",  col("a", circle(5.0), 0.0, 0.0),  col("b", circle(5.0), 8.0, 0.0),  Overlapping, Some(2.0)),
        c("circle-circle touch",    col("a", circle(5.0), 0.0, 0.0),  col("b", circle(5.0), 10.0, 0.0), Touching,    None),
        c("circle-circle inside",   col("a", circle(10.0), 0.0, 0.0), col("b", circle(2.0), 1.0, 0.0),  Inside,      None),
        c("rect-rect far",          col("a", rect(10.0, 10.0), 0.0, 0.0), col("b", rect(10.0, 10.0), 50.0, 0.0), Outside,     None),
        c("rect-rect overlap",      col("a", rect(10.0, 10.0), 0.0, 0.0), col("b", rect(10.0, 10.0), 5.0, 5.0),  Overlapping, None),
        c("square-square overlap",  col("a", square(10.0), 0.0, 0.0), col("b", square(10.0), 5.0, 5.0), Overlapping, None),
        c("rect-square overlap",    col("a", rect(10.0, 10.0), 0.0, 0.0), col("b", square(10.0), 1.0, 1.0), Overlapping, None),
        c("circle-rect overlap",    col("a", circle(5.0), 0.0, 0.0),  col("b", rect(10.0, 10.0), 8.0, 0.0),  Overlapping, None),
        c("circle-rect far",        col("a", circle(5.0), 0.0, 0.0),  col("b", rect(10.0, 10.0), 50.0, 0.0), Outside,     None),
        c("circle-ellipse far",     col("a", circle(5.0), 0.0, 0.0),  col("b", ellipse(5.0, 3.0), 100.0, 0.0), Outside,   None),
        c("circle-ellipse overlap", col("a", circle(5.0), 0.0, 0.0),  col("b", ellipse(5.0, 3.0), 6.0, 0.0),   Overlapping, None),
    ];

    let (mut pass, mut fail) = (0, 0);
    for case in &cases {
        let res = catch_unwind(AssertUnwindSafe(|| (collide(&case.a, &case.b), collide(&case.b, &case.a))));
        let (ab, ba) = match res {
            Ok(v) => v,
            Err(_) => {
                error!("[PANIC] {}", case.name);
                fail += 1;
                continue;
            }
        };

        let mut problems: Vec<String> = Vec::new();
        if !same(ab.relation, case.expected) {
            problems.push(format!("expected {:?}, got {:?}", case.expected, ab.relation));
        }
        if !same(ab.relation, ba.relation) {
            problems.push(format!("asymmetric: (a,b)={:?} (b,a)={:?}", ab.relation, ba.relation));
        }
        if let (Some(c), Some(want)) = (&ab.contact, case.depth) {
            if (c.depth - want).abs() > 1e-3 {
                problems.push(format!("depth {:.3}, expected {:.3}", c.depth, want));
            }
        }
        // NOTE: collide() orders the pair by shape type, so for different
        // shapes the normal may not flip with the argument order.
        if let (Some(c1), Some(c2)) = (&ab.contact, &ba.contact) {
            if (c1.normal.x + c2.normal.x).abs() > 1e-3 || (c1.normal.y + c2.normal.y).abs() > 1e-3 {
                problems.push(format!("normals not opposite: {:?} / {:?}", c1.normal, c2.normal));
            }
        }

        let contact = match &ab.contact {
            Some(c) => format!("depth={:.3} normal=({:.2},{:.2}) points={}", c.depth, c.normal.x, c.normal.y, c.points.len()),
            None => "no contact".to_string(),
        };

        if problems.is_empty() {
            pass += 1;
            debug!("[PASS]  {:<24} {:?} | {contact}", case.name, ab.relation);
        } else {
            fail += 1;
            debug!("[FAIL]  {:<24} {} | {contact}", case.name, problems.join("; "));
        }
    }
    (pass, fail)
}

fn check_aabb() -> (u32, u32) {
    let shapes: Vec<(&str, Shape)> = vec![
        ("circle",     circle(5.0)),
        ("rectangle",  rect(10.0, 4.0)),
        ("square",     square(10.0)),
        ("rhombus",    Shape::Rhombus(Rhombus { side: 10.0, angle: math::PI / 3.0 })),
        ("ellipse",    ellipse(5.0, 3.0)),
        ("multipoint", Shape::MultiPoint(MultiPoint { positions: vec![Vec2 { x: 0.0, y: 0.0 }, Vec2 { x: 3.0, y: 4.0 }] })),
    ];

    let (mut pass, mut fail) = (0, 0);
    for (name, shape) in shapes {
        let c = col(name, shape, 0.0, 0.0);
        match catch_unwind(AssertUnwindSafe(|| c.compute_aabb())) {
            Ok(a) => {
                let ok = [a.min.x, a.min.y, a.max.x, a.max.y].iter().all(|v| v.is_finite())
                    && a.min.x <= a.max.x
                    && a.min.y <= a.max.y;
                let text = format!("min=({},{}) max=({},{})", a.min.x, a.min.y, a.max.x, a.max.y);
                if ok {
                    pass += 1;
                    debug!("[PASS]  aabb {name:<10} {text}");
                } else {
                    fail += 1;
                    debug!("[FAIL]  aabb {name:<10} invalid: {text}");
                }
            }
            Err(_) => {
                fail += 1;
                error!("[PANIC] aabb {name}");
            }
        }
    }
    (pass, fail)
}

fn main() {
    build_logger(logs::TypeLogs::Trace);
    // silence the default panic message: panics are reported as [PANIC] lines
    std::panic::set_hook(Box::new(|_| {}));

    info!("== collision pairs ==");
    let (p1, f1) = check_pairs();
    info!("\n== aabb ==");
    let (p2, f2) = check_aabb();

    let (pass, fail) = (p1 + p2, f1 + f2);
    info!("\nresult: {pass} passed, {fail} failed");
    if fail > 0 {
        std::process::exit(1);
    }
}