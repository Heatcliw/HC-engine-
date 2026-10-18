//! SPH fluid checks (no logger, no assets).
//!
//! Run:  cargo run -p hc-physics --example check_fluid
//! Exit code is 1 if at least one check fails.
use log::{info, debug};
use hc::math::{self, Vec2};
use hc::*;
use hc::logs::{build_logger, TypeLogs};
const DT: f32 = 1.0 / 60.0;

struct Report {
    pass: u32,
    fail: u32,
}

impl Report {
    fn check(&mut self, name: &str, ok: bool, details: String) {
        if ok {
            self.pass += 1;
            debug!("[PASS]  {:<34} {}", name, details);
        } else {
            self.fail += 1;
            debug!("[FAIL] {:<34} {}", name, details);
        }
    }
}

fn zero_gravity() -> SphConfig {
    SphConfig { gravity: Vec2 { x: 0.0, y: 0.0 }, ..SphConfig::builtin() }
}

/// n x n square lattice of particles with the given spacing.
fn lattice(n: usize, spacing: f32, mass: f32) -> ParticleStorage {
    let mut st = ParticleStorage::default();
    for i in 0..n {
        for j in 0..n {
            st.add_particle(Particle::new(
                Vec2 { x: i as f32 * spacing, y: j as f32 * spacing },
                mass,
                MaterialId(0),
            ));
        }
    }
    st
}

fn nan_free(st: &ParticleStorage) -> bool {
    st.iter().all(|p| {
        [p.position.x, p.position.y, p.velocity.x, p.velocity.y, p.density, p.pressure]
            .iter()
            .all(|v| v.is_finite())
    })
}

// ---------------------------------------------------------------- kernels

fn check_kernels(r: &mut Report) {
    let h = 16.0_f32;
    let n = 800; // grid cells per side
    let dx = 2.0 * h / n as f32;

    let (mut w_sum, mut rdw_sum) = (0.0_f64, 0.0_f64);
    for i in 0..n {
        for j in 0..n {
            let x = -h + (i as f32 + 0.5) * dx;
            let y = -h + (j as f32 + 0.5) * dx;
            let r2 = x * x + y * y;
            w_sum += (math::poly6(r2, h) * dx * dx) as f64;
            let g = math::spiky_grad(Vec2 { x, y }, h);
            rdw_sum += ((x * g.x + y * g.y) * dx * dx) as f64;
        }
    }
    r.check(
        "poly6: integral over disc == 1",
        (w_sum - 1.0).abs() < 0.01,
        format!("integral = {w_sum:.4}"),
    );
    // For a normalized kernel in 2D: integral of r . grad(W) dA = -2
    r.check(
        "spiky grad: integral r.grad(W) == -2",
        (rdw_sum + 2.0).abs() < 0.05,
        format!("integral = {rdw_sum:.4}"),
    );

    let g = math::spiky_grad(Vec2 { x: 4.0, y: 0.0 }, h);
    r.check(
        "spiky grad: points toward -r",
        g.x < 0.0 && g.y.abs() < 1e-6,
        format!("grad(4,0) = ({:.5}, {:.5})", g.x, g.y),
    );
    let at_h = math::spiky_grad(Vec2 { x: h, y: 0.0 }, h);
    let at_0 = math::spiky_grad(Vec2 { x: 0.0, y: 0.0 }, h);
    r.check(
        "spiky grad: zero at r=0 and r>=h",
        at_h.x == 0.0 && at_0.x == 0.0 && at_0.y == 0.0,
        format!("grad(h) = {:.5}, grad(0) = {:.5}", at_h.x, at_0.x),
    );
}

// --------------------------------------------------------------- free fall

fn check_free_fall(r: &mut Report) {
    let cfg = SphConfig::builtin();
    let mut st = ParticleStorage::default();
    st.add_particle(Particle::new(Vec2 { x: 0.0, y: 0.0 }, 1.0, MaterialId(0)));
    let mut em: Vec<Emitter> = Vec::new();

    for _ in 0..60 {
        SphSolver::step(&mut st, &mut em, &cfg, DT);
    }
    let p = st.particles[0];
    let expected_v = cfg.gravity.y * 1.0; // g * 1 s
    r.check(
        "free fall: v = g*t after 1 s",
        (p.velocity.y - expected_v).abs() < 1e-2 && p.velocity.x.abs() < 1e-6,
        format!("vy = {:.3}, expected {:.3}", p.velocity.y, expected_v),
    );
}

// ----------------------------------------------------------- rest density

fn check_rest_density(r: &mut Report) {
    let cfg = zero_gravity();
    let spacing = cfg.smoothing_length / 2.0; // common choice: h = 2 * spacing
    let mut st = lattice(21, spacing, 1.0);
    let mut em: Vec<Emitter> = Vec::new();
    SphSolver::step(&mut st, &mut em, &cfg, DT);

    // central particle: full neighbourhood
    let center = 10 * 21 + 10;
    let rho = st.particles[center].density;
    let ratio = rho / cfg.rest_density;
    r.check(
        "lattice (h=2*dx): rho ~ rest_density",
        (0.8..=1.2).contains(&ratio),
        format!("rho = {rho:.4}, rest = {:.2}, ratio = {ratio:.4}", cfg.rest_density),
    );
    let p = st.particles[center].pressure;
    r.check(
        "lattice at rest: pressure is finite",
        p.is_finite(),
        format!("pressure = {p:.4}"),
    );
}

// ------------------------------------------------------------- repulsion

fn check_repulsion(r: &mut Report) {
    let cfg = zero_gravity();
    let mut st = ParticleStorage::default();
    st.add_particle(Particle::new(Vec2 { x: 0.0, y: 0.0 }, 1.0, MaterialId(0)));
    st.add_particle(Particle::new(Vec2 { x: 2.0, y: 0.0 }, 1.0, MaterialId(0)));
    let mut em: Vec<Emitter> = Vec::new();

    let before = st.particles[1].position.x - st.particles[0].position.x;
    for _ in 0..30 {
        SphSolver::step(&mut st, &mut em, &cfg, DT);
    }
    let after = st.particles[1].position.x - st.particles[0].position.x;
    let pmax = st.iter().map(|p| p.pressure).fold(0.0, f32::max);
    r.check(
        "two close particles repel",
        after > before,
        format!("distance {before:.3} -> {after:.3}, pressure max = {pmax:.4}"),
    );
}

// ------------------------------------------------------------- block

fn check_block(r: &mut Report) {
    let cfg = zero_gravity();
    let mut st = lattice(20, cfg.smoothing_length / 2.0, 1.0);
    let mut em: Vec<Emitter> = Vec::new();
    for _ in 0..120 {
        SphSolver::step(&mut st, &mut em, &cfg, DT);
    }
    let n = st.len() as f32;
    let mom_x = st.iter().map(|p| p.velocity.x * p.mass).sum::<f32>();
    let mom_y = st.iter().map(|p| p.velocity.y * p.mass).sum::<f32>();
    let vmax = st
        .iter()
        .map(|p| (p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y).sqrt())
        .fold(0.0, f32::max);

    r.check("block 120 steps: no NaN / inf", nan_free(&st), format!("{n} particles"));
    r.check(
        "block, no gravity: momentum conserved",
        mom_x.abs() < 1e-2 && mom_y.abs() < 1e-2,
        format!("sum(m*v) = ({mom_x:.5}, {mom_y:.5})"),
    );
    r.check(
        "block, no gravity: does not explode",
        vmax < 5.0,
        format!("vmax = {vmax:.4}"),
    );
}

// ------------------------------------------------------------- emitter

fn emitter(pps: f32, lifetime: f32, max: usize) -> Emitter {
    Emitter {
        shape: Shape::Circle(Circle { radius: 5.0 }),
        direction: Vec2 { x: 0.0, y: -1.0 },
        speed: 0.0,
        particles_per_second: pps,
        spread: 0.0,
        particle_mass: 1.0,
        particle_lifetime: lifetime,
        material: MaterialId(0),
        max_particles: max,
        accumulator: 0.0,
        active: true,
    }
}

fn check_emitter(r: &mut Report) {
    let cfg = zero_gravity();

    // rate
    let mut st = ParticleStorage::default();
    let mut em = vec![emitter(400.0, 1000.0, 100_000)];
    for _ in 0..60 {
        SphSolver::step(&mut st, &mut em, &cfg, DT);
    }
    r.check(
        "emitter: rate ~ particles_per_second",
        (st.len() as f32 - 400.0).abs() <= 8.0,
        format!("{} particles after 1 s, expected ~400", st.len()),
    );

    // cap
    let mut st = ParticleStorage::default();
    let mut em = vec![emitter(400.0, 1000.0, 100)];
    for _ in 0..60 {
        SphSolver::step(&mut st, &mut em, &cfg, DT);
    }
    r.check(
        "emitter: max_particles is respected",
        st.len() <= 100,
        format!("{} particles, max 100", st.len()),
    );

    // lifetime: particles older than 0.5 s must disappear
    let mut st = ParticleStorage::default();
    let mut em = vec![emitter(400.0, 0.5, 100_000)];
    for _ in 0..120 {
        SphSolver::step(&mut st, &mut em, &cfg, DT);
    }
    r.check(
        "emitter: particles expire (lifetime)",
        st.len() <= 230,
        format!("{} particles after 2 s, expected ~200 for lifetime 0.5 s", st.len()),
    );
}

fn main() {
    build_logger(TypeLogs::Trace);

    let mut r = Report { pass: 0, fail: 0 };
    check_kernels(&mut r);
    println!();
    check_free_fall(&mut r);
    check_rest_density(&mut r);
    check_repulsion(&mut r);
    check_block(&mut r);
    println!();
    check_emitter(&mut r);

    info!("\nresult: {} passed, {} failed", r.pass, r.fail);
    if r.fail > 0 {
        std::process::exit(1);
    }
}