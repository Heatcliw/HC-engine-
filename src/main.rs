use hc_physics::*;

fn main() {
    // Materials
    let mut materials = MaterialRegister::new();

    let water = materials.register(Material {
        name: "water".into(),
        ..Default::default() 
    });

    // Storage
    let mut storage = ParticleStorage::default();

    // Emitter
    let mut emitters = vec![
        Emitter {
            shape: Shape::Rhombus(Rhombus {
                side: 10.0,
                angle: math::PI / 3.0, // 60°
            }),
            direction: [0.0, -1.0],
            speed: 30.0,
            particles_per_second: 400.0,
            spread: 0.02,
            particle_mass: 1.0,
            particle_lifetime: 10.0,
            material: water,
            max_particles: 10_000,
            accumulator: 0.0,
            active: true,
        }
    ];
    
    // Config
    let config = SphConfig::default();

    let dt = 1.0 / 60.0;

    // Run simulation
    for frame in 0..300 {
        SphSolver::step(
            &mut storage,
            &mut emitters,
            &config,
            dt,
        );

       if frame % 60 == 0 {
            let max_density = storage
                .particles
                .iter()
                .map(|p| p.density)
                .fold(0.0, f32::max);

            let min_density = storage
                .particles
                .iter()
                .map(|p| p.density)
                .fold(f32::INFINITY, f32::min);

            let avg_density = storage
                .particles
                .iter()
                .map(|p| p.density)
                .sum::<f32>()
                / storage.len().max(1) as f32;

            println!(
                "frame {:3} | particles: {} | density[min={:.2}, avg={:.2}, max={:.2}]",
                frame,
                storage.len(),
                min_density,
                avg_density,
                max_density,
            );
        }
    }

    println!("Simulation finished");
}