extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point2, Vector2}; // For configuring and positioning bodies.
use ncollide2d::shape::{ConvexPolygon, Cuboid, ShapeHandle}; // Shapes for colliders.
use nphysics2d::algebra::Velocity2;
use nphysics2d::force_generator::{ForceGenerator, ForceGeneratorHandle};
use nphysics2d::object::{BodyHandle, BodySet, Material}; // Body handle and collider material.
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::volumetric::Volumetric; // To retrieve the center of mass and inertia properties of a shape.
use nphysics2d::world::World;           // The physics world to be initialized.
use nphysics_testbed2d::Testbed; // The testbed to display/run the simulation.

pub mod movement;
use self::movement::Movement;

const COLLIDER_MARGIN: f32 = 0.01;
const ACCELERATION_PER_TICK: f32 = 250.0;

#[inline(always)]
fn pt2(x: f32, y: f32) -> Point2<f32> {
    Point2::new(x, y)
}

fn create_barrier(width: f32, height: f32) -> Vec<Point2<f32>> {
    let half_width = width / 2.0;
    let half_height = height / 2.0;

    vec![
        pt2(half_width + 0.5, half_height + 0.75),
        pt2(half_width - 0.5, -half_height + 0.55),
        pt2(-half_width - 0.75, -half_height + 0.875),
        pt2(-half_width - 0.35, half_height + 0.45),
    ]
}

pub struct SpaceshipForceGenerator {
    movement: Movement,
    player_body_handle: BodyHandle,
}

impl SpaceshipForceGenerator {
    pub fn new(player_body_handle: BodyHandle, movement: Movement) -> Self {
        SpaceshipForceGenerator {
            movement,
            player_body_handle,
        }
    }
}

impl ForceGenerator<f32> for SpaceshipForceGenerator {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        let mut acceleration: Vector2<f32> = self.movement.into();
        acceleration *= ACCELERATION_PER_TICK;
        let acceleration = Velocity2::new(acceleration, 0.0);
        let mut part = bodies.body_part_mut(self.player_body_handle);
        let force = part.as_ref().inertia() * acceleration;
        part.apply_force(&force);

        true
    }
}

fn main() {
    let mut world: World<f32> = World::new();

    let barrier_height = 500.0;
    let barrier_width = 100.0;
    let barrier_pos = Isometry2::new(Vector2::new(300.0, 0.0), 0.0);

    let barrier_shape =
        ConvexPolygon::try_new(create_barrier(barrier_width, barrier_height)).unwrap();
    let barrier_shape_handle = ShapeHandle::new(barrier_shape);

    world.add_collider(
        COLLIDER_MARGIN,
        barrier_shape_handle,
        BodyHandle::ground(),
        barrier_pos,
        Material::default(),
    );

    let spaceship_size = 30.0;
    let spaceship_pos = Isometry2::new(Vector2::new(0.0, 150.0), 0.0);
    let spaceship_shape_handle = ShapeHandle::new(Cuboid::new(Vector2::repeat(
        spaceship_size - COLLIDER_MARGIN,
    )));

    let inertia = spaceship_shape_handle.inertia(1.0);
    let com = spaceship_shape_handle.center_of_mass();

    let body_handle = world.add_rigid_body(spaceship_pos, inertia, com);
    world.add_collider(
        COLLIDER_MARGIN,
        spaceship_shape_handle,
        body_handle,
        Isometry2::identity(),
        Material::default(),
    );

    let force_gen = SpaceshipForceGenerator::new(body_handle, Movement::Right);
    world.add_force_generator(force_gen);

    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(150.0, 150.0), 1.0);
    testbed.run();
}
