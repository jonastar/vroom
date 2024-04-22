use crate::{actions::Actions, scene::camera_look_at, GameState};
use bevy::{
    prelude::*,
    utils::{dbg, petgraph::matrix_graph::Zero},
};
use bevy_rapier3d::prelude::*;

pub struct CarPlugin;

#[derive(Component)]
pub struct Car;

/// This plugin handles player related stuff like movement
/// Player logic is only active during the State `GameState::Playing`
impl Plugin for CarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(GameState::Playing), spawn_car)
            .add_systems(OnExit(GameState::Playing), despawn_car)
            .add_systems(
                Update,
                (
                    move_car,
                    /*rotate_car,*/ turn_front_wheels,
                    camera_look_at::<CarBody>,
                    keep_car_awake,
                    down_force,
                )
                    .run_if(in_state(GameState::Playing)),
            );
    }
}

#[derive(Component)]
pub struct CarBody;

fn spawn_car(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Car,
            SpatialBundle::from_transform(Transform::from_translation(Vec3 {
                x: 10.0,
                y: 1.5,
                z: 10.0,
            })),
            // RigidBody::Dynamic,
            // Velocity::zero(),
            // ExternalImpulse::default(),
        ))
        .with_children(|car| {
            // Main body

            let body = car
                .spawn((
                    PbrBundle {
                        mesh: meshes.add(Cuboid::new(1.0, 0.5, 2.0)),
                        material: materials.add(Color::rgb(0.2, 0.1, 0.3)),
                        // transform: Transform::from_translation(Vec),
                        ..default()
                    },
                    CarBody,
                    Collider::cuboid(0.5, 0.25, 1.0),
                    RigidBody::Dynamic,
                    Velocity::zero(),
                    ExternalImpulse::default(),
                    ColliderMassProperties::Density(100.0),
                ))
                .id();

            // driver
            car.spawn(PbrBundle {
                mesh: meshes.add(Cuboid::new(0.2, 0.2, 0.2)),
                material: materials.add(Color::rgb(0.2, 1.0, 0.3)),
                transform: Transform::from_translation(Vec3 {
                    x: 0.0,
                    y: 0.35,
                    z: -0.8,
                }),
                ..default()
            });

            // Front Wheels
            spawn_wheel(
                &mut meshes,
                &mut materials,
                car,
                body,
                Vec3 {
                    x: -0.85,
                    y: -0.15,
                    z: -1.0,
                },
                Wheel::FrontLeft,
            );

            spawn_wheel(
                &mut meshes,
                &mut materials,
                car,
                body,
                Vec3 {
                    x: 0.85,
                    y: -0.15,
                    z: -1.0,
                },
                Wheel::FrontRight,
            );

            // Back wheels

            spawn_wheel(
                &mut meshes,
                &mut materials,
                car,
                body,
                Vec3 {
                    x: -0.85,
                    y: -0.15,
                    z: 1.0,
                },
                Wheel::RearLeft,
            );

            spawn_wheel(
                &mut meshes,
                &mut materials,
                car,
                body,
                Vec3 {
                    x: 0.85,
                    y: -0.15,
                    z: 1.0,
                },
                Wheel::RearRight,
            );
        });
}

#[derive(Component, Clone)]
enum Wheel {
    FrontRight,
    FrontLeft,
    RearRight,
    RearLeft,
}

impl Wheel {
    pub fn is_front(&self) -> bool {
        matches!(self, Self::FrontLeft | Self::FrontRight)
    }

    pub fn is_right(&self) -> bool {
        matches!(self, Self::FrontRight | Self::RearRight)
    }
}

#[derive(Component)]
struct WheelAxle;

const MAX_STEERING_ANGLE: f32 = 0.610865;

fn spawn_wheel(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    car_parent: &mut ChildBuilder,
    body: Entity,
    position: Vec3,
    wheel_kind: Wheel,
) {
    let wheel_start_rot = Quat::from_rotation_z(1.5708);

    let axle_mass = bevy_rapier3d::rapier::dynamics::MassProperties::from_ball(100.0, 0.25);

    let mut axle = car_parent.spawn((
        SpatialBundle {
            transform: Transform::from_translation(position),
            // .with_rotation(wheel_start_rot),
            ..Default::default()
        },
        RigidBody::Dynamic,
        Velocity::zero(),
        ExternalImpulse::default(),
        WheelAxle,
        AdditionalMassProperties::MassProperties(MassProperties::from_rapier(axle_mass, 1.0)),
        wheel_kind.clone(),
    ));

    let mut axle_locked_axes = JointAxesMask::Z | JointAxesMask::ANG_X | JointAxesMask::ANG_Z;
    if !wheel_kind.is_front() {
        axle_locked_axes |= JointAxesMask::ANG_Y;
    }

    let suspension_height = 0.22;
    let suspension_width = 0.15;

    let mut suspension_joint = GenericJointBuilder::new(axle_locked_axes)
        .limits(JointAxis::Y, [-suspension_height, 0.0])
        .limits(
            JointAxis::X,
            if wheel_kind.is_right() {
                [0.0, suspension_width]
            } else {
                [-suspension_width, 0.0]
            },
        )
        .motor_position(JointAxis::Y, -suspension_height, 3000.0, 1.0e2)
        .motor_position(
            JointAxis::X,
            if wheel_kind.is_right() {
                suspension_width
            } else {
                -suspension_width
            },
            2000.0,
            1.0e2,
        )
        .local_anchor1(position);

    if wheel_kind.is_front() {
        suspension_joint =
            suspension_joint.limits(JointAxis::AngY, [-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE]);
    }

    axle.insert(ImpulseJoint::new(body, suspension_joint));
    let axle_id = axle.id();

    let wheel_joint = RevoluteJointBuilder::new(Vec3::X).motor_max_force(100.0);
    // .motor_model(MotorModel::ForceBased)
    // .motor_max_force(50.0);

    let mut builder = car_parent.spawn((
        SpatialBundle {
            transform: Transform::from_translation(position),
            // .with_rotation(wheel_start_rot),
            ..Default::default()
        },
        RigidBody::Dynamic,
        Velocity::zero(),
        ExternalImpulse::default(),
        ImpulseJoint::new(axle_id, wheel_joint),
        wheel_kind,
    ));

    // let a = Quat::from_rotation_x(1.0) * Vec3::X;
    // let joint = RevoluteJointBuilder::new(Vec3::X)
    //     .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
    //     .local_anchor1(position)
    //     .motor_max_force(200.0)
    //     .motor_model(MotorModel::AccelerationBased);

    builder.with_children(|wheel| {
        wheel.spawn((
            PbrBundle {
                mesh: meshes.add(Cylinder::new(0.25, 0.15)),
                material: materials.add(Color::rgb(0.2, 0.1, 0.6)),
                transform: Transform::from_rotation(wheel_start_rot),
                ..default()
            },
            // Collider::round_cylinder(0.015, 0.15, 0.1),
            // Collider::capsule(Vec3::new(0.0, -0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 0.25),
            Collider::ball(0.25),
            // Restitution {
            //     coefficient: 10.0,
            //     combine_rule: CoefficientCombineRule::Min,
            // },
            ColliderMassProperties::Density(50.0),
            Friction::new(2.0),
        ));
    });
}
fn despawn_car() {}

fn move_car(
    actions: Res<Actions>,
    mut car_query: Query<(&mut ImpulseJoint, &mut Velocity, &Wheel)>,
) {
    let input = actions.player_movement.unwrap_or_default();

    let steering = -input.x;

    let differential_strength = 0.5;
    let sideways_shift = (MAX_STEERING_ANGLE * steering).sin() * differential_strength;
    let speed_diff = if sideways_shift > 0.0 {
        f32::hypot(1.0, sideways_shift)
    } else {
        1.0 / f32::hypot(1.0, sideways_shift)
    };

    let move_speed = 500.0;

    for (mut joint, velocity, wheel) in &mut car_query {
        let apply_diff = if matches!(wheel, Wheel::FrontLeft | Wheel::RearLeft) {
            speed_diff
        } else {
            1.0 / speed_diff
        };

        // let current_vel = velocity.angvel.length();
        // if()
        // let target_vel = -input.y * (current_vel + 1.0);

        joint
            .data
            .set_motor_velocity(JointAxis::AngX, -input.y * move_speed * apply_diff, 100.0);
    }
}

fn turn_front_wheels(
    time: Res<Time>,
    actions: Res<Actions>,
    mut joint_query: Query<(&mut ImpulseJoint, &Wheel), With<WheelAxle>>,
    mut steering_wheel: Local<f32>,
    cat_bodies: Query<(&Transform, &Velocity), (With<CarBody>, Without<Wheel>)>,
) {
    let x_input = actions.player_movement.map(|v| v.x).unwrap_or_default();
    *steering_wheel = steering_wheel.lerp(x_input / 1.5, time.delta_seconds() * 10.0);

    let (body_transform, body_vel) = cat_bodies.single();
    let speed = body_vel
        .linvel
        .project_onto(body_transform.forward().into())
        .length();

    let turn_modifier = if speed < 5.0 {
        1.0
    } else {
        1.0 / (speed / 5.0)
    };

    for (mut joint, wheel) in &mut joint_query {
        if !wheel.is_front() {
            continue;
        }

        joint.data.set_motor_position(
            JointAxis::AngY,
            (MAX_STEERING_ANGLE * turn_modifier) * -x_input,
            1.0e4,
            1.0e3,
        );
    }
}

fn keep_car_awake(mut query: Query<&mut Sleeping, Or<(With<CarBody>, With<Wheel>)>>) {
    for mut sleeping in &mut query {
        sleeping.sleeping = false;
    }
}

fn down_force(
    time: Res<Time>,
    mut cat_bodies: Query<
        (&Transform, &Velocity, &mut ExternalImpulse),
        (With<CarBody>, Without<Wheel>),
    >,
) {
    for (trans, vel, mut impulse) in &mut cat_bodies {
        let forward = trans.forward();

        let projected = vel.linvel.project_onto(forward.into());
        if projected.length() < 1.0 {
            continue;
        }

        let angle = forward.angle_between(projected);
        if angle > 90.0 || angle < -90.0 {
            // Going backwards
        } else {
            let down_force = projected.length() * time.delta_seconds();
            impulse.impulse += Vec3::new(0.0, down_force * -50.0, 0.0);
        }
    }
}
