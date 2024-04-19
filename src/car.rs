use crate::{actions::Actions, scene::camera_look_at, GameState};
use bevy::prelude::*;
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
                x: 0.0,
                y: 1.5,
                z: 0.0,
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
                    ColliderMassProperties::Density(5.0),
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
                    y: -0.25,
                    z: -1.0,
                },
                true,
            );

            spawn_wheel(
                &mut meshes,
                &mut materials,
                car,
                body,
                Vec3 {
                    x: 0.85,
                    y: -0.25,
                    z: -1.0,
                },
                true,
            );

            // Back wheels

            spawn_wheel(
                &mut meshes,
                &mut materials,
                car,
                body,
                Vec3 {
                    x: -0.85,
                    y: -0.25,
                    z: 1.0,
                },
                false,
            );

            spawn_wheel(
                &mut meshes,
                &mut materials,
                car,
                body,
                Vec3 {
                    x: 0.85,
                    y: -0.25,
                    z: 1.0,
                },
                false,
            );
        });
}

#[derive(Component)]
struct FrontWheel;

#[derive(Component)]
struct Wheel;

#[derive(Component)]
struct WheelAxle;

const MAX_STEERING_ANGLE: f32 = 0.610865;

fn spawn_wheel(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    car_parent: &mut ChildBuilder,
    body: Entity,
    position: Vec3,
    is_front_wheel: bool,
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
    ));

    if is_front_wheel {
        axle.insert(FrontWheel);
    }

    let mut axle_locked_axes =
        JointAxesMask::X | JointAxesMask::Z | JointAxesMask::ANG_X | JointAxesMask::ANG_Z;
    if !is_front_wheel {
        axle_locked_axes |= JointAxesMask::ANG_Y;
    }

    let suspension_height = 0.12;

    let mut suspension_joint = GenericJointBuilder::new(axle_locked_axes)
        .limits(JointAxis::Y, [0.0, suspension_height])
        .motor_position(JointAxis::Y, 0.0, 1.0e4, 1.0e3)
        .local_anchor1(position);

    if is_front_wheel {
        suspension_joint =
            suspension_joint.limits(JointAxis::AngY, [-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE]);
    }

    axle.insert(ImpulseJoint::new(body, suspension_joint));
    let axle_id = axle.id();

    let wheel_joint = RevoluteJointBuilder::new(Vec3::X);

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
        Wheel,
    ));

    if is_front_wheel {
        builder.insert(FrontWheel);
    }

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
            Collider::round_cylinder(0.075, 0.15, 0.1),
            Restitution {
                coefficient: 0.1,
                combine_rule: CoefficientCombineRule::Average,
            },
            ColliderMassProperties::Density(50.0),
            Friction::new(2.0),
        ));
    });
}
fn despawn_car() {}

fn move_car(actions: Res<Actions>, mut car_query: Query<&mut ImpulseJoint, With<Wheel>>) {
    let input = actions.player_movement.unwrap_or_default();

    // let differential_strength = 0.5;
    // let sideways_shift = (MAX_STEERING_ANGLE * steering).sin() * differential_strength;
    // let speed_diff = if sideways_shift > 0.0 {
    //     f32::hypot(1.0, sideways_shift)
    // } else {
    //     1.0 / f32::hypot(1.0, sideways_shift)
    // };

    let move_speed = 300.0;
    for mut joint in &mut car_query {
        joint
            .data
            .set_motor_velocity(JointAxis::AngX, move_speed * -input.y, 10.0);
    }
}

fn turn_front_wheels(
    time: Res<Time>,
    actions: Res<Actions>,
    mut joint_query: Query<&mut ImpulseJoint, (With<WheelAxle>, With<FrontWheel>)>,
    mut steering_wheel: Local<f32>,
) {
    let x_input = actions.player_movement.map(|v| v.x).unwrap_or_default();
    *steering_wheel = steering_wheel.lerp(x_input / 1.5, time.delta_seconds() * 10.0);

    for mut joint in &mut joint_query {
        joint
            .data
            .set_motor_position(JointAxis::AngY, MAX_STEERING_ANGLE * -x_input, 1.0e4, 1.0e3);
    }
}

fn keep_car_awake(mut query: Query<&mut Sleeping, Or<(With<CarBody>, With<Wheel>)>>) {
    for mut sleeping in &mut query {
        sleeping.sleeping = false;
    }
}
