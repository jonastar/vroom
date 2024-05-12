use crate::{
    actions::Actions,
    editor::StartPoint,
    raycast_vehicle_controller::{DynamicRayCastVehicleController, WheelDesc, WheelTuning},
    scene::camera_look_at,
    GameState,
};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

pub struct CarPlugin;

#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct Car {
    wheel_tuning: WheelTuning,
}

/// This plugin handles player related stuff like movement
/// Player logic is only active during the State `GameState::Playing`
impl Plugin for CarPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Car>()
            .add_systems(OnEnter(GameState::Playing), spawn_car)
            .add_systems(OnExit(GameState::Playing), despawn_car)
            .add_systems(
                FixedUpdate,
                (move_car_raycast, turn_front_wheels, apply_wheel_tuning),
            )
            .add_systems(
                FixedUpdate,
                (camera_look_at::<CarBody>).run_if(in_state(GameState::Playing)),
            );
        // .add_systems(
        //     Update,
        //     (
        //         move_car,
        //         /*rotate_car,*/ turn_front_wheels,
        //         camera_look_at::<CarBody>,
        //         keep_car_awake,
        //         down_force,
        //     )
        //         .run_if(in_state(GameState::Playing)),
        // );
    }
}

const MOTOR_STRENGTH: f32 = 75.0;
const START_POSITION: Vec3 = Vec3 {
    x: 20.0,
    y: -3.0,
    z: 20.0,
};

#[derive(Component)]
pub struct CarBody;

fn spawn_car(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_car_helper(
        START_POSITION,
        Quat::IDENTITY,
        &mut commands,
        &mut meshes,
        &mut materials,
    );
}

pub fn spawn_car_helper(
    start_position: Vec3,
    start_rotation: Quat,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let tuning = WheelTuning {
        suspension_stiffness: 20.0,
        suspension_compression: 1.0,
        suspension_damping: 1.0,
        max_suspension_travel: 0.25,
        side_friction_stiffness: 1.0,
        friction_slip: 1.5,
        max_suspension_force: 1000.0,
    };

    commands
        .spawn((
            Car {
                wheel_tuning: tuning.clone(),
            },
            // SpatialBundle::from_transform(Transform::from_translation(START_POSITION)),
            PbrBundle {
                mesh: meshes.add(Cuboid::new(1.25, 0.5, 2.5)),
                material: materials.add(Color::rgb(0.2, 0.1, 0.3)),
                transform: Transform::from_translation(start_position)
                    .with_rotation(start_rotation),
                ..default()
            },
            CarBody,
            Collider::round_cuboid(0.525, 0.15, 1.15, 0.1),
            RigidBody::Dynamic,
            Velocity::zero(),
            ExternalImpulse::default(),
            ColliderMassProperties::Density(100.0),
            AdditionalMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::new(0.0, -1.0, 0.0),
                mass: 50.0,
                principal_inertia_local_frame: Quat::IDENTITY,
                principal_inertia: Vec3::ONE,
            }),
            DynamicRayCastVehicleController::new(),
            Damping {
                angular_damping: 1.0,
                linear_damping: 0.1,
            },
            Sleeping::default(),
            ReadMassProperties::default(),
            TransformInterpolation::default(),
            Friction {
                coefficient: 0.0,
                combine_rule: CoefficientCombineRule::Min,
            },
            // RigidBody::Dynamic,W
            // Velocity::zero(),
            // ExternalImpulse::default(),
        ))
        .insert(Name::new("Car vroom"))
        .with_children(|car| {
            // Main body

            // let body = car
            //     .spawn((
            //         PbrBundle {
            //             mesh: meshes.add(Cuboid::new(1.0, 0.5, 2.0)),
            //             material: materials.add(Color::rgb(0.2, 0.1, 0.3)),
            //             // transform: Transform::from_translation(Vec),
            //             ..default()
            //         },
            //         CarBody,
            //         Collider::cuboid(0.5, 0.25, 1.0),
            //         RigidBody::Dynamic,
            //         Velocity::zero(),
            //         ExternalImpulse::default(),
            //         ColliderMassProperties::Density(100.0),
            //         Resetable,
            //         DynamicRayCastVehicleController::new(),
            //         Damping {
            //             angular_damping: 1.0,
            //             linear_damping: 0.1,
            //         },
            //     ))
            //     .id();

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
                meshes,
                materials,
                car,
                car.parent_entity(),
                Vec3 {
                    x: -0.70,
                    y: 0.0,
                    z: -1.0,
                },
                Wheel::FrontLeft,
                tuning.clone(),
            );

            spawn_wheel(
                meshes,
                materials,
                car,
                car.parent_entity(),
                Vec3 {
                    x: 0.70,
                    y: 0.0,
                    z: -1.0,
                },
                Wheel::FrontRight,
                tuning.clone(),
            );

            // Back wheels

            spawn_wheel(
                meshes,
                materials,
                car,
                car.parent_entity(),
                Vec3 {
                    x: -0.70,
                    y: 0.0,
                    z: 1.0,
                },
                Wheel::RearLeft,
                tuning.clone(),
            );

            spawn_wheel(
                meshes,
                materials,
                car,
                car.parent_entity(),
                Vec3 {
                    x: 0.70,
                    y: 0.0,
                    z: 1.0,
                },
                Wheel::RearRight,
                tuning.clone(),
            );
        });
}

// principal_inertia: Vec3(
//     87.814095,
//     104.11991,
//     27.594929,
// ),

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

const MAX_STEERING_ANGLE: f32 = 0.410865;

fn spawn_wheel(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    car_parent: &mut ChildBuilder,
    body: Entity,
    position: Vec3,
    wheel_kind: Wheel,
    tuning: WheelTuning,
) {
    car_parent.spawn((
        SpatialBundle {
            transform: Transform::from_translation(
                position
                    + Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
            ),
            // .with_rotation(wheel_start_rot),
            ..Default::default()
        },
        Collider::cuboid(0.1, 0.1, 0.1),
        ColliderMassProperties::Density(50.0),
    ));

    let mut builder = car_parent.spawn((
        SpatialBundle {
            transform: Transform::from_translation(position),
            // .with_rotation(wheel_start_rot),
            ..Default::default()
        },
        wheel_kind,
        // Ccd::enabled(),
        crate::raycast_vehicle_controller::Wheel::new(WheelDesc::new(
            position,
            -Vec3::Y,
            Vec3::X,
            0.25,
            0.25,
            &tuning,
        )),
        // Collider::ball(0.25),
    ));

    // let a = Quat::from_rotation_x(1.0) * Vec3::X;
    // let joint = RevoluteJointBuilder::new(Vec3::X)
    //     .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
    //     .local_anchor1(position)
    //     .motor_max_force(200.0)
    //     .motor_model(MotorModel::AccelerationBased);

    builder.with_children(|wheel| {
        wheel.spawn((PbrBundle {
            mesh: meshes.add(Cylinder::new(0.25, 0.15)),
            material: materials.add(Color::rgb(0.2, 0.1, 0.6)),
            transform: Transform::from_rotation(Quat::from_rotation_z(90f32.to_radians())),
            ..default()
        },));
    });
}
fn despawn_car() {}

fn move_car_raycast(
    actions: Res<Actions>,
    // time: Res<Time>,
    mut car_query: Query<(&mut crate::raycast_vehicle_controller::Wheel, &Wheel)>,
) {
    let input = actions.player_movement.unwrap_or_default();

    for (mut wheel, our_wheel) in &mut car_query {
        if actions.breaking {
            wheel.brake = 10.0;
        } else {
            wheel.brake = 0.0;
        }

        if our_wheel.is_front() {
            continue;
        }

        let engine_force = input.y * 1000.0;

        wheel.engine_force = engine_force;
    }
}

fn turn_front_wheels(
    time: Res<Time>,
    actions: Res<Actions>,
    // mut joint_query: Query<(&mut ImpulseJoint, &Wheel), With<WheelAxle>>,
    mut steering_wheel: Local<f32>,
    cat_bodies: Query<(&Transform, &Velocity), (With<CarBody>, Without<Wheel>)>,

    mut car_query: Query<(&mut crate::raycast_vehicle_controller::Wheel, &Wheel)>,
) {
    let x_input = actions.player_movement.map(|v| v.x).unwrap_or_default();
    *steering_wheel = steering_wheel.lerp(x_input, time.delta_seconds() * 10.0);

    let Ok((body_transform, body_vel)) = cat_bodies.get_single() else {
        return;
    };

    let speed = body_vel
        .linvel
        .project_onto(body_transform.forward().into())
        .length();

    let turn_modifier = if speed < 7.0 {
        1.0
    } else {
        1.0 / (speed / 10.0)
    };
    let turn_modifier = turn_modifier.clamp(0.1, 1.0);

    for (mut wheel, our_wheel) in &mut car_query {
        if !our_wheel.is_front() {
            continue;
        }

        wheel.steering = (MAX_STEERING_ANGLE * turn_modifier) * -*steering_wheel;
    }
}

fn apply_wheel_tuning(
    cars: Query<(&Car, &Children), Changed<Car>>,
    mut wheels: Query<&mut crate::raycast_vehicle_controller::Wheel>,
) {
    for (car, children) in &cars {
        for child in children.iter() {
            let Ok(mut wheel) = wheels.get_mut(*child) else {
                continue;
            };

            wheel.apply_tuning(&car.wheel_tuning);
        }
    }
}

// fn keep_car_awake(mut query: Query<&mut Sleeping, Or<(With<CarBody>, With<Wheel>)>>) {
//     for mut sleeping in &mut query {
//         sleeping.sleeping = false;
//     }
// }

// fn down_force(
//     time: Res<Time>,
//     mut cat_bodies: Query<
//         (&Transform, &Velocity, &mut ExternalImpulse),
//         (With<CarBody>, Without<Wheel>),
//     >,
// ) {
//     for (trans, vel, mut impulse) in &mut cat_bodies {
//         let forward = trans.forward();

//         let projected = vel.linvel.project_onto(forward.into());
//         if projected.length() < 1.0 {
//             continue;
//         }

//         let angle = forward.angle_between(vel.linvel).to_degrees();
//         if angle > 90.0 || angle < -90.0 {
//             // Going backwards, do not apply down force
//         } else {
//             let down_force = projected.length() * time.delta_seconds();
//             let down_force = down_force.clamp(0.0, 0.4);
//             dbg!(down_force);
//             impulse.impulse += Vec3::new(0.0, down_force * -100.0, 0.0);
//         }
//     }
// }

pub fn respawn_car_on_reset_action(
    actions: Res<Actions>,
    mut commands: Commands,
    query: Query<&Transform, With<StartPoint>>,
    mut existing_cars: Query<&mut Transform, (With<CarBody>, Without<StartPoint>)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !actions.reset {
        return;
    }

    let start = query.single();
    for mut existing in &mut existing_cars {
        existing.translation = start.translation;
        existing.rotation = start.rotation;
        // commands.entity(existing).despawn_recursive();
        return;
    }

    spawn_car_helper(
        start.translation,
        start.rotation,
        &mut commands,
        &mut meshes,
        &mut materials,
    );
}
