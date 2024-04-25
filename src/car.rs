use crate::{
    actions::Actions,
    raycast_vehicle_controller::{DynamicRayCastVehicleController, WheelDesc},
    reset_transform::Resetable,
    scene::camera_look_at,
    GameState,
};
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
                FixedUpdate,
                (
                    move_car_raycast,
                    turn_front_wheels,
                    camera_look_at::<CarBody>,
                )
                    .run_if(in_state(GameState::Playing)),
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
    commands
        .spawn((
            Car,
            // SpatialBundle::from_transform(Transform::from_translation(START_POSITION)),
            PbrBundle {
                mesh: meshes.add(Cuboid::new(1.0, 0.5, 2.0)),
                material: materials.add(Color::rgb(0.2, 0.1, 0.3)),
                transform: Transform::from_translation(START_POSITION),
                ..default()
            },
            CarBody,
            Collider::cuboid(0.5, 0.25, 1.0),
            RigidBody::Dynamic,
            Velocity::zero(),
            ExternalImpulse::default(),
            ColliderMassProperties::Density(100.0),
            Resetable,
            DynamicRayCastVehicleController::new(),
            Damping {
                angular_damping: 1.0,
                linear_damping: 0.1,
            },
            Sleeping::default(),
            ReadMassProperties::default(),
            TransformInterpolation::default(),
            // RigidBody::Dynamic,
            // Velocity::zero(),
            // ExternalImpulse::default(),
        ))
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
                &mut meshes,
                &mut materials,
                car,
                car.parent_entity(),
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
                car.parent_entity(),
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
                car.parent_entity(),
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
                car.parent_entity(),
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

const MAX_STEERING_ANGLE: f32 = 0.410865;

fn spawn_wheel(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    car_parent: &mut ChildBuilder,
    body: Entity,
    position: Vec3,
    wheel_kind: Wheel,
) {
    // let wheel_start_rot = Quat::from_rotation_z(1.5708);

    // let axle_mass = bevy_rapier3d::rapier::dynamics::MassProperties::from_ball(100.0, 0.25);

    // let mut axle = car_parent.spawn((
    //     SpatialBundle {
    //         transform: Transform::from_translation(position),
    //         // .with_rotation(wheel_start_rot),
    //         ..Default::default()
    //     },
    //     RigidBody::Dynamic,
    //     Velocity::zero(),
    //     ExternalImpulse::default(),
    //     WheelAxle,
    //     AdditionalMassProperties::MassProperties(MassProperties::from_rapier(axle_mass, 1.0)),
    //     wheel_kind.clone(),
    //     Resetable,
    // ));

    // let mut axle_locked_axes = JointAxesMask::Z | JointAxesMask::ANG_X | JointAxesMask::ANG_Z;
    // if !wheel_kind.is_front() {
    //     axle_locked_axes |= JointAxesMask::ANG_Y;
    // }

    // let suspension_height = 0.22;
    // let suspension_width = 0.15;

    // let mut suspension_joint = GenericJointBuilder::new(axle_locked_axes)
    //     .limits(JointAxis::Y, [-suspension_height, 0.0])
    //     .limits(
    //         JointAxis::X,
    //         if wheel_kind.is_right() {
    //             [0.0, suspension_width]
    //         } else {
    //             [-suspension_width, 0.0]
    //         },
    //     )
    //     .motor_position(JointAxis::Y, -suspension_height, 3000.0, 1000.0)
    //     .motor_position(
    //         JointAxis::X,
    //         if wheel_kind.is_right() {
    //             suspension_width
    //         } else {
    //             -suspension_width
    //         },
    //         10000.0,
    //         1000.0,
    //     )
    //     .local_anchor1(position);

    // if wheel_kind.is_front() {
    //     suspension_joint =
    //         suspension_joint.limits(JointAxis::AngY, [-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE]);
    // }

    // axle.insert(ImpulseJoint::new(body, suspension_joint));
    // let axle_id = axle.id();

    // let mut wheel_joint = RevoluteJointBuilder::new(Vec3::X)
    //     .motor_max_force(MOTOR_STRENGTH)
    //     .build();
    // wheel_joint.set_contacts_enabled(false);
    // // .motor_model(MotorModel::ForceBased)
    // // .motor_max_force(50.0);

    let mut builder = car_parent.spawn((
        SpatialBundle {
            transform: Transform::from_translation(position),
            // .with_rotation(wheel_start_rot),
            ..Default::default()
        },
        wheel_kind,
        // Ccd::enabled(),
        Resetable,
        crate::raycast_vehicle_controller::Wheel::new(WheelDesc {
            chassis_connection_cs: position,
            direction_cs: -Vec3::Y,
            axle_cs: Vec3::X,
            suspension_rest_length: 0.25,
            radius: 0.25,

            suspension_stiffness: 20.0,
            damping_compression: 1.0,
            damping_relaxation: 1.0,
            max_suspension_travel: 1.0,
            side_friction_stiffness: 1.0,
            friction_slip: 10.5,
            max_suspension_force: 6000.0,
        }),
        // Collider::ball(0.25),
    ));

    // let a = Quat::from_rotation_x(1.0) * Vec3::X;
    // let joint = RevoluteJointBuilder::new(Vec3::X)
    //     .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
    //     .local_anchor1(position)
    //     .motor_max_force(200.0)
    //     .motor_model(MotorModel::AccelerationBased);

    // builder.with_children(|wheel| {
    //     wheel.spawn((
    //         PbrBundle {
    //             mesh: meshes.add(Cylinder::new(0.25, 0.15)),
    //             material: materials.add(Color::rgb(0.2, 0.1, 0.6)),
    //             transform: Transform::from_rotation(wheel_start_rot),
    //             ..default()
    //         },
    //         // Collider::round_cylinder(0.015, 0.15, 0.1),
    //         // Collider::capsule(Vec3::new(0.0, -0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 0.25),
    //         Collider::ball(0.25),
    //         Restitution {
    //             coefficient: 0.0,
    //             combine_rule: CoefficientCombineRule::Min,
    //         },
    //         ColliderMassProperties::Density(10.0),
    //         Friction::new(2.0),
    //         // Ccd::enabled(),
    //         Resetable,
    //     ));
    // });
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
        1.0 / f32::hypot(1.0, sideways_shift)
    } else {
        f32::hypot(1.0, sideways_shift)
    };

    dbg!(speed_diff);

    let move_speed = 10000.0;

    for (mut joint, velocity, wheel) in &mut car_query {
        // let apply_diff = if matches!(wheel, Wheel::FrontLeft | Wheel::RearLeft) {
        //     speed_diff
        // } else {
        //     1.0 / speed_diff
        // };
        let apply_diff = 1.0;
        // let current_vel = velocity.angvel.length();
        // if()
        // let target_vebut i think l = -input.y * (current_vel + 1.0);

        joint
            .data
            .set_motor_velocity(JointAxis::AngX, -input.y * move_speed * apply_diff, 1.0);
        joint
            .data
            .set_motor_max_force(JointAxis::AngX, MOTOR_STRENGTH * apply_diff);
    }
}

fn move_car_raycast(
    actions: Res<Actions>,
    // time: Res<Time>,
    mut car_query: Query<(&mut crate::raycast_vehicle_controller::Wheel, &Wheel)>,
) {
    let input = actions.player_movement.unwrap_or_default();

    // fn update_vehicle_controller(&mut self, events: &ButtonInput<KeyCode>) {
    //     if self.state.running == RunMode::Stop {
    //         return;
    //     }

    //     if let Some(vehicle) = &mut self.state.vehicle_controller {
    //         let mut engine_force = 0.0;
    //         let mut steering_angle = 0.0;

    //         for key in events.get_pressed() {
    //             match *key {
    //                 KeyCode::ArrowRight => {
    //                     steering_angle += -0.7;
    //                 }
    //                 KeyCode::ArrowLeft => {
    //                     steering_angle += 0.7;
    //                 }
    //                 KeyCode::ArrowUp => {
    //                     engine_force += 30.0;
    //                 }
    //                 KeyCode::ArrowDown => {
    //                     engine_force += -30.0;
    //                 }
    //                 _ => {}
    //             }
    //         }

    //         let wheels = vehicle.wheels_mut();
    //         wheels[0].engine_force = engine_force;
    //         wheels[0].steering = steering_angle;
    //         wheels[1].engine_force = engine_force;
    //         wheels[1].steering = steering_angle;

    //         vehicle.update_vehicle(
    //             self.harness.physics.integration_parameters.dt,
    //             &mut self.harness.physics.bodies,
    //             &self.harness.physics.colliders,
    //             &self.harness.physics.query_pipeline,
    //             QueryFilter::exclude_dynamic().exclude_rigid_body(vehicle.chassis),
    //         );
    //     }
    // }

    for (mut wheel, our_wheel) in &mut car_query {
        if !our_wheel.is_front() {
            continue;
        }

        let engine_force = input.y * 700.0;
        // let steering = (input.x * MAX_STEERING_ANGLE) + (180.0f32).to_radians();
        // let target_steering = -input.x * MAX_STEERING_ANGLE;
        // let steering = wheel
        //     .steering
        //     .lerp(target_steering, time.delta_seconds() * 10.0);
        // let steering = -input.x * MAX_STEERING_ANGLE;

        wheel.engine_force = engine_force;
        // wheel.steering = steering;
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

    let (body_transform, body_vel) = cat_bodies.single();
    let speed = body_vel
        .linvel
        .project_onto(body_transform.forward().into())
        .length();

    let turn_modifier = if speed < 7.0 {
        1.0
    } else {
        1.0 / (speed / 7.0)
    };
    let turn_modifier = turn_modifier.clamp(0.1, 1.0);

    // for (mut joint, wheel) in &mut joint_query {
    //     if !wheel.is_front() {
    //         continue;
    //     }

    //     joint.data.set_motor_position(
    //         JointAxis::AngY,
    //         (MAX_STEERING_ANGLE * turn_modifier) * -*steering_wheel,
    //         1.0e4,
    //         1.0e3,
    //     );
    // }

    for (mut wheel, our_wheel) in &mut car_query {
        if !our_wheel.is_front() {
            continue;
        }

        // let steering = (input.x * MAX_STEERING_ANGLE) + (180.0f32).to_radians();
        // let target_steering = -input.x * MAX_STEERING_ANGLE;
        // let steering = wheel
        //     .steering
        //     .lerp(target_steering, time.delta_seconds() * 10.0);
        // let steering = -input.x * MAX_STEERING_ANGLE;

        wheel.steering = (MAX_STEERING_ANGLE * turn_modifier) * -*steering_wheel;
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

        let angle = forward.angle_between(vel.linvel).to_degrees();
        if angle > 90.0 || angle < -90.0 {
            // Going backwards, do not apply down force
        } else {
            let down_force = projected.length() * time.delta_seconds();
            let down_force = down_force.clamp(0.0, 0.4);
            dbg!(down_force);
            impulse.impulse += Vec3::new(0.0, down_force * -100.0, 0.0);
        }
    }
}
