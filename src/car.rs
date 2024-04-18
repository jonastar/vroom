use crate::{actions::Actions, scene::camera_look_at, GameState};
use bevy::{prelude::*, utils::petgraph::matrix_graph::Zero};
use bevy_rapier3d::{na::Vector, prelude::*};

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
                )
                    .run_if(in_state(GameState::Playing)),
            );
    }
}

#[derive(Component)]
struct FrontWheelMain {
    original_rot: Quat,
}

#[derive(Component)]
struct Wheel;

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
                y: 0.5,
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

fn spawn_wheel(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    car_parent: &mut ChildBuilder,
    body: Entity,
    position: Vec3,
    is_front_wheel: bool,
) {
    let wheel_start_rot = Quat::from_rotation_z(1.5708);

    // let a = Quat::from_rotation_x(1.0) * Vec3::X;
    let joint = SphericalJointBuilder::new()
        .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
        .local_anchor1(position);

    car_parent
        .spawn((
            SpatialBundle {
                transform: Transform::from_translation(position),
                // .with_rotation(wheel_start_rot),
                ..Default::default()
            },
            Friction::new(2000.0),
            RigidBody::Dynamic,
            Velocity::zero(),
            ExternalImpulse::default(),
            ImpulseJoint::new(body, joint),
            Wheel,
        ))
        .with_children(|wheel| {
            let mut builder = wheel.spawn((
                PbrBundle {
                    mesh: meshes.add(Cylinder::new(0.25, 0.15)),
                    material: materials.add(Color::rgb(0.2, 0.1, 0.6)),
                    transform: Transform::from_rotation(wheel_start_rot),
                    ..default()
                },
                Collider::round_cylinder(0.075, 0.15, 0.1),
            ));

            if is_front_wheel {
                builder.insert(FrontWheelMain {
                    original_rot: wheel_start_rot,
                });
            }
        });
}
fn despawn_car() {}

fn move_car(
    time: Res<Time>,
    actions: Res<Actions>,
    mut car_query: Query<(&mut ExternalImpulse, &Transform, &mut ImpulseJoint), With<Wheel>>,
) {
    let input = actions.player_movement.unwrap_or_default();
    // let Some(input) = actions.player_movement else {
    //     return;
    // };

    let move_speed = 20.0;
    // if !input.y.is_zero() {
    for (mut impulse, transform, mut joint) in &mut car_query {
        joint
            .data
            .set_motor_velocity(JointAxis::AngX, move_speed * -input.y, 0.0);
        // let dir = transform.left();
        // joint.data
        // impulse.torque_impulse += input.y * *dir * move_speed * time.delta_seconds();
    }
    // }
}

fn turn_front_wheels(
    time: Res<Time>,
    actions: Res<Actions>,
    mut wheel_query: Query<(&mut Transform, &FrontWheelMain, &Parent)>,
    mut joint_query: Query<&mut ImpulseJoint>,
    mut steering_wheel: Local<f32>,
) {
    let x_input = actions.player_movement.map(|v| v.x).unwrap_or_default();
    *steering_wheel = steering_wheel.lerp(x_input / 2.0, time.delta_seconds() * 10.0);

    for (mut transform, wheel, parent) in &mut wheel_query {
        let current_wheel_rot = wheel.original_rot * Quat::from_rotation_x(-*steering_wheel);
        transform.rotation = current_wheel_rot;

        let mut joint: Mut<'_, ImpulseJoint> = joint_query.get_mut(parent.get()).unwrap();
        let new_joint_axis = Quat::from_rotation_y(-*steering_wheel) * Vec3::X;

        joint.data.set_local_axis1(new_joint_axis);
        joint.data.set_local_axis2(new_joint_axis);
    }
}

// fn turn_front_wheels_joint(
//     time: Res<Time>,
//     actions: Res<Actions>,
//     mut wheel_query: Query<&mut ImpulseJoint, With<FrontWheelParent>>,
// ) {
//     let x_input = actions.player_movement.map(|v| v.x).unwrap_or_default();

//     for (mut joint) in &mut wheel_query {
//         let new_joint_axis = Quat::from_rotation_y(-x_input) * Vec3::X;
//         joint.data.set_local_axis1(new_joint_axis);
//         joint.data.set_local_axis2(new_joint_axis);
//     }
// }
