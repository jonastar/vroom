use bevy::{prelude::*, utils::petgraph::matrix_graph::Zero};

use crate::{actions::Actions, GameState};

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
                (move_car, turn_front_wheels).run_if(in_state(GameState::Playing)),
            );
    }
}

#[derive(Component)]
struct FrontWheel {
    original_rot: Quat,
}

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
        ))
        .with_children(|car| {
            // Main body
            car.spawn(PbrBundle {
                mesh: meshes.add(Cuboid::new(1.0, 0.5, 2.0)),
                material: materials.add(Color::rgb(0.2, 0.1, 0.3)),
                // transform: Transform::from_translation(Vec),
                ..default()
            });

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
            let wheel_start_rot = Quat::from_rotation_z(1.5708);
            car.spawn((
                PbrBundle {
                    mesh: meshes.add(Cylinder::new(0.25, 0.15)),
                    material: materials.add(Color::rgb(0.2, 0.1, 0.6)),
                    transform: Transform::from_translation(Vec3 {
                        x: -0.5,
                        y: -0.25,
                        z: -1.0,
                    })
                    .with_rotation(wheel_start_rot),
                    ..default()
                },
                FrontWheel {
                    original_rot: wheel_start_rot,
                },
            ));

            car.spawn((
                PbrBundle {
                    mesh: meshes.add(Cylinder::new(0.25, 0.15)),
                    material: materials.add(Color::rgb(0.2, 0.1, 0.6)),
                    transform: Transform::from_translation(Vec3 {
                        x: 0.5,
                        y: -0.25,
                        z: -1.0,
                    })
                    .with_rotation(wheel_start_rot),
                    ..default()
                },
                FrontWheel {
                    original_rot: wheel_start_rot,
                },
            ));

            // Back wheels
            car.spawn(PbrBundle {
                mesh: meshes.add(Cylinder::new(0.25, 0.15)),
                material: materials.add(Color::rgb(0.5, 0.1, 0.6)),
                transform: Transform::from_translation(Vec3 {
                    x: -0.5,
                    y: -0.25,
                    z: 1.0,
                })
                .with_rotation(wheel_start_rot),
                ..default()
            });

            car.spawn(PbrBundle {
                mesh: meshes.add(Cylinder::new(0.25, 0.15)),
                material: materials.add(Color::rgb(0.5, 0.1, 0.6)),
                transform: Transform::from_translation(Vec3 {
                    x: 0.5,
                    y: -0.25,
                    z: 1.0,
                })
                .with_rotation(wheel_start_rot),
                ..default()
            });
        });
}

fn despawn_car() {}

fn move_car(
    time: Res<Time>,
    actions: Res<Actions>,
    mut player_query: Query<&mut Transform, With<Car>>,
) {
    let Some(input) = actions.player_movement else {
        return;
    };

    let move_speed = 10.;
    if !input.y.is_zero() {
        let rotate_speed = 7.;

        if !input.x.is_zero() {
            for mut player_transform in &mut player_query {
                let up = player_transform.up();

                player_transform.rotate_axis(
                    up.into(),
                    (-input.x * input.y) * rotate_speed * time.delta_seconds(),
                );
            }
        }

        for mut player_transform in &mut player_query {
            let dir = player_transform.forward();
            player_transform.translation += dir * input.y * move_speed * time.delta_seconds();
        }
    }
}

fn turn_front_wheels(
    time: Res<Time>,
    actions: Res<Actions>,
    mut wheel_query: Query<(&mut Transform, &FrontWheel)>,
) {
    let x_input = actions.player_movement.map(|v| v.x).unwrap_or_default();

    for (mut transform, wheel) in &mut wheel_query {
        let target = wheel.original_rot * Quat::from_rotation_x(-x_input);
        let new_rot = transform.rotation.lerp(target, time.delta_seconds() * 10.0);
        transform.rotation = new_rot;
    }
}
