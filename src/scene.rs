use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::{na::DMatrix, rapier::geometry::ColliderBuilder};

use crate::{loading::TextureAssets, GameState};

pub struct ScenePlugin;

#[derive(Component)]
pub struct Scene;

/// This plugin handles player related stuff like movement
/// Player logic is only active during the State `GameState::Playing`
impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(GameState::Playing), spawn_scene)
            .add_systems(OnExit(GameState::Playing), despawn_scene);
    }
}

#[derive(Component)]
struct Ground;

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    textures: Res<TextureAssets>,
) {
    // plane
    commands.spawn((
        PbrBundle {
            // mesh: meshes.add(Plane3d::default().mesh().size(20., 20.)),
            mesh: meshes.add(Cuboid::new(100.0, 0.5, 100.0)),
            material: materials.add(Color::rgb(0.5, 0.2, 0.1)),
            transform: Transform::from_translation(Vec3 {
                x: 0.0,
                y: -0.25,
                z: 0.0,
            }),
            ..default()
        },
        Ground,
        Collider::cuboid(50.0, 0.25, 50.0),
        RigidBody::Fixed,
        Friction {
            coefficient: 1.0,
            combine_rule: CoefficientCombineRule::Average,
        },
    ));

    // light
    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_translation(Vec3 {
            x: -1.0,
            y: 1.0,
            z: 0.0,
        })
        .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // camera
    // commands.spawn(Camera3dBundle {
    //     transform: Transform::from_xyz(15.0, 5.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    //     ..default()
    // });
    spawn_box(
        Vec3 {
            x: 10.0,
            y: 0.5,
            z: 10.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    spawn_box(
        Vec3 {
            x: 2.0,
            y: 0.5,
            z: 10.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    spawn_box(
        Vec3 {
            x: 10.0,
            y: 0.5,
            z: 3.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    spawn_box(
        Vec3 {
            x: 5.0,
            y: 0.5,
            z: 8.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    spawn_box(
        Vec3 {
            x: -5.0,
            y: 0.5,
            z: -8.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    spawn_box(
        Vec3 {
            x: -2.0,
            y: 0.5,
            z: -2.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    spawn_ramp(
        Vec3 {
            x: -3.0,
            y: 0.0,
            z: -3.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );
    spawn_ramp(
        Vec3 {
            x: -4.0,
            y: 0.0,
            z: -3.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );
    spawn_box(
        Vec3 {
            x: -3.0,
            y: 0.0,
            z: -4.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );
    spawn_box(
        Vec3 {
            x: -4.0,
            y: 0.0,
            z: -4.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );
}

fn spawn_box(
    position: Vec3,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb(0.8, 0.1, 0.3)),
            transform: Transform::from_translation(position),
            ..default()
        },
        Collider::cuboid(0.5, 0.5, 0.5),
        RigidBody::Fixed,
    ));
}

fn spawn_ramp(
    position: Vec3,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb(0.8, 0.1, 0.3)),
            transform: Transform::from_translation(position)
                .with_rotation(Quat::from_rotation_x(45f32.to_radians())),
            ..default()
        },
        Collider::cuboid(0.5, 0.5, 0.5),
        RigidBody::Fixed,
    ));
}

fn despawn_scene() {}

pub fn camera_look_at<T: Component>(
    time: Res<Time>,
    mut camera: Query<&mut Transform, (With<Camera>, Without<T>)>,
    target: Query<&Transform, With<T>>,
) {
    for mut camera in &mut camera {
        let target = target.single();

        let target_pos = (target.translation + (target.back() * 15.0)) + Vec3::new(0.0, 8.0, 1.0);
        let new_pos = camera
            .translation
            .lerp(target_pos, time.delta_seconds() * 5.0);
        camera.look_at(target.translation, Vec3::Y);
        camera.translation = new_pos
    }
}

// fn load_ground() {

//     Collider::heightfield(heights, num_rows, num_cols, scale)

//     let collider = ColliderBuilder::heightfield(heights, ground_size)
//         .translation(vector![-7.0, 0.0, 0.0])
//         .friction(1.0);
//     colliders.insert(collider);
// }
