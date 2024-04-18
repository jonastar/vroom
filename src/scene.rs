use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::GameState;

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
) {
    // plane
    commands.spawn((
        PbrBundle {
            // mesh: meshes.add(Plane3d::default().mesh().size(20., 20.)),
            mesh: meshes.add(Cuboid::new(100.0, 0.5, 100.0)),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3)),
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

fn despawn_scene() {}

pub fn camera_look_at<T: Component>(
    mut camera: Query<&mut Transform, (With<Camera>, Without<T>)>,
    target: Query<&Transform, With<T>>,
) {
    for mut camera in &mut camera {
        let target = target.single();
        camera.look_at(target.translation, Vec3::Y);
    }
}
