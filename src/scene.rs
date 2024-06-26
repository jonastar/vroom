use bevy::{gltf::Gltf, prelude::*};
use bevy_rapier3d::prelude::*;
use transform_gizmo_bevy::GizmoCamera;

use crate::{car::respawn_car_on_reset_action, loading::TextureAssets, GameState};

pub struct ScenePlugin;

#[derive(Component)]
pub struct Level;

/// This plugin handles player related stuff like movement
/// Player logic is only active during the State `GameState::Playing`
impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(GameState::Playing), spawn_scene)
            .add_systems(OnExit(GameState::Playing), despawn_scene)
            .add_systems(
                Update,
                respawn_car_on_reset_action.run_if(in_state(GameState::Playing)),
            );
    }
}

#[derive(Component)]
struct Ground;

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    gltf_assets: ResMut<Assets<Gltf>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    textures: Res<TextureAssets>,
    mut scenes: ResMut<Assets<Scene>>,
) {
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(15.0, 5.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        IsDefaultUiCamera,
        GizmoCamera,
    ));

    let level = &gltf_assets.get(&textures.level).unwrap().scenes[0];
    let scene = scenes.get_mut(level).unwrap();
    let colliders = get_scene_colliders(&mut meshes, &mut scene.world);
    commands
        .spawn(SceneBundle {
            scene: level.clone(),
            transform: Transform::default().with_scale(Vec3 {
                x: 3.0,
                y: 3.0,
                z: 3.0,
            }),
            ..default()
        })
        .with_children(|parent| {
            for (collider, transform) in &colliders {
                parent.spawn((
                    collider.clone(),
                    TransformBundle::from_transform(*transform),
                    Friction::coefficient(2.0),
                ));
            }
        });

    // // light
    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_translation(Vec3 {
            x: -1.0,
            y: 1.0,
            z: 0.0,
        })
        .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    spawn_ramp(
        Vec3 {
            x: -220.0,
            y: -16.5,
            z: -220.0,
        },
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    // // camera
    // // commands.spawn(Camera3dBundle {
    // //     transform: Transform::from_xyz(15.0, 5.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    // //     ..default()
    // // });
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
            mesh: meshes.add(Cuboid::new(5.0, 5.0, 5.0)),
            material: materials.add(Color::rgb(0.8, 0.1, 0.3)),
            transform: Transform::from_translation(position)
                .with_rotation(Quat::from_rotation_x(30f32.to_radians())),
            ..default()
        },
        Collider::cuboid(2.5, 2.5, 2.5),
        Friction::coefficient(1.0),
        RigidBody::Fixed,
    ));
}

fn despawn_scene() {}

// fn load_ground() {

//     Collider::heightfield(heights, num_rows, num_cols, scale)

//     let collider = ColliderBuilder::heightfield(heights, ground_size)
//         .translation(vector![-7.0, 0.0, 0.0])
//         .friction(1.0);
//     colliders.insert(collider);
// }

/// Get all colliders from a scene.
///
/// It will search for all nodes with name starting with "collider" and will create a collider from the mesh.
///
/// NOTE: should be called only once per scene as it will remove the colliders meshes from it.
pub fn get_scene_colliders(
    meshes: &mut Assets<Mesh>,
    world: &mut World,
) -> Vec<(Collider, Transform)> {
    let mut result = Vec::new();

    let mut entities_to_despawn = Vec::new();
    let mut meshes_q = world.query::<(Entity, &Name, Option<&Children>)>();
    for (entity, entity_name, children) in meshes_q.iter(world) {
        match process_mesh_collider(entity_name, children, world, meshes) {
            None => {}
            Some(collider) => {
                let transform = *world.get::<Transform>(entity).unwrap();
                result.push((collider, transform));
                entities_to_despawn.push(entity);
            }
        }
    }

    result
}

pub(super) fn process_mesh_collider(
    node_name: &str,
    children: Option<&Children>,
    world: &World,
    meshes: &mut Assets<Mesh>,
) -> Option<Collider> {
    // if !node_name.starts_with(COLLIDER_MESH_NAME) {
    //     return None;
    // }

    let children = if let Some(children) = children {
        children
    } else {
        return None;
    };

    children.iter().find_map(|&child| {
        if let Some(mesh) = world.get::<Handle<Mesh>>(child) {
            let mesh = meshes.get(mesh).unwrap();

            Some(Collider::from_bevy_mesh(
                &mesh,
                &ComputedColliderShape::TriMesh,
            ))
            .flatten()
        } else {
            None
        }
    })
}
