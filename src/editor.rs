use bevy::{
    prelude::*,
    render::{
        camera,
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};
use bevy_editor_cam::{
    controller::component::{EditorCam, EnabledMotion, OrbitConstraint},
    input::default_camera_inputs,
};
use transform_gizmo_bevy::{GizmoCamera, GizmoTarget};

use crate::{loading::TextureAssets, GameState};

pub struct EditorPlugin;

impl Plugin for EditorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(GameState::Editor), spawn_editor)
            .add_systems(
                Update,
                disable_camera_movement_on_gizmo
                    .run_if(in_state(GameState::Editor))
                    .before(default_camera_inputs),
            )
            .add_systems(Update, (visualize_track_segment_splines, generate_track));
    }
}

#[derive(Component)]
struct EditorMarker;

fn spawn_editor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    textures: Res<TextureAssets>,
    mut scenes: ResMut<Assets<Scene>>,
) {
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(15.0, 5.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        GizmoCamera,
        EditorCam {
            orbit_constraint: OrbitConstraint::Fixed {
                up: Vec3::Y,
                can_pass_tdc: false,
            },
            last_anchor_depth: 2.0,
            ..Default::default()
        },
        // IsDefaultUiCamera,
    ));

    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_translation(Vec3 {
            x: -1.0,
            y: 1.0,
            z: 0.0,
        })
        .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb(0.8, 0.1, 0.3)),
            transform: Transform::from_translation(Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            ..default()
        },
        // GizmoTarget::default(),
    ));

    // Define your control points
    // These points will define the curve
    // You can learn more about bezier curves here
    // https://en.wikipedia.org/wiki/B%C3%A9zier_curve

    commands
        .spawn((TrackSegment, SpatialBundle::default()))
        .with_children(|builder| {
            builder
                .spawn(segment_bundle(
                    &mut meshes,
                    &mut materials,
                    0,
                    Transform::from_translation(Vec3::new(-6., 2., 0.)),
                ))
                .insert(GizmoTarget::default());

            builder.spawn(segment_bundle(
                &mut meshes,
                &mut materials,
                1,
                Transform::from_translation(Vec3::new(12., 8., 0.)),
            ));

            builder.spawn(segment_bundle(
                &mut meshes,
                &mut materials,
                2,
                Transform::from_translation(Vec3::new(-12., 8., 0.)),
            ));

            builder.spawn(segment_bundle(
                &mut meshes,
                &mut materials,
                3,
                Transform::from_translation(Vec3::new(6., 2., 0.)),
            ));
        });

    // let points = [[
    //     vec3(-6., 2., 0.),
    //     vec3(12., 8., 0.),
    //     vec3(-12., 8., 0.),
    //     vec3(6., 2., 0.),
    // ]];

    // // Make a CubicCurve
    // let bezier = CubicBezier::new(points).to_curve();
}

fn segment_bundle(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    index: i32,
    transform: Transform,
) -> impl Bundle {
    (
        PbrBundle {
            mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb(0.8, 0.1, 0.3)),
            transform,
            ..default()
        },
        TrackSegmentCurvePoint(index),
    )
}

fn disable_camera_movement_on_gizmo(
    gizmos: Query<&GizmoTarget>,
    mut cameras: Query<&mut EditorCam>,
) {
    let blocked = gizmos.iter().any(|v| v.is_active || v.is_focused);
    for mut camera in &mut cameras {
        if blocked {
            camera.enabled_motion = EnabledMotion {
                orbit: true,
                pan: false,
                zoom: true,
            }
        } else {
            camera.enabled_motion = EnabledMotion {
                orbit: true,
                pan: true,
                zoom: true,
            }
        }
    }
}

#[derive(Component)]
struct TrackSegment;

#[derive(Component, PartialEq, PartialOrd, Ord, Eq)]
struct TrackSegmentCurvePoint(i32);

#[derive(Component)]
struct TrackSegmentBox;

fn visualize_track_segment_splines(
    tracks: Query<&Children, With<TrackSegment>>,
    transforms: Query<(&TrackSegmentCurvePoint, &Transform)>,
    mut gizmos: Gizmos,
) {
    for segment_children in &tracks {
        let mut points = Vec::with_capacity(4);
        for child in segment_children {
            let Ok((point, transform)) = transforms.get(*child) else {
                continue;
            };

            points.push((point.0, transform.translation));
        }

        points.sort_by(|(index_a, _), (index_b, _)| index_a.cmp(index_b));

        let curve = CubicCardinalSpline::new(
            5.0,
            points
                .into_iter()
                .map(|(_, point)| point)
                .collect::<Vec<_>>(),
        )
        .to_curve();
        gizmos.linestrip(curve.iter_positions(50), Color::WHITE);
    }
}

fn generate_track(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    changed_tracks: Query<&Parent, (Changed<Transform>, With<TrackSegmentCurvePoint>)>,
    tracks: Query<&Children, With<TrackSegment>>,
    curve_points: Query<(&TrackSegmentCurvePoint, &Transform)>,
    boxes: Query<Entity, With<TrackSegmentBox>>,

    mut gizmos: Gizmos,
) {
    let mut updated_tracks = Vec::new();
    for parent in &changed_tracks {
        if updated_tracks.contains(&parent.get()) {
            continue;
        }

        let Ok(children) = tracks.get(parent.get()) else {
            continue;
        };

        updated_tracks.push(parent.get());

        let mut points = Vec::with_capacity(4);
        for child in children {
            let Ok((point, transform)) = curve_points.get(*child) else {
                if boxes.contains(*child) {
                    commands.entity(*child).despawn_recursive();
                }
                continue;
            };

            points.push((point.0, transform.translation));
        }

        points.sort_by(|(index_a, _), (index_b, _)| index_a.cmp(index_b));

        let curve = CubicCardinalSpline::new(
            5.0,
            points
                .into_iter()
                .map(|(_, point)| point)
                .collect::<Vec<_>>(),
        )
        .to_curve();

        let mut iter = curve.iter_positions(50);
        let mut previous = iter.next().unwrap();
        let mut previous_connection_points_world = default_connection_points(previous);

        commands.entity(parent.get()).with_children(|builder| {
            for position in iter {
                let center = ((position - previous) / 2.0) + previous;
                // let rot =
                let length = position.distance(previous);
                let spawn_transform =
                    Transform::from_translation(center).looking_at(previous, Vec3::Y);

                let inversed = spawn_transform.compute_affine().inverse();
                let cur_connection_points_local = [
                    inversed.transform_point(previous_connection_points_world[0]),
                    inversed.transform_point(previous_connection_points_world[1]),
                    inversed.transform_point(previous_connection_points_world[2]),
                    inversed.transform_point(previous_connection_points_world[3]),
                ];
                gizmos.line(
                    spawn_transform.translation,
                    spawn_transform.transform_point(cur_connection_points_local[0]),
                    Color::RED,
                );
                gizmos.line(
                    spawn_transform.translation,
                    spawn_transform.transform_point(cur_connection_points_local[1]),
                    Color::GREEN,
                );
                gizmos.line(
                    spawn_transform.translation,
                    spawn_transform.transform_point(cur_connection_points_local[2]),
                    Color::BLUE,
                );
                gizmos.line(
                    spawn_transform.translation,
                    spawn_transform.transform_point(cur_connection_points_local[3]),
                    Color::YELLOW,
                );
                // let cur_connection_points_local =
                //     global.transform_point(previous_connection_points_world);

                let (mesh, next_connection_points_local) =
                    generate_segment_mesh(length, cur_connection_points_local);

                previous_connection_points_world[0] =
                    spawn_transform.transform_point(next_connection_points_local[0]);
                previous_connection_points_world[1] =
                    spawn_transform.transform_point(next_connection_points_local[1]);
                previous_connection_points_world[2] =
                    spawn_transform.transform_point(next_connection_points_local[2]);
                previous_connection_points_world[3] =
                    spawn_transform.transform_point(next_connection_points_local[3]);

                builder.spawn((
                    PbrBundle {
                        // mesh: meshes.add(Cuboid::new(1.0, 1.0, length)),
                        mesh: meshes.add(mesh),
                        material: materials.add(Color::rgb(0.0, 0.5, 0.3)),
                        transform: spawn_transform,
                        ..default()
                    },
                    TrackSegmentBox,
                ));

                previous = position;
            }
        });
    }

    // for segment_children in &tracks {
    //     let mut points = Vec::with_capacity(4);
    //     for child in segment_children {
    //         let Ok((point, transform)) = transforms.get(*child) else {
    //             continue;
    //         };

    //         points.push((point.0, transform.translation));
    //     }

    //     points.sort_by(|(index_a, _), (index_b, _)| index_a.cmp(index_b));

    //     let curve = CubicCardinalSpline::new(
    //         5.0,
    //         points
    //             .into_iter()
    //             .map(|(_, point)| point)
    //             .collect::<Vec<_>>(),
    //     )
    //     .to_curve();
    //     gizmos.linestrip(curve.iter_positions(50), Color::WHITE);
    // }
}

const TRACK_WIDTH: f32 = 4.0;
const HALF_TRACK_WIDTH: f32 = TRACK_WIDTH / 2.0;
const TRACK_HEIGHT: f32 = 1.0;
const HALF_TRACK_HEIGHT: f32 = TRACK_HEIGHT / 2.0;

fn default_connection_points(position: Vec3) -> [Vec3; 4] {
    let next_connection_points = [
        Vec3::new(-HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, 0.0) + position,
        Vec3::new(HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, 0.0) + position,
        Vec3::new(HALF_TRACK_WIDTH, -HALF_TRACK_HEIGHT, 0.0) + position,
        Vec3::new(-HALF_TRACK_WIDTH, -HALF_TRACK_HEIGHT, 0.0) + position,
    ];
    // let transform = Transform::IDENTITY;

    // transform.transform_point(point)
    next_connection_points
}

fn generate_segment_mesh(length: f32, connection_points: [Vec3; 4]) -> (Mesh, [Vec3; 4]) {
    let half_size = Vec3::new(HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, length / 2.0);
    let min = -half_size;
    let max = half_size;

    // let next_connection_points = [
    //     Vec3::new(min.x, max.y, min.z),
    //     Vec3::new(max.x, max.y, min.z),
    //     Vec3::new(max.x, min.y, min.z),
    //     Vec3::new(min.x, min.y, min.z),
    // ];
    let next_connection_points = [
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, max.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(min.x, min.y, max.z),
    ];

    // Suppose Y-up right hand, and camera look from +Z to -Z
    let vertices = &[
        // Front
        ([min.x, min.y, max.z], [0.0, 0.0, 1.0], [0.0, 0.0]),
        ([max.x, min.y, max.z], [0.0, 0.0, 1.0], [1.0, 0.0]),
        ([max.x, max.y, max.z], [0.0, 0.0, 1.0], [1.0, 1.0]),
        ([min.x, max.y, max.z], [0.0, 0.0, 1.0], [0.0, 1.0]),
        //
        // (connection_points[3].to_array(), [0.0, 0.0, 1.0], [0.0, 0.0]),
        // (connection_points[2].to_array(), [0.0, 0.0, 1.0], [1.0, 0.0]),
        // (connection_points[1].to_array(), [0.0, 0.0, 1.0], [1.0, 1.0]),
        // (connection_points[0].to_array(), [0.0, 0.0, 1.0], [0.0, 1.0]),
        // Back
        // ([min.x, max.y, min.z], [0.0, 0.0, -1.0], [1.0, 0.0]),
        // ([max.x, max.y, min.z], [0.0, 0.0, -1.0], [0.0, 0.0]),
        // ([max.x, min.y, min.z], [0.0, 0.0, -1.0], [0.0, 1.0]),
        // ([min.x, min.y, min.z], [0.0, 0.0, -1.0], [1.0, 1.0]),
        (
            connection_points[0].to_array(),
            [0.0, 0.0, -1.0],
            [1.0, 0.0],
        ),
        (
            connection_points[1].to_array(),
            [0.0, 0.0, -1.0],
            [0.0, 0.0],
        ),
        (
            connection_points[2].to_array(),
            [0.0, 0.0, -1.0],
            [0.0, 1.0],
        ),
        (
            connection_points[3].to_array(),
            [0.0, 0.0, -1.0],
            [1.0, 1.0],
        ),
        // Right
        (connection_points[2].to_array(), [1.0, 0.0, 0.0], [0.0, 0.0]),
        (connection_points[1].to_array(), [1.0, 0.0, 0.0], [1.0, 0.0]),
        ([max.x, max.y, max.z], [1.0, 0.0, 0.0], [1.0, 1.0]),
        ([max.x, min.y, max.z], [1.0, 0.0, 0.0], [0.0, 1.0]),
        // Left
        ([min.x, min.y, max.z], [-1.0, 0.0, 0.0], [1.0, 0.0]),
        ([min.x, max.y, max.z], [-1.0, 0.0, 0.0], [0.0, 0.0]),
        (
            connection_points[0].to_array(),
            [-1.0, 0.0, 0.0],
            [0.0, 1.0],
        ),
        (
            connection_points[3].to_array(),
            [-1.0, 0.0, 0.0],
            [1.0, 1.0],
        ),
        // Top
        (connection_points[1].to_array(), [0.0, 1.0, 0.0], [1.0, 0.0]),
        (connection_points[0].to_array(), [0.0, 1.0, 0.0], [0.0, 0.0]),
        ([min.x, max.y, max.z], [0.0, 1.0, 0.0], [0.0, 1.0]),
        ([max.x, max.y, max.z], [0.0, 1.0, 0.0], [1.0, 1.0]),
        // Bottom
        ([max.x, min.y, max.z], [0.0, -1.0, 0.0], [0.0, 0.0]),
        ([min.x, min.y, max.z], [0.0, -1.0, 0.0], [1.0, 0.0]),
        (
            connection_points[3].to_array(),
            [0.0, -1.0, 0.0],
            [1.0, 1.0],
        ),
        (
            connection_points[2].to_array(),
            [0.0, -1.0, 0.0],
            [0.0, 1.0],
        ),
    ];

    let positions: Vec<_> = vertices.iter().map(|(p, _, _)| *p).collect();
    let normals: Vec<_> = vertices.iter().map(|(_, n, _)| *n).collect();
    let uvs: Vec<_> = vertices.iter().map(|(_, _, uv)| *uv).collect();

    let indices = Indices::U32(vec![
        0, 1, 2, 2, 3, 0, // front
        4, 5, 6, 6, 7, 4, // back
        8, 9, 10, 10, 11, 8, // right
        12, 13, 14, 14, 15, 12, // left
        16, 17, 18, 18, 19, 16, // top
        20, 21, 22, 22, 23, 20, // bottom
    ]);

    (
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
        .with_inserted_indices(indices),
        next_connection_points,
    )
}
