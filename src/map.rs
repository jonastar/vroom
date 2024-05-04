use bevy::{
    math::cubic_splines::CubicCurve,
    pbr::wireframe::Wireframe,
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};
use bevy_rapier3d::{
    geometry::{Collider, ComputedColliderShape, VHACDParameters},
    math::{Rot, Vect},
};

use crate::{
    actions::Actions,
    map_file::{LoadMap, MapFile, PrepareSaveMap, SavedTrack},
};

pub struct MapPlugin;

impl Plugin for MapPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(RotatedSeam(false))
            .add_systems(LoadMap, load_tracks)
            .add_systems(PrepareSaveMap, save_tracks)
            .add_systems(
                Update,
                (
                    generate_tracks,
                    visualize_track_segment_splines,
                    toggle_seam,
                ),
            );
    }
}

#[derive(Component)]
pub struct Track {
    pub built_curve: Option<CubicCurve<Vec3>>,
    pub points: Vec<(Vec3, Quat)>,
    pub bezier_segments: Vec<[(Vec3, Quat); 4]>,
}

impl From<&SavedTrack> for Track {
    fn from(saved: &SavedTrack) -> Self {
        let cloned_points = saved.points.clone();
        let bezier_segments = points_to_bezier_segments(&cloned_points);

        let points_only: Vec<_> = bezier_segments.iter().map(|v| v.map(|iv| iv.0)).collect();
        let curve: CubicCurve<Vec3> = CubicBezier::new(points_only).to_curve();

        Self {
            built_curve: Some(curve),
            points: cloned_points,
            bezier_segments,
        }
    }
}

fn load_tracks(mut commands: Commands, file: Res<MapFile>) {
    for saved_track in &file.tracks {
        let track = Track::from(saved_track);
        commands.spawn((SpatialBundle::default(), track));
        info!("Loading... {}", saved_track.points.len());
    }
}

fn save_tracks(mut file: ResMut<MapFile>, tracks: Query<&Track>) {
    for track in &tracks {
        file.tracks.push(track.into());
    }
}

pub fn points_to_bezier_segments(points: &Vec<(Vec3, Quat)>) -> Vec<[(Vec3, Quat); 4]> {
    let mut converted_points = Vec::with_capacity(points.len() / 2);

    let mut iter = points.into_iter();
    let Some(p0) = iter.next() else {
        return converted_points;
    };
    let Some(p1) = iter.next() else {
        return converted_points;
    };
    let Some(p2) = iter.next() else {
        return converted_points;
    };
    let Some(p3) = iter.next() else {
        return converted_points;
    };

    converted_points.push([*p0, *p1, *p2, *p3]);

    let mut prev_p2 = p2;
    let mut prev_p3 = p3;
    loop {
        let p0 = prev_p3;

        let p1_local_inverted = -(prev_p2.0 - p0.0);

        let p1 = p1_local_inverted + p0.0;

        let Some(p2) = iter.next() else {
            return converted_points;
        };
        let Some(p3) = iter.next() else {
            return converted_points;
        };

        prev_p2 = p2;
        prev_p3 = p3;

        converted_points.push([*p0, (p1, prev_p2.1), *p2, *p3]);
    }
}

fn visualize_track_segment_splines(tracks: Query<&Track>, mut gizmos: Gizmos) {
    for track in &tracks {
        for curve in &track.bezier_segments {
            gizmos.line(curve[0].0, curve[1].0, Color::WHITE);
            gizmos.line(curve[3].0, curve[2].0, Color::WHITE);
        }

        if let Some(curve) = &track.built_curve {
            gizmos.linestrip(curve.iter_positions(50), Color::WHITE);
        }
    }
}

#[derive(Resource, Debug)]
struct RotatedSeam(bool);

#[derive(Component)]
struct GeneratedTrackSegment;

fn generate_tracks(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    changed_tracks: Query<(Entity, &Track, &Children), Changed<Track>>,
    all_tracks: Query<(Entity, &Track, &Children)>,
    boxes: Query<Entity, With<GeneratedTrackSegment>>,
    mut gizmos: Gizmos,

    seam: Res<RotatedSeam>,
) {
    if seam.is_changed() {
        for (entity, track, children) in &all_tracks {
            generate_track(
                &mut commands,
                &boxes,
                &mut gizmos,
                &mut meshes,
                &mut materials,
                entity,
                track,
                children,
                seam.0,
            );
        }
    } else {
        for (entity, track, children) in &changed_tracks {
            generate_track(
                &mut commands,
                &boxes,
                &mut gizmos,
                &mut meshes,
                &mut materials,
                entity,
                track,
                children,
                seam.0,
            );
        }
    }
}

fn generate_track(
    commands: &mut Commands,
    boxes: &Query<Entity, With<GeneratedTrackSegment>>,
    gizmos: &mut Gizmos,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,

    entity: Entity,
    track: &Track,
    children: &Children,
    seam: bool,
) {
    // Despawn existing graphics and colliders
    for child in children {
        if boxes.contains(*child) {
            commands.entity(*child).despawn_recursive();
        }
    }

    let Some(curve) = &track.built_curve else {
        return;
    };

    if track.bezier_segments.is_empty() {
        return;
    }
    let start_position = curve.position(0.0);

    let mut previous_pos = start_position;
    let mut previous_connection_points_world = default_connection_points(previous_pos);

    commands.entity(entity).with_children(|builder| {
        for segment_index in 0..track.bezier_segments.len() {
            let subdivisions = 20;

            let start_rot = track.bezier_segments[segment_index][0].1;
            let end_rot = track.bezier_segments[segment_index][3].1;

            for sub_index in 0..subdivisions {
                if segment_index == 0 && sub_index == 0 {
                    continue;
                }

                let local_t = sub_index as f32 / subdivisions as f32;
                let global_t = segment_index as f32 + local_t;
                let position = curve.position(global_t);

                let center = ((position - previous_pos) / 2.0) + previous_pos;

                let length = position.distance(previous_pos);
                let interpolated_rot = start_rot.lerp(end_rot, local_t);
                gizmos.arrow(
                    center,
                    center + (interpolated_rot * Vec3::Y) * 10.0,
                    Color::RED,
                );
                let spawn_transform = Transform::from_translation(center)
                    .looking_at(previous_pos, interpolated_rot * Vec3::Y);

                let inverted = spawn_transform.compute_affine().inverse();
                let cur_connection_points_local = [
                    inverted.transform_point(previous_connection_points_world[0]),
                    inverted.transform_point(previous_connection_points_world[1]),
                    inverted.transform_point(previous_connection_points_world[2]),
                    inverted.transform_point(previous_connection_points_world[3]),
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

                let (mesh, collider, next_connection_points_local) =
                    generate_segment_mesh(length, cur_connection_points_local, seam);

                // let collider = Collider::from_bevy_mesh(
                //     &mesh,
                //     &ComputedColliderShape::ConvexDecomposition(VHACDParameters::default()),
                // );

                previous_connection_points_world[0] =
                    spawn_transform.transform_point(next_connection_points_local[0]);
                previous_connection_points_world[1] =
                    spawn_transform.transform_point(next_connection_points_local[1]);
                previous_connection_points_world[2] =
                    spawn_transform.transform_point(next_connection_points_local[2]);
                previous_connection_points_world[3] =
                    spawn_transform.transform_point(next_connection_points_local[3]);

                let mut another_builder = builder.spawn((
                    PbrBundle {
                        // mesh: meshes.add(Cuboid::new(1.0, 1.0, length)),
                        mesh: meshes.add(mesh),
                        material: materials.add(Color::rgb(0.0, 0.5, 0.3)),
                        transform: spawn_transform,
                        ..default()
                    },
                    GeneratedTrackSegment,
                    // Wireframe,
                ));

                another_builder.insert(collider);
                // if let Some(collider) = collider {
                //     another_builder.insert(collider);
                // }

                previous_pos = position;
            }
        }
    });
}

const TRACK_WIDTH: f32 = 10.0;
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

fn generate_segment_mesh(
    length: f32,
    connection_points: [Vec3; 4],
    rot: bool,
) -> (Mesh, Collider, [Vec3; 4]) {
    let half_size = Vec3::new(HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, length / 2.0);
    let min = -half_size;
    let max = half_size;

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

    let indices = if rot {
        Indices::U32(vec![
            0, 1, 2, 2, 3, 0, // front
            4, 5, 6, 6, 7, 4, // back
            8, 9, 10, 10, 11, 8, // right
            12, 13, 14, 14, 15, 12, // left
            16, 17, 18, 18, 19, 16, // top
            // 17, 18, 19, 19, 16, 17, // top
            // 20, 21, 22, 22, 23, 20, // bottom
            21, 22, 23, 23, 20, 21, // bottom
        ])
    } else {
        Indices::U32(vec![
            0, 1, 2, 2, 3, 0, // front
            4, 5, 6, 6, 7, 4, // back
            8, 9, 10, 10, 11, 8, // right
            12, 13, 14, 14, 15, 12, // left
            // 16, 17, 18, 18, 19, 16, // top
            17, 18, 19, 19, 16, 17, // top
            // 20, 21, 22, 22, 23, 20, // bottom
            21, 22, 23, 23, 20, 21, // bottom
        ])
    };

    // let collider_a_indices = [
    //     16, 17, 18, // top

    // ];

    // Collider::convex_mesh

    let collider = Collider::compound(vec![
        (
            // Top
            Vect::ZERO,
            Rot::IDENTITY,
            if rot {
                Collider::triangle(
                    positions[16].into(),
                    positions[17].into(),
                    positions[18].into(),
                    // positions[17].into(),
                    // positions[18].into(),
                    // positions[19].into(),
                )
            } else {
                Collider::triangle(
                    // positions[16].into(),
                    // positions[17].into(),
                    // positions[18].into(),
                    positions[17].into(),
                    positions[18].into(),
                    positions[19].into(),
                )
            },
        ),
        (
            // Top
            Vect::ZERO,
            Rot::IDENTITY,
            if rot {
                Collider::triangle(
                    positions[18].into(),
                    positions[19].into(),
                    positions[16].into(),
                    // positions[19].into(),
                    // positions[16].into(),
                    // positions[17].into(),
                )
            } else {
                Collider::triangle(
                    // positions[18].into(),
                    // positions[19].into(),
                    // positions[16].into(),
                    positions[19].into(),
                    positions[16].into(),
                    positions[17].into(),
                )
            },
        ),
        // (
        //     // Left
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[12].into(),
        //         positions[13].into(),
        //         positions[14].into(),
        //     ),
        // ),
        // (
        //     // Left
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[14].into(),
        //         positions[15].into(),
        //         positions[12].into(),
        //     ),
        // ),
        // (
        //     // Back
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[4].into(),
        //         positions[5].into(),
        //         positions[6].into(),
        //     ),
        // ),
        // (
        //     // Back
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[6].into(),
        //         positions[7].into(),
        //         positions[4].into(),
        //     ),
        // ),
        // (
        //     // half-cross
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[12].into(),
        //         positions[13].into(),
        //         positions[17].into(),
        //     ),
        // ),
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
        collider,
        next_connection_points,
    )
}

fn toggle_seam(actions: Res<Actions>, mut seam: ResMut<RotatedSeam>) {
    if !actions.toggle_seam || !actions.is_changed() {
        return;
    }

    info!("Toggled seam {seam:?}");
    seam.0 = !seam.0;
}
