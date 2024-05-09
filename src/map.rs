use bevy::{math::cubic_splines::CubicCurve, pbr::wireframe::Wireframe, prelude::*};
use bevy_rapier3d::geometry::{Collider, ComputedColliderShape, VHACDParameters};

use crate::{
    editor::StartPoint,
    map_file::{LoadMap, MapFile, PrepareSaveMap, SavedTrack},
    track_mesh::{generate_segment_mesh_new, MeshGeneratorBuffers, RotatedSeam},
};

pub struct MapPlugin;

impl Plugin for MapPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(RotatedSeam(false))
            .add_systems(LoadMap, (load_tracks, load_start_point))
            .add_systems(PrepareSaveMap, (save_tracks, save_start_point))
            .add_systems(Update, (generate_tracks, visualize_track_segment_splines));
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

fn load_start_point(
    mut commands: Commands,
    file: Res<MapFile>,
    mut start_points: Query<&mut Transform, With<StartPoint>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let Ok(mut start_transform) = start_points.get_single_mut() else {
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
                material: materials.add(Color::rgb(0.2, 0.3, 0.1)),
                transform: Transform::from_translation(file.start_point)
                    .with_rotation(file.start_rot),
                ..default()
            },
            StartPoint,
        ));
        return;
    };

    start_transform.translation = file.start_point;
    start_transform.rotation = file.start_rot;
}

fn save_tracks(mut file: ResMut<MapFile>, tracks: Query<&Track>) {
    for track in &tracks {
        file.tracks.push(track.into());
    }
}

fn save_start_point(mut file: ResMut<MapFile>, start_points: Query<&Transform, With<StartPoint>>) {
    for t in &start_points {
        file.start_point = t.translation;
        file.start_rot = t.rotation;
        // file.tracks.push(track.into());
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

    let mut mesh_gen_buf = MeshGeneratorBuffers::default();

    commands.entity(entity).with_children(|builder| {
        for segment_index in 0..track.bezier_segments.len() {
            let subdivisions = 40;

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

                let generate_back = segment_index == track.bezier_segments.len() - 1
                    && sub_index == subdivisions - 1;

                let (mesh, next_connection_points_local) = generate_segment_mesh_new(
                    &mut mesh_gen_buf,
                    length,
                    cur_connection_points_local,
                    generate_back,
                    segment_index == 0 && sub_index == 1,
                );
                mesh_gen_buf.clear();
                // let (mesh, collider, next_connection_points_local) =
                //     generate_segment_mesh(length, cur_connection_points_local);

                // let collider = Collider::from_bevy_mesh(
                //     &mesh,
                //     &ComputedColliderShape::ConvexDecomposition(VHACDParameters {
                //         ..Default::default()
                //     }),
                // );

                let collider = Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::TriMesh);

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
                    Wireframe,
                ));

                // another_builder.insert(collider);
                if let Some(collider) = collider {
                    another_builder.insert(collider);
                }

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
