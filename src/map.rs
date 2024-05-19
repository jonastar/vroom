use bevy::{
    math::cubic_splines::CubicCurve,
    pbr::wireframe::Wireframe,
    prelude::*,
    render::mesh::{Indices, VertexAttributeValues},
};
use bevy_rapier3d::{
    geometry::{Collider, ComputedColliderShape, TriMeshFlags},
    parry::shape::SharedShape,
};

use crate::{
    editor::StartPoint,
    extract_vertices_from_mesh::extract_mesh_vertices_indices,
    map_file::{LoadMap, MapFile, PrepareSaveMap, SavedTrack},
    track_mesh::{generate_track_mesh, RotatedSeam},
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
                &mut meshes,
                &mut materials,
                &mut gizmos,
                entity,
                track,
                children,
            );

            track_gizmos(&mut gizmos, track);
        }
    } else {
        for (entity, track, children) in &changed_tracks {
            generate_track(
                &mut commands,
                &boxes,
                &mut meshes,
                &mut materials,
                &mut gizmos,
                entity,
                track,
                children,
            );

            track_gizmos(&mut gizmos, track);
        }
    }
}

fn generate_track(
    commands: &mut Commands,
    boxes: &Query<Entity, With<GeneratedTrackSegment>>,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,

    gizmos: &mut Gizmos,

    entity: Entity,
    track: &Track,
    children: &Children,
) {
    // Despawn existing graphics and colliders
    for child in children {
        if boxes.contains(*child) {
            commands.entity(*child).despawn_recursive();
        }
    }

    commands.entity(entity).with_children(|builder| {
        let mesh = generate_track_mesh(&track, gizmos);
        let Some(mesh) = mesh else {
            dbg!("No mesh");
            return;
        };

        let collider: Option<Collider> = extract_mesh_vertices_indices(&mesh).map(|(vtx, idx)| {
            SharedShape::trimesh_with_flags(
                vtx,
                idx,
                TriMeshFlags::MERGE_DUPLICATE_VERTICES | TriMeshFlags::FIX_INTERNAL_EDGES, // | TriMeshFlags::ORIENTED,
            )
            .into()
        });

        let mut another_builder = builder.spawn((
            PbrBundle {
                // mesh: meshes.add(Cuboid::new(1.0, 1.0, length)),
                mesh: meshes.add(mesh),
                material: materials.add(Color::rgb(0.0, 0.5, 0.3)),
                transform: Transform::default(),
                ..default()
            },
            GeneratedTrackSegment,
            Wireframe,
        ));

        if let Some(collider) = collider {
            another_builder.insert(collider);
        }
    });
}

fn track_gizmos(gizmos: &mut Gizmos, track: &Track) {
    // Despawn existing graphics and colliders

    let Some(curve) = &track.built_curve else {
        return;
    };

    if track.bezier_segments.is_empty() {
        return;
    }
    let start_position = curve.position(0.0);

    let mut previous_pos = start_position;

    for segment_index in 0..track.bezier_segments.len() {
        let subdivisions = 100;

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

            let interpolated_rot = start_rot.lerp(end_rot, local_t);
            gizmos.arrow(
                center,
                center + (interpolated_rot * Vec3::Y) * 10.0,
                Color::RED,
            );

            previous_pos = position;
        }
    }
}
