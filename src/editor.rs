use bevy::{
    math::cubic_splines::CubicCurve,
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};
use bevy_editor_cam::{
    controller::component::{EditorCam, EnabledMotion, OrbitConstraint},
    input::default_camera_inputs,
};
use bevy_mod_picking::{
    events::{Click, Pointer},
    prelude::{ListenerInput, On},
    PickableBundle,
};
use bevy_rapier3d::geometry::{Collider, ComputedColliderShape};
use transform_gizmo_bevy::{GizmoCamera, GizmoTarget};

#[derive(States, Default, Clone, Eq, PartialEq, Debug, Hash)]
enum EditorState {
    #[default]
    Editor,
    Testing,
}

use crate::{
    actions::Actions,
    car::{spawn_car_helper, CarBody},
    GameState,
};

pub struct EditorPlugin;

impl Plugin for EditorPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<EditorState>()
            .add_systems(OnEnter(GameState::Editor), spawn_editor)
            .add_systems(OnEnter(EditorState::Testing), try_map)
            .add_systems(
                Update,
                disable_camera_movement_on_gizmo
                    .run_if(in_state(GameState::Editor))
                    .before(default_camera_inputs),
            )
            .add_systems(Update, reset.run_if(in_state(EditorState::Testing)))
            .add_systems(Update, build_segment_curves)
            .add_systems(
                Update,
                (
                    visualize_track_segment_splines,
                    generate_track,
                    check_try_map,
                    update_extends_segment_buttons,
                )
                    .after(build_segment_curves),
            );
    }
}

#[derive(Component)]
struct EditorMarker;

fn spawn_editor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
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

    // Define your control points
    // These points will define the curve
    // You can learn more about bezier curves here
    // https://en.wikipedia.org/wiki/B%C3%A9zier_curve

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb(0.2, 0.3, 0.1)),
            transform: Transform::IDENTITY,
            ..default()
        },
        PickableBundle::default(),
        On::<Pointer<Click>>::run(clicked_segment_handle),
        StartPoint,
    ));

    commands
        .spawn((
            TrackSegment {
                built_curve: None,
                rotations: None,
                bezier_segments: Vec::new(),
            },
            SpatialBundle::default(),
        ))
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

            builder.spawn(segment_bundle(
                &mut meshes,
                &mut materials,
                4,
                Transform::from_translation(Vec3::new(16., 2., 0.)),
            ));

            builder.spawn(segment_bundle(
                &mut meshes,
                &mut materials,
                5,
                Transform::from_translation(Vec3::new(16., 12., 0.)),
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
        PickableBundle::default(),
        On::<Pointer<Click>>::run(clicked_segment_handle),
    )
}

fn clicked_segment_handle(
    mut commands: Commands,
    input: Res<ListenerInput<Pointer<Click>>>,
    existing_gizmos: Query<Entity, With<GizmoTarget>>,
) {
    let mut already_enabled_gizmo_on_entity = false;
    for entity in &existing_gizmos {
        if entity == input.target {
            already_enabled_gizmo_on_entity = true;
            continue;
        }

        commands.entity(entity).remove::<GizmoTarget>();
    }

    if already_enabled_gizmo_on_entity {
        return;
    }

    commands.entity(input.target).insert(GizmoTarget::default());
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
struct TrackSegment {
    built_curve: Option<CubicCurve<Vec3>>,
    bezier_segments: Vec<[(Vec3, Quat); 4]>,
    rotations: Option<Quat>,
}

#[derive(Component, PartialEq, PartialOrd, Ord, Eq)]
struct TrackSegmentCurvePoint(i32);

#[derive(Component)]
struct TrackSegmentBox;

#[derive(Component)]
struct StartPoint;

fn build_segment_curves(
    changed_tracks: Query<&Parent, (Changed<Transform>, With<TrackSegmentCurvePoint>)>,
    mut tracks: Query<(&mut TrackSegment, &Children)>,
    curve_points: Query<(&TrackSegmentCurvePoint, &Transform)>,
) {
    let mut updated_tracks = Vec::new();
    for parent in &changed_tracks {
        if updated_tracks.contains(&parent.get()) {
            continue;
        }

        let Ok((mut track, children)) = tracks.get_mut(parent.get()) else {
            continue;
        };

        updated_tracks.push(parent.get());

        let mut points = Vec::with_capacity(4);
        for child in children {
            let Ok((point, transform)) = curve_points.get(*child) else {
                continue;
            };

            points.push((point.0, transform.translation, transform.rotation));
        }

        points.sort_by(|(index_a, _, _), (index_b, _, _)| index_a.cmp(index_b));

        let points = points_to_bezier(
            points
                .into_iter()
                .map(|(_, point, rot)| (point, rot))
                .collect::<Vec<_>>(),
        );
        let points_only: Vec<_> = points.iter().map(|v| v.map(|iv| iv.0)).collect();
        let curve: CubicCurve<Vec3> = CubicBezier::new(points_only).to_curve();
        track.built_curve = Some(curve);
        track.bezier_segments = points;
    }
}

fn visualize_track_segment_splines(
    tracks: Query<(&TrackSegment, &Children)>,
    mut gizmos: Gizmos,
    curve_points: Query<(&TrackSegmentCurvePoint, &Transform)>,
) {
    for (track, children) in &tracks {
        let mut points = Vec::with_capacity(4);
        for child in children {
            let Ok((point, transform)) = curve_points.get(*child) else {
                continue;
            };

            points.push((point.0, transform.translation, transform.rotation));
        }
        points.sort_by(|(index_a, _, _), (index_b, _, _)| index_a.cmp(index_b));
        let points = points_to_bezier(
            points
                .into_iter()
                .map(|(_, point, rot)| (point, rot))
                .collect::<Vec<_>>(),
        );

        for curve in points {
            gizmos.line(curve[0].0, curve[1].0, Color::WHITE);
            gizmos.line(curve[3].0, curve[2].0, Color::WHITE);
        }

        if let Some(curve) = &track.built_curve {
            gizmos.linestrip(curve.iter_positions(50), Color::WHITE);
        }
    }
}

fn generate_track(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    changed_tracks: Query<(Entity, &TrackSegment, &Children), Changed<TrackSegment>>,
    boxes: Query<Entity, With<TrackSegmentBox>>,
    mut gizmos: Gizmos,
) {
    for (entity, track, children) in &changed_tracks {
        // Despawn existing graphics and colliders
        for child in children {
            if boxes.contains(*child) {
                commands.entity(*child).despawn_recursive();
            }
        }

        let Some(curve) = &track.built_curve else {
            continue;
        };

        if track.bezier_segments.is_empty() {
            continue;
        }
        let start_position = curve.position(0.0);

        let mut previous_pos = start_position;
        let mut previous_connection_points_world = default_connection_points(previous_pos);

        commands.entity(entity).with_children(|builder| {
            for segment_index in 0..track.bezier_segments.len() {
                let subdivisions = 10;

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
                    let spawn_transform = Transform::from_translation(center)
                        .looking_at(previous_pos, interpolated_rot * Vec3::Y);

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

                    let (mesh, next_connection_points_local) =
                        generate_segment_mesh(length, cur_connection_points_local);

                    let collider =
                        Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull);

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
                        TrackSegmentBox,
                    ));

                    if let Some(collider) = collider {
                        another_builder.insert(collider);
                    }

                    previous_pos = position;
                }
            }
        });
    }
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

fn generate_segment_mesh(length: f32, connection_points: [Vec3; 4]) -> (Mesh, [Vec3; 4]) {
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

fn points_to_bezier(points: Vec<(Vec3, Quat)>) -> Vec<[(Vec3, Quat); 4]> {
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

    converted_points.push([p0, p1, p2, p3]);

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

        converted_points.push([p0, (p1, prev_p2.1), p2, p3]);
    }
}

fn check_try_map(actions: Res<Actions>, mut next_state: ResMut<NextState<EditorState>>) {
    if !actions.try_map {
        return;
    }

    info!("Setting to testing");
    next_state.set(EditorState::Testing);
}

fn try_map(
    mut commands: Commands,
    query: Query<&Transform, With<StartPoint>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    info!("Spawned car");
    let start = query.single().translation;
    spawn_car_helper(start, &mut commands, &mut meshes, &mut materials);
}
fn reset(
    actions: Res<Actions>,
    mut commands: Commands,
    query: Query<&Transform, With<StartPoint>>,
    existing_cars: Query<Entity, With<CarBody>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !actions.reset {
        return;
    }

    for existing in &existing_cars {
        commands.entity(existing).despawn_recursive();
    }

    let start = query.single().translation;
    spawn_car_helper(start, &mut commands, &mut meshes, &mut materials);
}

#[derive(Component)]
enum AddSegmentButton {
    Start,
    End,
}

fn update_extends_segment_buttons(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    segments: Query<(Entity, &Children), Changed<TrackSegment>>,
    curve_points: Query<(Entity, &TrackSegmentCurvePoint), With<TrackSegmentCurvePoint>>,
    add_buttons: Query<(Entity, &Parent), With<AddSegmentButton>>,
) {
    for (segment_entity, segment_children) in &segments {
        for (button_entity, button_parent) in &add_buttons {
            if button_parent.get() == segment_entity {
                commands.entity(button_entity).despawn_recursive()
            } else if curve_points.contains(button_parent.get()) {
                commands.entity(button_entity).despawn_recursive()
            }
        }

        if segment_children.is_empty() {
            // spawn a single button here
            commands.entity(segment_entity).with_children(|builder| {
                builder.spawn(add_segment_button_bundle(
                    &mut meshes,
                    &mut materials,
                    AddSegmentButton::Start,
                ));
            });

            continue;
        }

        let mut low: Option<(Entity, &TrackSegmentCurvePoint)> = None;
        let mut high: Option<(Entity, &TrackSegmentCurvePoint)> = None;

        // Find the start and end segments
        for child in segment_children {
            let Ok((entity, curve_point)) = curve_points.get(*child) else {
                continue;
            };

            if let Some(inner_low) = low {
                if inner_low.1 .0 > curve_point.0 {
                    low = Some((entity, curve_point));
                }
            } else {
                low = Some((entity, curve_point));
            }

            if let Some(inner_high) = high {
                if inner_high.1 .0 < curve_point.0 {
                    high = Some((entity, curve_point));
                }
            } else {
                high = Some((entity, curve_point));
            }
        }

        if let Some(low) = low {
            // Spawn point on start
            commands.entity(low.0).with_children(|builder| {
                builder.spawn(add_segment_button_bundle(
                    &mut meshes,
                    &mut materials,
                    AddSegmentButton::Start,
                ));
            });

            if let Some(high) = high {
                if low.0 != high.0 {
                    // Spawn point on end
                    commands.entity(high.0).with_children(|builder| {
                        builder.spawn(add_segment_button_bundle(
                            &mut meshes,
                            &mut materials,
                            AddSegmentButton::End,
                        ));
                    });
                }
            }
        }
    }
}

fn add_segment_button_bundle(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    kind: AddSegmentButton,
) -> impl Bundle {
    (
        PbrBundle {
            mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb(0.4, 0.1, 1.0)),
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, -2.0)),
            ..default()
        },
        PickableBundle::default(),
        kind,
        On::<Pointer<Click>>::run(clicked_add_segment_button),
    )
}

fn clicked_add_segment_button(
    mut commands: Commands,
    input: Res<ListenerInput<Pointer<Click>>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    add_buttons: Query<(&AddSegmentButton, &Parent)>,
    segments: Query<(Entity, &Children), Changed<TrackSegment>>,
    curve_points: Query<
        (Entity, &TrackSegmentCurvePoint, &Parent, &Transform),
        With<TrackSegmentCurvePoint>,
    >,
) {
    let Ok((add_button_type, button_parent)) = add_buttons.get(input.target) else {
        return;
    };

    // Find the relevant track segment
    let (entity, children, position) = if let Ok(res) = segments.get(button_parent.get()) {
        (res.0, res.1, Vec3::ZERO)
    } else {
        if let Ok(curve) = curve_points.get(button_parent.get()) {
            if let Ok(segment) = segments.get(curve.2.get()) {
                (segment.0, segment.1, curve.3.translation)
            } else {
                return;
            }
        } else {
            return;
        }
    };

    let (index_0, index_1) = match add_button_type {
        AddSegmentButton::Start => children
            .iter()
            .filter_map(|child| curve_points.get(*child).ok().map(|(_, p, _, _)| p.0))
            .min()
            .map(|v| (v - 1, v - 2))
            .unwrap_or((0, 1)),
        AddSegmentButton::End => children
            .iter()
            .filter_map(|child| curve_points.get(*child).ok().map(|(_, p, _, _)| p.0))
            .max()
            .map(|v| (v + 1, v + 2))
            .unwrap_or((0, 1)),
    };

    commands.entity(entity).with_children(|builder| {
        builder.spawn(segment_bundle(
            &mut meshes,
            &mut materials,
            index_0,
            Transform::from_translation(position + Vec3::new(2., 0.0, 0.)),
        ));

        builder.spawn(segment_bundle(
            &mut meshes,
            &mut materials,
            index_1,
            Transform::from_translation(position + Vec3::new(3., 2.0, 0.)),
        ));
    });
}
