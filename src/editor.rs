use bevy::{math::cubic_splines::CubicCurve, prelude::*};
use bevy_editor_cam::{
    controller::component::{EditorCam, EnabledMotion, OrbitConstraint},
    input::default_camera_inputs,
};
use bevy_mod_picking::{
    events::{Click, Pointer},
    prelude::{ListenerInput, On},
    PickableBundle,
};
use transform_gizmo_bevy::{GizmoCamera, GizmoTarget};

#[derive(States, Default, Clone, Eq, PartialEq, Debug, Hash)]
enum EditorState {
    #[default]
    Editor,
    Testing,
}

use crate::{
    actions::Actions,
    camera_controller::camera_look_at,
    car::{respawn_car_on_reset_action, spawn_car_helper, CarBody},
    map::{points_to_bezier_segments, Track},
    GameState,
};

pub struct EditorPlugin;

impl Plugin for EditorPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<EditorState>()
            .add_systems(OnEnter(GameState::Editor), spawn_editor)
            .add_systems(OnEnter(EditorState::Testing), try_map)
            .add_systems(OnExit(EditorState::Testing), stop_try_map)
            .add_systems(
                Update,
                disable_camera_movement_on_gizmo
                    .run_if(in_state(GameState::Editor))
                    .before(default_camera_inputs),
            )
            .add_systems(
                Update,
                respawn_car_on_reset_action.run_if(in_state(EditorState::Testing)),
            )
            .add_systems(Update, (spawn_segment_handles, build_segment_curves))
            .add_systems(
                Update,
                (
                    check_try_map,
                    check_exit_try_map.run_if(in_state(EditorState::Testing)),
                    update_extends_segment_buttons,
                )
                    .after(build_segment_curves),
            )
            .add_systems(
                PostUpdate,
                (camera_look_at::<CarBody>).run_if(in_state(EditorState::Testing)),
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
        IsDefaultUiCamera,
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
            transform: Transform::from_translation(Vec3::new(0.0, 2.0, 10.0)),
            ..default()
        },
        PickableBundle::default(),
        On::<Pointer<Click>>::run(clicked_segment_handle),
        StartPoint,
    ));

    commands.spawn((
        Track {
            built_curve: None,
            bezier_segments: Vec::new(),
            points: vec![
                (Vec3::new(0.0, 0.0, -30.0), Quat::IDENTITY),
                (Vec3::new(0.0, 1.0, -20.0), Quat::IDENTITY),
                (Vec3::new(0.0, 1.0, 30.0), Quat::IDENTITY),
                (Vec3::new(0.0, 0.0, 40.0), Quat::IDENTITY),
            ],
        },
        SpatialBundle::default(),
    ));
}

fn spawn_segment_handles(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    tracks: Query<(Entity, &Track), Added<Track>>,
) {
    for (entity, added_track) in &tracks {
        commands.entity(entity).with_children(|builder| {
            for (i, point) in added_track.points.iter().enumerate() {
                builder.spawn(segment_bundle(
                    &mut meshes,
                    &mut materials,
                    i as i32,
                    Transform::from_translation(point.0).with_rotation(point.1),
                ));
            }
        });
    }
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

#[derive(Component, PartialEq, PartialOrd, Ord, Eq)]
pub struct TrackSegmentCurvePoint(i32);

#[derive(Component)]
pub struct StartPoint;

fn build_segment_curves(
    changed_tracks: Query<&Parent, (Changed<Transform>, With<TrackSegmentCurvePoint>)>,
    mut tracks: Query<(&mut Track, &Children)>,
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

        let raw_points = points
            .into_iter()
            .map(|(_, point, rot)| (point, rot))
            .collect::<Vec<_>>();

        let points = points_to_bezier_segments(&raw_points);
        let points_only: Vec<_> = points.iter().map(|v| v.map(|iv| iv.0)).collect();
        let curve: CubicCurve<Vec3> = CubicBezier::new(points_only).to_curve();
        track.built_curve = Some(curve);
        track.bezier_segments = points;
        track.points = raw_points;
    }
}

fn check_try_map(actions: Res<Actions>, mut next_state: ResMut<NextState<EditorState>>) {
    if !actions.try_map {
        return;
    }

    info!("Setting to testing");
    next_state.set(EditorState::Testing);
}

fn check_exit_try_map(actions: Res<Actions>, mut next_state: ResMut<NextState<EditorState>>) {
    if !actions.escape {
        return;
    }

    info!("Setting to editor");
    next_state.set(EditorState::Editor);
}

fn try_map(
    mut commands: Commands,
    query: Query<&Transform, With<StartPoint>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    info!("Spawned car");
    let start = query.iter().next().unwrap();
    spawn_car_helper(
        start.translation,
        start.rotation,
        &mut commands,
        &mut meshes,
        &mut materials,
    );
}

fn stop_try_map(mut commands: Commands, existing_cars: Query<Entity, With<CarBody>>) {
    for existing in &existing_cars {
        commands.entity(existing).despawn_recursive();
    }
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

    segments: Query<(Entity, &Children), Changed<Track>>,
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
    segments: Query<(Entity, &Children), Changed<Track>>,
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
