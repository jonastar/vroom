#![allow(clippy::type_complexity)]

mod actions;
mod audio;
mod car;
mod editor;
mod editor_ui;
mod loading;
mod map;
mod map_file;
mod menu;
mod player;
mod raycast_vehicle_controller;
mod scene;
pub mod setup;
mod speedometer;
mod track_mesh;
mod ui_util;
mod walker;

use std::time::Duration;

use crate::actions::ActionsPlugin;
use crate::audio::InternalAudioPlugin;
use crate::loading::LoadingPlugin;
use crate::menu::MenuPlugin;

use bevy::app::App;
#[cfg(debug_assertions)]
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_editor_cam::DefaultEditorCamPlugins;
use bevy_editor_pls::EditorPlugin;
use bevy_egui::EguiPlugin;
use bevy_rapier3d::{
    plugin::{NoUserData, RapierConfiguration, RapierPhysicsPlugin, TimestepMode},
    render::RapierDebugRenderPlugin,
};
use car::CarPlugin;
use raycast_vehicle_controller::RaycastVehiclePlugin;
use scene::ScenePlugin;
use speedometer::SpeedometerPlugin;
use transform_gizmo_bevy::{EnumSet, GizmoMode, GizmoOptions, TransformGizmoPlugin};

// This example game uses States to separate logic
// See https://bevy-cheatbook.github.io/programming/states.html
// Or https://github.com/bevyengine/bevy/blob/main/examples/ecs/state.rs
#[derive(States, Default, Clone, Eq, PartialEq, Debug, Hash)]
enum GameState {
    // During the loading State the LoadingPlugin will load our assets
    #[default]
    Loading,

    // Handles starting up the game, this will either launch into menu and so on depending on launch options
    Setup,

    // During this State the actual game logic is executed
    Playing,

    // Here the menu is drawn and waiting for player interaction
    Menu,

    Editor,
}

pub struct GamePlugin;
impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        let mut physics_config = RapierConfiguration::new(1.0);
        physics_config.timestep_mode = TimestepMode::Fixed {
            dt: 1.0 / 100.0,
            substeps: 1,
        };

        let gizmo_options = GizmoOptions {
            gizmo_modes: EnumSet::from_iter(vec![GizmoMode::Rotate, GizmoMode::Translate]),
            ..Default::default()
        };

        app.init_state::<GameState>()
            .insert_resource(Time::<Fixed>::from_hz(100.0))
            .insert_resource(physics_config)
            .add_plugins((
                RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule(),
                RapierDebugRenderPlugin::default(),
                LoadingPlugin,
                MenuPlugin,
                ActionsPlugin,
                InternalAudioPlugin,
                // PlayerPlugin,
                ScenePlugin,
                CarPlugin,
                SpeedometerPlugin,
                RaycastVehiclePlugin,
                editor::EditorPlugin,
                editor_ui::EditorUiPlugin,
                // EguiPlugin,
                walker::SteppingPlugin::default()
                    .at(Val::Px(10.0), Val::Px(100.0))
                    .add_schedule(FixedUpdate),
            ))
            .add_plugins((map_file::MapFilePlugin, map::MapPlugin))
            .insert_resource(gizmo_options)
            .add_plugins((
                bevy_mod_picking::DefaultPickingPlugins,
                TransformGizmoPlugin,
                DefaultEditorCamPlugins,
                EditorPlugin::default(),
            ));

        #[cfg(debug_assertions)]
        {
            app.add_plugins((FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin::default()));
        }
    }
}
