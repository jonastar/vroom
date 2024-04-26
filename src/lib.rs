#![allow(clippy::type_complexity)]

mod actions;
mod audio;
mod car;
mod loading;
mod menu;
mod player;
mod raycast_vehicle_controller;
mod reset_transform;
mod scene;
mod speedometer;

use crate::actions::ActionsPlugin;
use crate::audio::InternalAudioPlugin;
use crate::loading::LoadingPlugin;
use crate::menu::MenuPlugin;

use bevy::app::App;
#[cfg(debug_assertions)]
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_editor_pls::EditorPlugin;
use bevy_rapier3d::{
    plugin::{NoUserData, RapierConfiguration, RapierPhysicsPlugin, TimestepMode},
    render::RapierDebugRenderPlugin,
};
use car::CarPlugin;
use raycast_vehicle_controller::RaycastVehiclePlugin;
use reset_transform::ResetPlugin;
use scene::ScenePlugin;
use speedometer::SpeedometerPlugin;

// This example game uses States to separate logic
// See https://bevy-cheatbook.github.io/programming/states.html
// Or https://github.com/bevyengine/bevy/blob/main/examples/ecs/state.rs
#[derive(States, Default, Clone, Eq, PartialEq, Debug, Hash)]
enum GameState {
    // During the loading State the LoadingPlugin will load our assets
    #[default]
    Loading,
    // During this State the actual game logic is executed
    Playing,
    // Here the menu is drawn and waiting for player interaction
    Menu,
}

pub struct GamePlugin;
impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        let mut physics_config = RapierConfiguration::default();
        physics_config.timestep_mode = TimestepMode::Fixed {
            dt: 1.0 / 64.0,
            substeps: 1,
        };

        app.init_state::<GameState>()
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
                ResetPlugin,
                RaycastVehiclePlugin,
                // EditorPlugin::default(),
            ));

        #[cfg(debug_assertions)]
        {
            app.add_plugins((FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin::default()));
        }
    }
}
