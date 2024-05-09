use bevy::prelude::*;
use clap::{Parser, ValueEnum};

use crate::GameState;

pub struct SetupPlugin {
    pub args: Args,
}

impl SetupPlugin {
    pub fn load() -> Self {
        let conf = read_setup();
        Self { args: conf }
    }
}

impl Plugin for SetupPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.args.clone())
            .add_systems(OnEnter(GameState::Setup), setup_system);
    }
}

fn setup_system(mut commands: Commands, conf: Res<Args>, mut state: ResMut<NextState<GameState>>) {
    if let Some(map) = &conf.map {
        #[cfg(not(target_family = "wasm"))]
        load_file_native(&mut commands, map);

        #[cfg(target_family = "wasm")]
        panic!("this platform does not support the map option");
    }

    info!("Game is starting, start state: {:?}", conf.state);
    match conf.state {
        StartState::Menu => state.set(GameState::Menu),
        StartState::Playing => state.set(GameState::Playing),
        StartState::Editor => state.set(GameState::Editor),
        StartState::EditorPlaying => todo!(),
    }
}

#[cfg(not(target_family = "wasm"))]
fn load_file_native(commands: &mut Commands, path: &str) {
    use std::io::Read;

    use crate::map_file::LoadMapDataCommand;

    let mut file = std::fs::File::open(path).unwrap();
    let mut data = Vec::new();
    file.read_to_end(&mut data).unwrap();

    commands.add(LoadMapDataCommand { data });
}

/// Simple program to greet a person
#[derive(Parser, Debug, Default, Resource, Clone)]
#[command(version, about, long_about = None)]
struct Args {
    /// Map to load
    #[arg(short, long)]
    map: Option<String>,

    /// Instantly play map after loading
    #[arg(short, long, value_enum, default_value_t = StartState::Menu)]
    state: StartState,
}

#[derive(ValueEnum, Clone, Debug, Default)]
enum StartState {
    /// Doc comment
    // #[value(POSSIBLE VALUE ATTRIBUTE)]
    #[default]
    Menu,
    Playing,
    Editor,
    EditorPlaying,
}

fn read_setup() -> Args {
    #[cfg(target_family = "wasm")]
    return read_setup_wasm();

    #[cfg(not(target_family = "wasm"))]
    read_setup_native()
}

fn read_setup_native() -> Args {
    let args = Args::parse();
    args
}

fn read_setup_wasm() -> Args {
    Args::default()
}
