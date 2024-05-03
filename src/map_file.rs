use bevy::{
    ecs::{schedule::ScheduleLabel, system::Command},
    prelude::*,
    tasks::{block_on, poll_once, IoTaskPool, Task},
};
use rfd::FileHandle;
use serde::{Deserialize, Serialize};

use crate::map::Track;

pub struct MapFilePlugin;

impl Plugin for MapFilePlugin {
    fn build(&self, app: &mut App) {
        app.init_schedule(PrepareSaveMap)
            .init_schedule(LoadMap)
            .add_systems(Update, (poll_saving_result, poll_load_task));
    }
}

#[derive(Serialize, Deserialize, Resource)]
pub struct MapFile {
    pub name: String,
    pub tracks: Vec<SavedTrack>,
    pub start_point: Vec3,
}

impl Default for MapFile {
    fn default() -> Self {
        Self {
            name: "unnamed".to_owned(),
            tracks: Default::default(),
            start_point: Vec3::ZERO,
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct SavedTrack {
    pub points: Vec<(Vec3, Quat)>,
}

impl From<&Track> for SavedTrack {
    fn from(value: &Track) -> Self {
        let cloned_points = value.points.clone();
        Self {
            points: cloned_points,
        }
    }
}

// This schedule is ran when saving a map, a "MapFile" resource is inserted which systems can add data to for saving
#[derive(ScheduleLabel, Debug, Clone, PartialEq, Eq, Hash)]

pub struct PrepareSaveMap;

// This schedule is ran when loading a map, a "MapFile" resource is inserted which systems can load entities and components from
#[derive(ScheduleLabel, Debug, Clone, PartialEq, Eq, Hash)]
pub struct LoadMap;

pub struct LoadMapFileCommand {
    pub handle: FileHandle,
}

#[derive(Component)]
struct LoadMapFileTask(Task<Vec<u8>>);

impl Command for LoadMapFileCommand {
    fn apply(self, world: &mut bevy::prelude::World) {
        let tasks = IoTaskPool::get();
        let task = tasks.spawn(async move { self.handle.read().await });
        world.spawn(LoadMapFileTask(task));
    }
}

fn poll_load_task(mut commands: Commands, mut query: Query<(Entity, &mut LoadMapFileTask)>) {
    for (entity, mut task) in &mut query {
        match block_on(poll_once(&mut task.0)) {
            Some(data) => {
                info!("Loading file data complete successfully!");
                commands.entity(entity).despawn_recursive();
                commands.add(LoadMapDataCommand { data });
            }
            None => {}
        }
    }
}

struct LoadMapDataCommand {
    pub data: Vec<u8>,
}

impl Command for LoadMapDataCommand {
    fn apply(self, world: &mut World) {
        let map: MapFile = match serde_json::from_slice(&self.data) {
            Ok(v) => v,
            Err(err) => {
                error!(%err, "failed deserializing map data");
                return;
            }
        };

        info!("Loading map {}", map.name);
        world.insert_resource(map);
        world.run_schedule(LoadMap);
        info!("Loading has been completed.");
    }
}

pub struct SaveMapCommand {
    pub handle: FileHandle,
}

#[derive(Component)]
pub struct SavingTask(Task<Result<(), std::io::Error>>);

impl Command for SaveMapCommand {
    fn apply(self, world: &mut bevy::prelude::World) {
        let map = MapFile::default();

        world.insert_resource(map);
        world.run_schedule(PrepareSaveMap);

        let map = world.resource::<MapFile>();
        let serialized = serde_json::to_vec(map).unwrap();

        let tasks = IoTaskPool::get();
        let task = tasks.spawn(async move { self.handle.write(&serialized).await });

        world.spawn(SavingTask(task));
    }
}

fn poll_saving_result(mut commands: Commands, mut query: Query<(Entity, &mut SavingTask)>) {
    for (entity, mut task) in &mut query {
        match block_on(poll_once(&mut task.0)) {
            Some(Ok(())) => {
                info!("Saving complete successfully!");
                commands.entity(entity).despawn_recursive();
            }
            Some(Err(err)) => {
                error!(%err, "Saving failed.");
                commands.entity(entity).despawn_recursive();
            }
            None => {}
        }
    }
}
