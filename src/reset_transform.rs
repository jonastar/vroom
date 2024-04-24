use bevy::prelude::*;
use bevy_rapier3d::dynamics::Velocity;

use crate::{actions::Actions, GameState};

pub struct ResetPlugin;

impl Plugin for ResetPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (new_resettables, reset).run_if(in_state(GameState::Playing)),
        );
    }
}

#[derive(Component)]
pub struct Resetable;

#[derive(Component)]
struct ResetTracker {
    original_transform: Transform,
}

fn new_resettables(
    mut commands: Commands,
    query: Query<(Entity, &Transform), (Added<Resetable>, Without<ResetTracker>)>,
) {
    for (entity, transform) in &query {
        commands.entity(entity).insert(ResetTracker {
            original_transform: *transform,
        });
    }
}

fn reset(
    actions: Res<Actions>,

    mut resettables: Query<(&mut Transform, &ResetTracker, Option<&mut Velocity>)>,
) {
    if actions.reset {
        for (mut trans, tracker, maybe_vel) in &mut resettables {
            *trans = tracker.original_transform;

            if let Some(mut vel) = maybe_vel {
                *vel = Velocity::default();
            }
        }
    }
}
