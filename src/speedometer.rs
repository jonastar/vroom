use bevy::prelude::*;
use bevy_rapier3d::dynamics::Velocity;

use crate::{car::CarBody, GameState};

pub struct SpeedometerPlugin;

impl Plugin for SpeedometerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(GameState::Playing), add_speedometer)
            .add_systems(
                Update,
                update_speedometer.run_if(in_state(GameState::Playing)),
            );
    }
}

#[derive(Component)]
struct Speedometer;

fn add_speedometer(mut commands: Commands) {
    commands.spawn((
        TextBundle::from_section(
            "",
            TextStyle {
                // font: asset_server.load("fonts/FiraMono-Medium.ttf"),
                font_size: 24.,
                color: Color::WHITE,
                ..Default::default()
            },
        ),
        Speedometer,
    ));
}

fn update_speedometer(
    cars: Query<&Velocity, With<CarBody>>,
    mut texts: Query<&mut Text, With<Speedometer>>,
) {
    let car = cars.single();
    for mut text in &mut texts {
        let speed = car.linvel.length();
        let kmh = (speed * 60.0 * 60.0) / 1000.0;
        text.sections[0].value = format!("{kmh} ({speed})");
    }
}
