use bevy::math::Vec3Swizzles;
use bevy::prelude::*;

use crate::actions::game_control::{get_movement, GameControl};
use crate::player::Player;
use crate::GameState;

mod game_control;

pub const FOLLOW_EPSILON: f32 = 5.;

pub struct ActionsPlugin;

// This plugin listens for keyboard input and converts the input into Actions
// Actions can then be used as a resource in other systems to act on the player input.
impl Plugin for ActionsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Actions>()
            .add_systems(Update, set_movement_actions);
    }
}

#[derive(Default, Resource)]
pub struct Actions {
    pub player_movement: Option<Vec2>,
    pub breaking: bool,
    pub reset: bool,
    pub try_map: bool,
    pub toggle_seam: bool,
    pub escape: bool,

    pub camera_1: bool,
    pub camera_2: bool,
    pub camera_3: bool,
}

pub fn set_movement_actions(
    mut actions: ResMut<Actions>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    touch_input: Res<Touches>,
    player: Query<&Transform, With<Player>>,
    camera: Query<(&Camera, &GlobalTransform), With<Camera2d>>,
) {
    let mut player_movement = Vec2::new(
        get_movement(GameControl::Right, &keyboard_input)
            - get_movement(GameControl::Left, &keyboard_input),
        get_movement(GameControl::Up, &keyboard_input)
            - get_movement(GameControl::Down, &keyboard_input),
    );

    if let Some(touch_position) = touch_input.first_pressed_position() {
        let (camera, camera_transform) = camera.single();
        if let Some(touch_position) = camera.viewport_to_world_2d(camera_transform, touch_position)
        {
            let diff = touch_position - player.single().translation.xy();
            if diff.length() > FOLLOW_EPSILON {
                player_movement = diff.normalize();
            }
        }
    }

    if player_movement != Vec2::ZERO {
        actions.player_movement = Some(player_movement);
    } else {
        actions.player_movement = None;
    }

    actions.reset = keyboard_input.just_pressed(KeyCode::KeyR);
    actions.breaking = keyboard_input.pressed(KeyCode::Space);
    actions.try_map = keyboard_input.just_pressed(KeyCode::KeyK);
    actions.toggle_seam = keyboard_input.just_pressed(KeyCode::KeyT);
    actions.escape = keyboard_input.just_pressed(KeyCode::Escape);

    actions.camera_1 = keyboard_input.just_pressed(KeyCode::Digit1);
    actions.camera_2 = keyboard_input.just_pressed(KeyCode::Digit2);
    actions.camera_3 = keyboard_input.just_pressed(KeyCode::Digit3);
}
