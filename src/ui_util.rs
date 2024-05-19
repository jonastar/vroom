use bevy::{
    ecs::schedule::{SystemConfig, SystemConfigs},
    prelude::*,
};

use crate::GameState;

pub struct UiUtilPlugin;

impl Plugin for UiUtilPlugin {
    fn build(&self, app: &mut App) {}
}

pub fn button_clicked_eq_cond<B: Eq + Component + 'static>(
    variant: B,
) -> impl Fn(Query<(&Interaction, &B), Changed<Interaction>>) -> bool {
    move |query: Query<(&Interaction, &B), Changed<Interaction>>| -> bool {
        for (interaction, v) in &query {
            match interaction {
                Interaction::Pressed => {
                    if v == &variant {
                        return true;
                    }
                }
                _ => {}
            }
        }

        false
    }
}

#[derive(Component)]
struct ButtonColors {
    normal: Color,
    hovered: Color,
}

impl Default for ButtonColors {
    fn default() -> Self {
        ButtonColors {
            normal: Color::rgb(0.15, 0.15, 0.15),
            hovered: Color::rgb(0.25, 0.25, 0.25),
        }
    }
}

#[derive(Component)]
struct ChangeState(GameState);

#[derive(Component)]
struct OpenLink(&'static str);

fn button_effects(
    mut next_state: ResMut<NextState<GameState>>,
    mut interaction_query: Query<
        (
            &Interaction,
            &mut BackgroundColor,
            &ButtonColors,
            Option<&ChangeState>,
            Option<&OpenLink>,
        ),
        (Changed<Interaction>, With<Button>),
    >,
) {
    for (interaction, mut color, button_colors, change_state, open_link) in &mut interaction_query {
        match *interaction {
            Interaction::Pressed => {
                if let Some(state) = change_state {
                    next_state.set(state.0.clone());
                } else if let Some(link) = open_link {
                    if let Err(error) = webbrowser::open(link.0) {
                        warn!("Failed to open link {error:?}");
                    }
                }
            }
            Interaction::Hovered => {
                *color = button_colors.hovered.into();
            }
            Interaction::None => {
                *color = button_colors.normal.into();
            }
        }
    }
}
