use bevy::{
    ecs::schedule::{SystemConfig, SystemConfigs},
    prelude::*,
};

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
