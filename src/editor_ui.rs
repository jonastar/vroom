use crate::map::Track;
use crate::map_file::{LoadMapFileHandleCommand, SaveMapCommand};
use crate::{ui_util::button_clicked_eq_cond, GameState};
use bevy::prelude::*;
use bevy::tasks::{futures_lite::future, AsyncComputeTaskPool, Task};
use rfd::{AsyncFileDialog, FileHandle};

pub struct EditorUiPlugin;

impl Plugin for EditorUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(GameState::Editor), spawn_editor_ui)
            .add_systems(
                Update,
                (
                    on_save_clicked.run_if(button_clicked_eq_cond(EditorButton::Save)),
                    on_load_clicked.run_if(button_clicked_eq_cond(EditorButton::Load)),
                    on_clear_clicked.run_if(button_clicked_eq_cond(EditorButton::Clear)),
                    poll_file_dialog,
                ),
            )
            .add_systems(OnExit(GameState::Editor), despawn_editor_ui);
    }
}

#[derive(Component, Clone)]
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
struct EditorUi;

#[derive(Component, Eq, PartialEq)]
enum EditorButton {
    Save,
    Load,
    Clear,
}

fn spawn_editor_ui(mut commands: Commands) {
    commands
        .spawn((
            NodeBundle {
                style: Style {
                    width: Val::Percent(100.0),
                    height: Val::Px(150.0),
                    align_self: AlignSelf::End,
                    // margin: UiRect::all(Val::Px(10.0)),
                    flex_direction: FlexDirection::Column,
                    flex_wrap: FlexWrap::Wrap,
                    align_items: AlignItems::Start,
                    justify_content: JustifyContent::FlexEnd,
                    ..default()
                },
                background_color: BackgroundColor(Color::rgba(0.1, 0.1, 0.1, 0.5)),
                ..default()
            },
            EditorUi,
        ))
        .with_children(|children| {
            let button_colors = ButtonColors::default();
            children
                .spawn((
                    ButtonBundle {
                        style: Style {
                            width: Val::Px(140.0),
                            height: Val::Px(50.0),
                            justify_content: JustifyContent::Center,
                            align_items: AlignItems::Center,
                            margin: UiRect::all(Val::Px(10.0)),
                            ..Default::default()
                        },
                        background_color: button_colors.normal.clone().into(),
                        ..Default::default()
                    },
                    button_colors.clone(),
                    EditorButton::Save,
                ))
                .with_children(|parent| {
                    parent.spawn(TextBundle::from_section(
                        "Save",
                        TextStyle {
                            font_size: 40.0,
                            color: Color::rgb(0.9, 0.9, 0.9),
                            ..default()
                        },
                    ));
                });

            children
                .spawn((
                    ButtonBundle {
                        style: Style {
                            width: Val::Px(140.0),
                            height: Val::Px(50.0),
                            justify_content: JustifyContent::Center,
                            align_items: AlignItems::Center,
                            margin: UiRect::all(Val::Px(10.0)),
                            ..Default::default()
                        },
                        background_color: button_colors.normal.clone().into(),
                        ..Default::default()
                    },
                    button_colors.clone(),
                    EditorButton::Load,
                ))
                .with_children(|parent| {
                    parent.spawn(TextBundle::from_section(
                        "Load",
                        TextStyle {
                            font_size: 40.0,
                            color: Color::rgb(0.9, 0.9, 0.9),
                            ..default()
                        },
                    ));
                });

            children
                .spawn((
                    ButtonBundle {
                        style: Style {
                            width: Val::Px(140.0),
                            height: Val::Px(50.0),
                            justify_content: JustifyContent::Center,
                            align_items: AlignItems::Center,
                            margin: UiRect::all(Val::Px(10.0)),
                            ..Default::default()
                        },
                        background_color: button_colors.normal.clone().into(),
                        ..Default::default()
                    },
                    button_colors.clone(),
                    EditorButton::Clear,
                ))
                .with_children(|parent| {
                    parent.spawn(TextBundle::from_section(
                        "Clear",
                        TextStyle {
                            font_size: 40.0,
                            color: Color::rgb(0.9, 0.9, 0.9),
                            ..default()
                        },
                    ));
                });
        });

    // commands
    //     .spawn((
    //         NodeBundle {
    //             style: Style {
    //                 flex_direction: FlexDirection::Row,
    //                 align_items: AlignItems::Center,
    //                 justify_content: JustifyContent::SpaceAround,
    //                 bottom: Val::Px(5.),
    //                 width: Val::Percent(100.),
    //                 position_type: PositionType::Absolute,
    //                 ..default()
    //             },
    //             ..default()
    //         },
    //         Menu,
    //     ))
}

#[derive(Component)]
struct ChangeState(GameState);

// fn click_play_button(
//     mut next_state: ResMut<NextState<GameState>>,
//     mut interaction_query: Query<
//         (
//             &Interaction,
//             &mut BackgroundColor,
//             &ButtonColors,
//             Option<&ChangeState>,
//         ),
//         (Changed<Interaction>, With<Button>),
//     >,
// ) {
//     for (interaction, mut color, button_colors, change_state, open_link) in &mut interaction_query {
//         match *interaction {
//             Interaction::Pressed => {
//                 if let Some(state) = change_state {
//                     next_state.set(state.0.clone());
//                 } else if let Some(link) = open_link {
//                     if let Err(error) = webbrowser::open(link.0) {
//                         warn!("Failed to open link {error:?}");
//                     }
//                 }
//             }
//             Interaction::Hovered => {
//                 *color = button_colors.hovered.into();
//             }
//             Interaction::None => {
//                 *color = button_colors.normal.into();
//             }
//         }
//     }
// }

fn despawn_editor_ui(mut commands: Commands, menu: Query<Entity, With<EditorUi>>) {
    for entity in menu.iter() {
        commands.entity(entity).despawn_recursive();
    }
}

fn on_save_clicked(mut commands: Commands) {
    let thread_pool = AsyncComputeTaskPool::get();
    info!("Saving...");
    let task = thread_pool.spawn(AsyncFileDialog::new().save_file());
    commands.insert_resource(FileDialogInstance {
        kind: FileDialogKind::Save,
        task,
    });
}

fn on_load_clicked(mut commands: Commands) {
    let thread_pool = AsyncComputeTaskPool::get();
    info!("Loading...");
    let task = thread_pool.spawn(AsyncFileDialog::new().pick_file());
    commands.insert_resource(FileDialogInstance {
        kind: FileDialogKind::Load,
        task,
    });
}

#[derive(Resource)]
struct FileDialogInstance {
    kind: FileDialogKind,
    task: Task<Option<FileHandle>>,
}

enum FileDialogKind {
    Load,
    Save,
}

fn poll_file_dialog(mut commands: Commands, dialog: Option<ResMut<FileDialogInstance>>) {
    let Some(mut dialog) = dialog else {
        return;
    };

    if let Some(result) = future::block_on(future::poll_once(&mut dialog.task)) {
        commands.remove_resource::<FileDialogInstance>();

        if let Some(handle) = result {
            info!("Got handle {}", handle.file_name());
            match dialog.kind {
                FileDialogKind::Load => commands.add(LoadMapFileHandleCommand { handle }),
                FileDialogKind::Save => commands.add(SaveMapCommand { handle }),
            }
        }
    }
}

fn on_clear_clicked(mut commands: Commands, tracks: Query<Entity, With<Track>>) {
    for track in &tracks {
        commands.entity(track).despawn_recursive();
    }
}
