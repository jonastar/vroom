use bevy::prelude::*;

use crate::actions::Actions;

pub struct CameraControllerPlugin;

/// This plugin handles player related stuff like movement
/// Player logic is only active during the State `GameState::Playing`
impl Plugin for CameraControllerPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<CameraState>()
            .add_systems(Update, update_camera_state);
    }
}

#[derive(States, Default, Clone, Eq, PartialEq, Debug, Hash)]
pub enum CameraState {
    #[default]
    Close,
    Further,
    Disabled,
}

pub fn camera_look_at<T: Component>(
    time: Res<Time>,
    mut camera: Query<&mut Transform, (With<Camera>, Without<T>)>,
    target: Query<&GlobalTransform, With<T>>,
    state: Res<State<CameraState>>,
) {
    if matches!(state.get(), CameraState::Disabled) {
        return;
    }

    for mut camera in &mut camera {
        let target = target.single();

        // let mut back = target.back();

        let (distance_back, distance_up) = match state.get() {
            CameraState::Close => (3.5, 0.5),
            CameraState::Further => (7.0, 1.0),
            CameraState::Disabled => unreachable!(),
        };

        let target_pos = target.transform_point(Vec3::new(0.0, distance_up, distance_back));

        // back.y = 0.0;
        // let target_pos = (target.translation() + (back * 10.0)) + Vec3::new(5.0, 3.0, 0.0);
        // let target_pos = (target.translation() + (back * 3.0)) + Vec3::new(0.0, 0.5, 0.0);
        let new_pos = camera
            .translation
            .lerp(target_pos, time.delta_seconds() * 20.0);

        let trget_rot = camera
            .looking_at(target.translation() + (target.up() * 0.5), Vec3::Y)
            .rotation;
        // camera.look_at(target.translation(), Vec3::Y);
        camera.rotation = camera.rotation.lerp(trget_rot, time.delta_seconds() * 20.0);

        camera.translation = new_pos
    }
}

fn update_camera_state(actions: Res<Actions>, mut next_state: ResMut<NextState<CameraState>>) {
    if actions.camera_1 {
        next_state.set(CameraState::Close)
    } else if actions.camera_2 {
        next_state.set(CameraState::Further)
    } else if actions.camera_3 {
        next_state.set(CameraState::Disabled)
    }
}
