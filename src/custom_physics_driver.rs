use std::time::Duration;

use bevy::{
    ecs::{schedule::ScheduleLabel, system::SystemState},
    prelude::*,
    transform::TransformSystem,
};
use bevy_rapier3d::{
    dynamics::{RapierRigidBodyHandle, TransformInterpolation},
    plugin::{
        NoUserData, PhysicsSet, RapierConfiguration, RapierContext, RapierPhysicsPlugin,
        SimulationToRenderTime, TimestepMode,
    },
};

pub struct CustomPhysicsDriverPlugin;

impl Plugin for CustomPhysicsDriverPlugin {
    fn build(&self, app: &mut App) {
        let mut physics_config = RapierConfiguration::new(1.0);
        physics_config.timestep_mode = TimestepMode::Fixed {
            dt: 1.0 / 64.0,
            substeps: 1,
        };

        app.init_schedule(PhysicsStepSchedule)
            .insert_resource(physics_config)
            .add_plugins(
                RapierPhysicsPlugin::<NoUserData>::default().with_default_system_setup(false),
            );

        app.configure_sets(
            PhysicsStepSchedule,
            (
                PhysicsSet::SyncBackend,
                PhysicsSet::StepSimulation,
                // PhysicsSet::Writeback,
            )
                .chain()
                .before(TransformSystem::TransformPropagate),
        );

        // These *must* be in the main schedule currently so that they do not miss events.
        app.add_systems(PostUpdate, (bevy_rapier3d::plugin::systems::sync_removals,));

        app.add_systems(
            PhysicsStepSchedule,
            (
                RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::SyncBackend)
                    .in_set(PhysicsSet::SyncBackend),
                RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::StepSimulation)
                    .in_set(PhysicsSet::StepSimulation),
            ),
        );

        // app.add_systems(
        //     Update,
        //     (
        //         RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::Writeback)
        //             .in_set(PhysicsSet::Writeback),
        //     ),
        // );

        app.add_systems(
            Update,
            (
                drive_physics,
                RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::Writeback)
                    .in_set(PhysicsSet::Writeback),
            )
                .chain(),
        );

        // Warn user if the timestep mode isn't in Fixed
        // if PhysicsStepSchedule
        //     .as_dyn_eq()
        //     .dyn_eq(FixedUpdate.as_dyn_eq())
        // {
        //     let config = app.world.resource::<RapierConfiguration>();
        //     match config.timestep_mode {
        //         TimestepMode::Fixed { .. } => {}
        //         mode => {
        //             warn!(
        //                 "TimestepMode is set to `{:?}`, it is recommended to use \
        //                  `TimestepMode::Fixed` if you have the physics in `FixedUpdate`",
        //                 mode
        //             );
        //         }
        //     }
        // }
    }
}

#[derive(ScheduleLabel, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PhysicsStepSchedule;

fn drive_physics(
    world: &mut World,
    params: &mut SystemState<(
        Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
        ResMut<RapierContext>,
    )>,
) {
    let fixed_step_dt = {
        let config = world.resource::<RapierConfiguration>();
        match config.timestep_mode {
            TimestepMode::Fixed { dt, .. } => dt,
            TimestepMode::Variable { .. } => todo!(),
            TimestepMode::Interpolated { dt, .. } => dt,
        }
    };

    let mut sim_to_render_time = world
        .remove_resource::<SimulationToRenderTime>()
        .unwrap_or_default();
    let old_time = world.remove_resource::<Time>().unwrap();

    let mut fixed_time = Time::new_with(());
    fixed_time.advance_by(Duration::from_secs_f32(fixed_step_dt));
    world.insert_resource(fixed_time);

    let old_sim_to_render_time = sim_to_render_time.diff;
    sim_to_render_time.diff += old_time.delta_seconds();

    if sim_to_render_time.diff > 0.0 {
        // We are running one or more steps, set up interpolation

        let (mut q_interpolate, ctx) = params.get_mut(world);

        for (handle, mut interpolation) in q_interpolate.iter_mut() {
            if let Some(body) = ctx.bodies.get(handle.0) {
                // dbg!(old_sim_to_render_time, old_time);
                // if let Some(interpolated) = interpolation.lerp_slerp(
                //     ((old_sim_to_render_time + old_time.delta_seconds()).clamp(0.0, 1.0))
                //         / fixed_step_dt,
                // ) {
                //     // interpolated_pos = utils::iso_to_transform(&interpolated);
                //     interpolation.start = Some(interpolated);
                // } else {
                // }

                interpolation.start = Some(interpolation.end.unwrap_or(*body.position()));
                interpolation.end = None;
            }
        }
    }

    while sim_to_render_time.diff > 0.0 {
        world.insert_resource(SimulationToRenderTime {
            diff: sim_to_render_time.diff,
        });

        world.run_schedule(PhysicsStepSchedule);

        sim_to_render_time.diff -= fixed_step_dt;
    }

    world.insert_resource(sim_to_render_time);
    world.insert_resource(old_time);
}
