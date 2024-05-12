//! A vehicle controller based on ray-casting, ported and modified from Bullet’s `btRaycastVehicle`.

use std::collections::VecDeque;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_rapier3d::dynamics::{
    ExternalImpulse, RapierRigidBodyHandle, ReadMassProperties, Sleeping, Velocity,
};
use bevy_rapier3d::geometry::{Collider, RapierColliderHandle};
use bevy_rapier3d::na::Vector3;
use bevy_rapier3d::parry::math::Isometry;
use bevy_rapier3d::pipeline::QueryFilter;
use bevy_rapier3d::plugin::{PhysicsSet, RapierContext};
use bevy_rapier3d::rapier::dynamics::{RigidBody, RigidBodyHandle};
use bevy_rapier3d::rapier::math::Real;
use egui_plot::{CoordinatesFormatter, Corner, Legend, Line, Plot, PlotPoints};
use transform_gizmo_bevy::gizmo;

pub struct RaycastVehiclePlugin;

impl Plugin for RaycastVehiclePlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<DynamicRayCastVehicleController>()
            .register_type::<Wheel>()
            .register_type::<WheelTuning>()
            .add_systems(Update, debug_wheels)
            .add_systems(
                FixedUpdate,
                (update_vehicles, update_wheel_visuals)
                    .chain()
                    .after(PhysicsSet::Writeback),
            );
    }
}

type Point = Vec3;

/// A character controller to simulate vehicles using ray-casting for the wheels.
#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct DynamicRayCastVehicleController {
    // wheels: Vec<Wheel>,
    // forward_ws: Vec<Vec3>,
    // axle: Vec<Vec3>,
    /// The current forward speed of the vehicle.
    pub current_vehicle_speed: Real,

    /// The chassis’ local _up_ direction (`0 = x, 1 = y, 2 = z`)
    pub index_up_axis: usize,
    /// The chassis’ local _forward_ direction (`0 = x, 1 = y, 2 = z`)
    pub index_forward_axis: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Reflect)]
/// Parameters affecting the physical behavior of a wheel.
pub struct WheelTuning {
    /// The suspension stiffness.
    ///
    /// Increase this value if the suspension appears to not push the vehicle strong enough.
    pub suspension_stiffness: Real,
    /// The suspension’s damping when it is being compressed.
    pub suspension_compression: Real,
    /// The suspension’s damping when it is being released.
    ///
    /// Increase this value if the suspension appears to overshoot.
    pub suspension_damping: Real,
    /// The maximum distance the suspension can travel before and after its resting length.
    pub max_suspension_travel: Real,
    /// The multiplier of friction between a tire and the collider it's on top of.
    pub side_friction_stiffness: Real,
    /// Parameter controlling how much traction the tire has.
    ///
    /// The larger the value, the more instantaneous braking will happen (with the risk of
    /// causing the vehicle to flip if it’s too strong).
    pub friction_slip: Real,
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,
}

impl Default for WheelTuning {
    fn default() -> Self {
        Self {
            suspension_stiffness: 5.88,
            suspension_compression: 0.83,
            suspension_damping: 0.88,
            max_suspension_travel: 5.0,
            side_friction_stiffness: 1.0,
            friction_slip: 10.5,
            max_suspension_force: 6000.0,
        }
    }
}

/// Objects used to initialize a wheel.
#[derive(Reflect)]
pub struct WheelDesc {
    /// The position of the wheel, relative to the chassis.
    pub chassis_connection_cs: Point,
    /// The direction of the wheel’s suspension, relative to the chassis.
    ///
    /// The ray-casting will happen following this direction to detect the ground.
    pub direction_cs: Vec3,
    /// The wheel’s axle axis, relative to the chassis.
    pub axle_cs: Vec3,
    /// The rest length of the wheel’s suspension spring.
    pub suspension_rest_length: Real,
    /// The maximum distance the suspension can travel before and after its resting length.
    pub max_suspension_travel: Real,
    /// The wheel’s radius.
    pub radius: Real,

    /// The suspension stiffness.
    ///
    /// Increase this value if the suspension appears to not push the vehicle strong enough.
    pub suspension_stiffness: Real,
    /// The suspension’s damping when it is being compressed.
    pub damping_compression: Real,
    /// The suspension’s damping when it is being released.
    ///
    /// Increase this value if the suspension appears to overshoot.
    pub damping_relaxation: Real,
    /// Parameter controlling how much traction the tire has.
    ///
    /// The larger the value, the more instantaneous braking will happen (with the risk of
    /// causing the vehicle to flip if it’s too strong).
    pub friction_slip: Real,
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,
    /// The multiplier of friction between a tire and the collider it's on top of.
    pub side_friction_stiffness: Real,
}

impl WheelDesc {
    pub fn new(
        chassis_connection_cs: Point,
        direction_cs: Vec3,
        axle_cs: Vec3,
        suspension_rest_length: Real,
        radius: Real,
        tuning: &WheelTuning,
    ) -> Self {
        WheelDesc {
            chassis_connection_cs,
            direction_cs,
            axle_cs,
            suspension_rest_length,
            radius,
            suspension_stiffness: tuning.suspension_stiffness,
            damping_compression: tuning.suspension_compression,
            damping_relaxation: tuning.suspension_damping,
            friction_slip: tuning.friction_slip,
            max_suspension_travel: tuning.max_suspension_travel,
            max_suspension_force: tuning.max_suspension_force,
            side_friction_stiffness: tuning.side_friction_stiffness,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
/// A wheel attached to a vehicle.
#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct Wheel {
    raycast_info: RayCastInfo,

    center: Point,
    wheel_direction_ws: Vec3,
    wheel_axle_ws: Vec3,

    forward_ws: Vec3,
    axle: Vec3,

    /// The position of the wheel, relative to the chassis.
    pub chassis_connection_point_cs: Point,
    /// The direction of the wheel’s suspension, relative to the chassis.
    ///
    /// The ray-casting will happen following this direction to detect the ground.
    pub direction_cs: Vec3,
    /// The wheel’s axle axis, relative to the chassis.
    pub axle_cs: Vec3,
    /// The rest length of the wheel’s suspension spring.
    pub suspension_rest_length: Real,
    /// The maximum distance the suspension can travel before and after its resting length.
    pub max_suspension_travel: Real,
    /// The wheel’s radius.
    pub radius: Real,
    /// The suspension stiffness.
    ///
    /// Increase this value if the suspension appears to not push the vehicle strong enough.
    pub suspension_stiffness: Real,
    /// The suspension’s damping when it is being compressed.
    pub damping_compression: Real,
    /// The suspension’s damping when it is being released.
    ///
    /// Increase this value if the suspension appears to overshoot.
    pub damping_relaxation: Real,
    /// Parameter controlling how much traction the tire has.
    ///
    /// The larger the value, the more instantaneous braking will happen (with the risk of
    /// causing the vehicle to flip if it’s too strong).
    pub friction_slip: Real,
    /// The multiplier of friction between a tire and the collider it's on top of.
    pub side_friction_stiffness: Real,
    /// The wheel’s current rotation on its axle.
    pub rotation: Real,
    delta_rotation: Real,
    roll_influence: Real, // TODO: make this public?
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,

    /// The forward impulses applied by the wheel on the chassis.
    pub forward_impulse: Real,
    /// The side impulses applied by the wheel on the chassis.
    pub side_impulse: Real,

    /// The steering angle for this wheel.
    pub steering: Real,
    /// The forward force applied by this wheel on the chassis.
    pub engine_force: Real,
    /// The maximum amount of braking impulse applied to slow down the vehicle.
    pub brake: Real,

    clipped_inv_contact_dot_suspension: Real,
    suspension_relative_velocity: Real,
    /// The force applied by the suspension.
    pub wheel_suspension_force: Real,
    skid_info: Real,
}

impl Wheel {
    pub fn new(info: WheelDesc) -> Self {
        Self {
            raycast_info: RayCastInfo::default(),
            suspension_rest_length: info.suspension_rest_length,
            max_suspension_travel: info.max_suspension_travel,
            radius: info.radius,
            suspension_stiffness: info.suspension_stiffness,
            damping_compression: info.damping_compression,
            damping_relaxation: info.damping_relaxation,
            chassis_connection_point_cs: info.chassis_connection_cs,
            direction_cs: info.direction_cs,
            axle_cs: info.axle_cs,
            wheel_direction_ws: info.direction_cs,
            wheel_axle_ws: info.axle_cs,
            center: Point::ZERO,
            friction_slip: info.friction_slip,
            steering: 0.0,
            engine_force: 0.0,
            rotation: 0.0,
            delta_rotation: 0.0,
            brake: 0.0,
            roll_influence: 0.1,
            clipped_inv_contact_dot_suspension: 0.0,
            suspension_relative_velocity: 0.0,
            wheel_suspension_force: 0.0,
            max_suspension_force: info.max_suspension_force,
            skid_info: 0.0,
            side_impulse: 0.0,
            forward_impulse: 0.0,
            side_friction_stiffness: info.side_friction_stiffness,
            forward_ws: Vec3::ZERO,
            axle: Vec3::ZERO,
        }
    }

    pub fn apply_tuning(&mut self, info: &WheelTuning) {
        self.suspension_stiffness = info.suspension_stiffness;
        self.damping_compression = info.suspension_compression;
        self.damping_relaxation = info.suspension_damping;
        self.friction_slip = info.friction_slip;
        self.max_suspension_travel = info.max_suspension_travel;
        self.max_suspension_force = info.max_suspension_force;
        self.side_friction_stiffness = info.side_friction_stiffness;
    }

    /// Information about suspension and the ground obtained from the ray-casting
    /// for this wheel.
    pub fn raycast_info(&self) -> &RayCastInfo {
        &self.raycast_info
    }

    /// The world-space center of the wheel.
    pub fn center(&self) -> Point {
        self.center
    }

    /// The world-space direction of the wheel’s suspension.
    pub fn suspension(&self) -> Vec3 {
        self.wheel_direction_ws
    }

    /// The world-space direction of the wheel’s axle.
    pub fn axle(&self) -> Vec3 {
        self.wheel_axle_ws
    }

    pub fn suspension_impulse(&self, dt: f32) -> Vec3 {
        let mut suspension_force = self.wheel_suspension_force;

        if suspension_force > self.max_suspension_force {
            suspension_force = self.max_suspension_force;
        }
        // if suspension_force < 0.0 {
        //     suspension_force = 0.0;
        // }

        // gizmos.

        let impulse = self.raycast_info.contact_normal_ws * suspension_force * dt;
        impulse
    }
}

/// Information about suspension and the ground obtained from the ray-casting
/// to simulate a wheel’s suspension.
#[derive(Copy, Clone, Debug, PartialEq, Default, Reflect)]
pub struct RayCastInfo {
    /// The (world-space) contact normal between the wheel and the floor.
    pub contact_normal_ws: Vec3,
    /// The (world-space) point hit by the wheel’s ray-cast.
    pub contact_point_ws: Point,
    /// The suspension length for the wheel.
    pub suspension_length: Real,
    /// The (world-space) starting point of the ray-cast.
    pub hard_point_ws: Point,
    /// Is the wheel in contact with the ground?
    pub is_in_contact: bool,
    /// The collider hit by the ray-cast.
    pub ground_object: Option<Entity>,
}

impl DynamicRayCastVehicleController {
    /// Creates a new vehicle represented by the given rigid-body.
    ///
    /// Wheels have to be attached afterwards calling [`Self::add_wheel`].
    pub fn new() -> Self {
        Self {
            // wheels: vec![],
            // forward_ws: vec![],
            // axle: vec![],
            current_vehicle_speed: 0.0,
            index_up_axis: 1,
            index_forward_axis: 3,
        }
    }

    //
    // basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
    //index_forward_axis
    /// Adds a wheel to this vehicle.
    // pub fn add_wheel(
    //     &mut self,
    //     chassis_connection_cs: Point,
    //     direction_cs: Vec3,
    //     axle_cs: Vec3,
    //     suspension_rest_length: Real,
    //     radius: Real,
    //     tuning: &WheelTuning,
    // ) -> &mut Wheel {
    //     let ci = WheelDesc {
    //         chassis_connection_cs,
    //         direction_cs,
    //         axle_cs,
    //         suspension_rest_length,
    //         radius,
    //         suspension_stiffness: tuning.suspension_stiffness,
    //         damping_compression: tuning.suspension_compression,
    //         damping_relaxation: tuning.suspension_damping,
    //         friction_slip: tuning.friction_slip,
    //         max_suspension_travel: tuning.max_suspension_travel,
    //         max_suspension_force: tuning.max_suspension_force,
    //         side_friction_stiffness: tuning.side_friction_stiffness,
    //     };

    //     let wheel_id = self.wheels.len();
    //     self.wheels.push(Wheel::new(ci));

    //     &mut self.wheels[wheel_id]
    // }

    fn update_wheel_transform(&mut self, chassis_transform: &Transform, wheel: &mut Wheel) {
        self.update_wheel_transforms_ws(chassis_transform, wheel);

        let steering_orn = Quat::from_scaled_axis(-wheel.wheel_direction_ws * wheel.steering);

        // let steering_orn = Rotation::new(-wheel.wheel_direction_ws * wheel.steering);
        wheel.wheel_axle_ws = steering_orn * (chassis_transform.rotation * wheel.axle_cs);
        wheel.center = wheel.raycast_info.hard_point_ws
            + wheel.wheel_direction_ws * wheel.raycast_info.suspension_length;
    }

    fn update_wheel_transforms_ws(&mut self, chassis_transform: &Transform, wheel: &mut Wheel) {
        wheel.raycast_info.is_in_contact = false;

        wheel.raycast_info.hard_point_ws =
            chassis_transform.transform_point(wheel.chassis_connection_point_cs);
        wheel.wheel_direction_ws = chassis_transform.rotation * wheel.direction_cs;
        // wheel.wheel_axle_ws = chassis_transform.transform_point(wheel.axle_cs);
    }

    fn ray_cast(
        &mut self,
        ctx: &RapierContext,
        // bodies: &RigidBodySet,
        // colliders: &ColliderSet,
        // queries: &QueryPipeline,
        filter: QueryFilter,
        // chassis: &RigidBody,
        wheel: &mut Wheel,
        colliders: &Query<(&Collider, &Transform)>,
        chassis_velocity: &Velocity,
        chassis_center_of_mass_world: Vec3,
    ) {
        // let wheel = &mut self.wheels[wheel_id];
        // wheel.suspension_rest_length + wheel.max_suspension_travel
        let raylen = wheel.suspension_rest_length + wheel.max_suspension_travel + wheel.radius;
        let rayvector = wheel.wheel_direction_ws * raylen;
        let source = wheel.raycast_info.hard_point_ws;
        wheel.raycast_info.contact_point_ws = source + rayvector;
        // let ray = Ray::new(source, rayvector);
        // let hit = queries.cast_ray_and_get_normal(bodies, colliders, &ray, 1.0, true, filter);
        let hit = ctx.cast_ray_and_get_normal(source, rayvector, 1.0, true, filter);

        wheel.raycast_info.ground_object = None;

        if let Some((entity_hit, mut hit)) = hit {
            if hit.time_of_impact == 0.0 {
                // let collider = &colliders[collider_hit];
                // hit.
                let (collider, coll_trans) = colliders.get(entity_hit).unwrap();

                let up_ray = bevy_rapier3d::parry::query::Ray::new(
                    (source + rayvector).into(),
                    (-rayvector).into(),
                );
                // collider_hit
                if let Some(hit2) = collider.raw.cast_ray_and_get_normal(
                    &Isometry::from_parts(
                        coll_trans.translation.into(),
                        coll_trans.rotation.into(),
                    ),
                    &up_ray,
                    1.0,
                    false,
                ) {
                    hit.normal = (-hit2.normal).into();
                }

                if hit.normal == Vec3::ZERO {
                    // If the hit is still not defined, set the normal.
                    hit.normal = -wheel.wheel_direction_ws;
                }
            }

            wheel.raycast_info.contact_normal_ws = hit.normal;
            wheel.raycast_info.is_in_contact = true;
            wheel.raycast_info.ground_object = Some(entity_hit);

            let hit_distance = hit.time_of_impact * raylen;
            wheel.raycast_info.suspension_length = hit_distance - wheel.radius;

            // clamp on max suspension travel
            let min_suspension_length = wheel.suspension_rest_length - wheel.max_suspension_travel;
            let max_suspension_length = wheel.suspension_rest_length + wheel.max_suspension_travel;
            wheel.raycast_info.suspension_length = wheel
                .raycast_info
                .suspension_length
                .clamp(min_suspension_length, max_suspension_length);
            // wheel.raycast_info.contact_point_ws = ray.point_at(hit.toi);
            wheel.raycast_info.contact_point_ws = hit.point;

            let denominator = wheel
                .raycast_info
                .contact_normal_ws
                .dot(wheel.wheel_direction_ws);

            let chassis_velocity_at_contact_point = chassis_velocity.linear_velocity_at_point(
                wheel.raycast_info.contact_point_ws,
                chassis_center_of_mass_world,
            );

            let proj_vel = wheel
                .raycast_info
                .contact_normal_ws
                .dot(chassis_velocity_at_contact_point);

            if denominator >= -0.1 {
                wheel.suspension_relative_velocity = 0.0;
                wheel.clipped_inv_contact_dot_suspension = 1.0 / 0.1;
            } else {
                let inv = -1.0 / denominator;
                // TODO: verify this
                // wheel.suspension_relative_velocity = proj_vel * inv;
                wheel.suspension_relative_velocity = proj_vel;
                wheel.clipped_inv_contact_dot_suspension = inv;
            }
        } else {
            // No contact, put wheel info as in rest position
            wheel.raycast_info.suspension_length = wheel.suspension_rest_length;
            wheel.suspension_relative_velocity = 0.0;
            wheel.raycast_info.contact_normal_ws = -wheel.wheel_direction_ws;
            wheel.clipped_inv_contact_dot_suspension = 1.0;
        }
    }

    /// Updates the vehicle’s velocity based on its suspension, engine force, and brake.
    // pub fn update_vehicle(
    //     &mut self,
    //     dt: Real,
    //     bodies: &mut RigidBodySet,
    //     colliders: &ColliderSet,
    //     queries: &QueryPipeline,
    //     filter: QueryFilter,
    // ) {
    //     let num_wheels = self.wheels.len();
    //     let chassis = &bodies[self.chassis];

    //     for i in 0..num_wheels {
    //         self.update_wheel_transform(chassis, i);
    //     }

    //     self.current_vehicle_speed = chassis.linvel().norm();

    //     let forward_w = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);

    //     if forward_w.dot(chassis.linvel()) < 0.0 {
    //         self.current_vehicle_speed *= -1.0;
    //     }

    //     //
    //     // simulate suspension
    //     //

    //     for wheel_id in 0..self.wheels.len() {
    //         self.ray_cast(bodies, colliders, queries, filter, chassis, wheel_id);
    //     }

    //     let chassis_mass = chassis.mass();
    //     self.update_suspension(chassis_mass);

    //     let chassis = bodies
    //         .get_mut_internal_with_modification_tracking(self.chassis)
    //         .unwrap();

    //     for wheel in &mut self.wheels {
    //         if wheel.engine_force > 0.0 {
    //             chassis.wake_up(true);
    //         }

    //         // apply suspension force
    //         let mut suspension_force = wheel.wheel_suspension_force;

    //         if suspension_force > wheel.max_suspension_force {
    //             suspension_force = wheel.max_suspension_force;
    //         }

    //         let impulse = wheel.raycast_info.contact_normal_ws * suspension_force * dt;
    //         chassis.apply_impulse_at_point(impulse, wheel.raycast_info.contact_point_ws, false);
    //     }

    //     self.update_friction(bodies, colliders, dt);

    //     let chassis = bodies
    //         .get_mut_internal_with_modification_tracking(self.chassis)
    //         .unwrap();

    //     for wheel in &mut self.wheels {
    //         let vel = chassis.velocity_at_point(&wheel.raycast_info.hard_point_ws);

    //         if wheel.raycast_info.is_in_contact {
    //             let mut fwd = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);
    //             let proj = fwd.dot(&wheel.raycast_info.contact_normal_ws);
    //             fwd -= wheel.raycast_info.contact_normal_ws * proj;

    //             let proj2 = fwd.dot(&vel);

    //             wheel.delta_rotation = (proj2 * dt) / (wheel.radius);
    //             wheel.rotation += wheel.delta_rotation;
    //         } else {
    //             wheel.rotation += wheel.delta_rotation;
    //         }

    //         wheel.delta_rotation *= 0.99; //damping of rotation when not in contact
    //     }
    // }

    // /// Reference to all the wheels attached to this vehicle.
    // pub fn wheels(&self) -> &[Wheel] {
    //     &self.wheels
    // }

    // /// Mutable reference to all the wheels attached to this vehicle.
    // pub fn wheels_mut(&mut self) -> &mut [Wheel] {
    //     &mut self.wheels
    // }

    // fn update_suspension(&mut self, chassis_mass: Real) {
    //     for w_it in 0..self.wheels.len() {
    //         let wheels = &mut self.wheels[w_it];
    //         self.update_wheel_suspension(chassis_mass, wheels)
    //     }
    // }

    fn update_wheel_suspension(&mut self, chassis_mass: Real, wheel: &mut Wheel) {
        if wheel.raycast_info.is_in_contact {
            let mut force;
            //	Spring
            {
                let rest_length = wheel.suspension_rest_length;
                let current_length = wheel.raycast_info.suspension_length;
                let length_diff = rest_length - current_length;

                force = wheel.suspension_stiffness * (length_diff * 2.0);
                // The harsher the angle the more suspension? this makes no sense to me
                // * wheel.clipped_inv_contact_dot_suspension;
            }

            // Damper
            {
                let projected_rel_vel = wheel.suspension_relative_velocity;
                {
                    let susp_damping = if projected_rel_vel < 0.0 {
                        wheel.damping_compression
                    } else {
                        wheel.damping_relaxation
                    };
                    force -= susp_damping * projected_rel_vel;
                }
            }

            // RESULT
            // wheel.wheel_suspension_force = (force * chassis_mass).max(0.0);
            wheel.wheel_suspension_force = (force * chassis_mass).max(0.0);
        } else {
            wheel.wheel_suspension_force = 0.0;
        }
    }

    // fn update_friction(&mut self, bodies: &mut RigidBodySet, colliders: &ColliderSet, dt: Real) {
    //     let num_wheels = self.wheels.len();

    //     if num_wheels == 0 {
    //         return;
    //     }

    //     self.forward_ws.resize(num_wheels, Default::default());
    //     self.axle.resize(num_wheels, Default::default());

    //     let mut num_wheels_on_ground = 0;

    //     //TODO: collapse all those loops into one!
    //     for wheel in &mut self.wheels {
    //         let ground_object = wheel.raycast_info.ground_object;

    //         if ground_object.is_some() {
    //             num_wheels_on_ground += 1;
    //         }

    //         wheel.side_impulse = 0.0;
    //         wheel.forward_impulse = 0.0;
    //     }

    //     {
    //         for i in 0..num_wheels {
    //             let wheel = &mut self.wheels[i];
    //             let ground_object = wheel.raycast_info.ground_object;

    //             if ground_object.is_some() {
    //                 self.axle[i] = wheel.wheel_axle_ws;

    //                 let surf_normal_ws = wheel.raycast_info.contact_normal_ws;
    //                 let proj = self.axle[i].dot(&surf_normal_ws);
    //                 self.axle[i] -= surf_normal_ws * proj;
    //                 self.axle[i] = self.axle[i]
    //                     .try_normalize(1.0e-5)
    //                     .unwrap_or_else(Vector::zeros);
    //                 wheel.forward_ws = surf_normal_ws
    //                     .cross(&self.axle[i])
    //                     .try_normalize(1.0e-5)
    //                     .unwrap_or_else(Vector::zeros);

    //                 if let Some(ground_body) = ground_object
    //                     .and_then(|h| colliders[h].parent())
    //                     .map(|h| &bodies[h])
    //                     .filter(|b| b.is_dynamic())
    //                 {
    //                     wheel.side_impulse = resolve_single_bilateral(
    //                         &bodies[self.chassis],
    //                         &wheel.raycast_info.contact_point_ws,
    //                         &ground_body,
    //                         &wheel.raycast_info.contact_point_ws,
    //                         &self.axle[i],
    //                     );
    //                 } else {
    //                     wheel.side_impulse = resolve_single_unilateral(
    //                         &bodies[self.chassis],
    //                         &wheel.raycast_info.contact_point_ws,
    //                         &self.axle[i],
    //                     );
    //                 }

    //                 wheel.side_impulse *= wheel.side_friction_stiffness;
    //             }
    //         }
    //     }

    //     let side_factor = 1.0;
    //     let fwd_factor = 0.5;

    //     let mut sliding = false;
    //     {
    //         for wheel_id in 0..num_wheels {
    //             let wheel = &mut self.wheels[wheel_id];
    //             let ground_object = wheel.raycast_info.ground_object;

    //             let mut rolling_friction = 0.0;

    //             if ground_object.is_some() {
    //                 if wheel.engine_force != 0.0 {
    //                     rolling_friction = wheel.engine_force * dt;
    //                 } else {
    //                     let default_rolling_friction_impulse = 0.0;
    //                     let max_impulse = if wheel.brake != 0.0 {
    //                         wheel.brake
    //                     } else {
    //                         default_rolling_friction_impulse
    //                     };
    //                     let contact_pt = WheelContactPoint::new(
    //                         &bodies[self.chassis],
    //                         ground_object
    //                             .and_then(|h| colliders[h].parent())
    //                             .map(|h| &bodies[h]),
    //                         wheel.raycast_info.contact_point_ws,
    //                         self.forward_ws[wheel_id],
    //                         max_impulse,
    //                     );
    //                     assert!(num_wheels_on_ground > 0);
    //                     rolling_friction = contact_pt.calc_rolling_friction(num_wheels_on_ground);
    //                 }
    //             }

    //             //switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

    //             wheel.forward_impulse = 0.0;
    //             wheel.skid_info = 1.0;

    //             if ground_object.is_some() {
    //                 let max_imp = wheel.wheel_suspension_force * dt * wheel.friction_slip;
    //                 let max_imp_side = max_imp;
    //                 let max_imp_squared = max_imp * max_imp_side;
    //                 assert!(max_imp_squared >= 0.0);

    //                 wheel.forward_impulse = rolling_friction;

    //                 let x = wheel.forward_impulse * fwd_factor;
    //                 let y = wheel.side_impulse * side_factor;

    //                 let impulse_squared = x * x + y * y;

    //                 if impulse_squared > max_imp_squared {
    //                     sliding = true;

    //                     let factor = max_imp * crate::utils::inv(impulse_squared.sqrt());
    //                     wheel.skid_info *= factor;
    //                 }
    //             }
    //         }
    //     }

    //     if sliding {
    //         for wheel in &mut self.wheels {
    //             if wheel.side_impulse != 0.0 {
    //                 if wheel.skid_info < 1.0 {
    //                     wheel.forward_impulse *= wheel.skid_info;
    //                     wheel.side_impulse *= wheel.skid_info;
    //                 }
    //             }
    //         }
    //     }

    //     // apply the impulses
    //     {
    //         let chassis = bodies
    //             .get_mut_internal_with_modification_tracking(self.chassis)
    //             .unwrap();

    //         for wheel_id in 0..num_wheels {
    //             let wheel = &self.wheels[wheel_id];
    //             let mut impulse_point = wheel.raycast_info.contact_point_ws;

    //             if wheel.forward_impulse != 0.0 {
    //                 chassis.apply_impulse_at_point(
    //                     self.forward_ws[wheel_id] * wheel.forward_impulse,
    //                     impulse_point,
    //                     false,
    //                 );
    //             }
    //             if wheel.side_impulse != 0.0 {
    //                 let side_impulse = self.axle[wheel_id] * wheel.side_impulse;

    //                 let v_chassis_world_up =
    //                     chassis.position().rotation * Vector::ith(self.index_up_axis, 1.0);
    //                 impulse_point -= v_chassis_world_up
    //                     * (v_chassis_world_up.dot(&(impulse_point - chassis.center_of_mass()))
    //                         * (1.0 - wheel.roll_influence));

    //                 chassis.apply_impulse_at_point(side_impulse, impulse_point, false);

    //                 // TODO: apply friction impulse on the ground
    //                 // let ground_object = self.wheels[wheel_id].raycast_info.ground_object;
    //                 // ground_object.apply_impulse_at_point(
    //                 //     -side_impulse,
    //                 //     wheels.raycast_info.contact_point_ws,
    //                 //     false,
    //                 // );
    //             }
    //         }
    //     }
    // }

    fn update_friction_bevy(
        &mut self,
        dt: Real,
        context: &RapierContext,
        // wheels: &mut Vec<&mut Wheel>,
        chassis_handle: RigidBodyHandle,
        chassis_ext_impulse: &mut ExternalImpulse,
        chassis_rotation: Quat,
        chassis_center_of_mass_world: Vec3,
        // wheels: &mut Vec<Mut<'_, Wheel>>,
        wheel_entities: Vec<Entity>,
        wheels: &mut Query<&mut Wheel>,
        // bodies: &mut RigidBodySet,
        // colliders: &ColliderSet,
        colliders: &Query<&RapierColliderHandle>,
    ) {
        if wheel_entities.is_empty() {
            return;
        }

        // self.forward_ws.resize(num_wheels, Default::default());
        // self.axle.resize(num_wheels, Default::default());

        let mut num_wheels_on_ground = 0;

        //TODO: collapse all those loops into one!
        for wheel_e in &wheel_entities {
            let mut wheel = wheels.get_mut(*wheel_e).unwrap();

            let ground_object: Option<Entity> = wheel.raycast_info.ground_object;

            if ground_object.is_some() {
                num_wheels_on_ground += 1;
            }

            wheel.side_impulse = 0.0;
            wheel.forward_impulse = 0.0;
        }

        {
            for wheel_e in &wheel_entities {
                let mut wheel = wheels.get_mut(*wheel_e).unwrap();
                // for i in 0..num_wheels {
                // let wheel = &mut self.wheels[i];
                let ground_object = wheel.raycast_info.ground_object;

                if ground_object.is_some() {
                    wheel.axle = wheel.wheel_axle_ws;

                    let surf_normal_ws = wheel.raycast_info.contact_normal_ws;
                    let proj = wheel.axle.dot(surf_normal_ws);
                    wheel.axle -= surf_normal_ws * proj;
                    wheel.axle = wheel.axle.normalize_or_zero();

                    wheel.forward_ws = surf_normal_ws.cross(wheel.axle).normalize_or_zero();

                    if let Some(ground_body) = ground_object
                        // .and_then(|h| colliders[h].parent())
                        .and_then(|h| colliders.get(h).ok())
                        .and_then(|v| context.colliders[v.0].parent())
                        // .map(|h| &bodies[h])
                        .map(|h| &context.bodies[h])
                        .filter(|b| b.is_dynamic())
                    {
                        wheel.side_impulse = resolve_single_bilateral(
                            // &bodies[self.chassis],
                            &context.bodies[chassis_handle],
                            &wheel.raycast_info.contact_point_ws,
                            &ground_body,
                            &wheel.raycast_info.contact_point_ws,
                            &wheel.axle,
                        );
                    } else {
                        wheel.side_impulse = resolve_single_unilateral(
                            &context.bodies[chassis_handle],
                            &wheel.raycast_info.contact_point_ws,
                            &wheel.axle,
                        );
                    }

                    wheel.side_impulse *= wheel.side_friction_stiffness;
                }
            }
        }

        let side_factor = 1.0;
        let fwd_factor = 0.0;
        let mut factors = Vec::new();

        let mut sliding = false;
        {
            for wheel_e in &wheel_entities {
                let mut wheel = wheels.get_mut(*wheel_e).unwrap();
                let ground_object = wheel.raycast_info.ground_object;

                let mut rolling_friction = 0.0;

                if ground_object.is_some() {
                    if wheel.engine_force != 0.0 {
                        rolling_friction = wheel.engine_force * dt;
                    } else {
                        let default_rolling_friction_impulse = 0.0;
                        let max_impulse = if wheel.brake != 0.0 {
                            wheel.brake
                        } else {
                            default_rolling_friction_impulse
                        };
                        let contact_pt = WheelContactPoint::new(
                            // &bodies[self.chassis],
                            &context.bodies[chassis_handle],
                            ground_object
                                .and_then(|h| colliders.get(h).ok())
                                .and_then(|h| context.colliders.get(h.0).unwrap().parent())
                                // .and_then(|h| colliders[h].parent())
                                // .map(|h| &bodies[h]),
                                .map(|h| &context.bodies[h]),
                            wheel.raycast_info.contact_point_ws,
                            wheel.forward_ws,
                            max_impulse,
                        );
                        assert!(num_wheels_on_ground > 0);
                        rolling_friction = contact_pt.calc_rolling_friction(num_wheels_on_ground);
                    }
                }

                //switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

                wheel.forward_impulse = 0.0;
                wheel.skid_info = 1.0;

                if ground_object.is_some() {
                    let max_imp = wheel.wheel_suspension_force * dt * wheel.friction_slip;
                    // let max_imp = 500.0 * dt * wheel.friction_slip;
                    let max_imp_side = max_imp;
                    let max_imp_squared = max_imp * max_imp_side;
                    assert!(max_imp_squared >= 0.0);

                    wheel.forward_impulse = rolling_friction;

                    let x = wheel.forward_impulse * fwd_factor;
                    let y = wheel.side_impulse * side_factor;

                    let impulse_squared = x * x + y * y;
                    // dbg!(
                    //     impulse_squared,
                    //     max_imp_squared,
                    //     wheel.wheel_suspension_force
                    // );

                    // TODO: this seems unreliable!
                    if impulse_squared > max_imp_squared {
                        sliding = true;

                        let factor = max_imp * inv(impulse_squared.sqrt());
                        factors.push(factor);
                        wheel.skid_info *= factor;
                    } else {
                        // dbg!("Not higher!");
                    }
                }
            }
        }
        // dbg!(&factors);

        if sliding {
            for wheel_e in &wheel_entities {
                let mut wheel = wheels.get_mut(*wheel_e).unwrap();
                if wheel.side_impulse != 0.0 {
                    if wheel.skid_info < 1.0 {
                        wheel.forward_impulse *= wheel.skid_info;
                        let delta = wheel.side_impulse - (wheel.side_impulse * wheel.skid_info);
                        wheel.side_impulse -= delta / 1.2;
                    }
                }
            }
        }

        // apply the impulses
        {
            // let chassis = context
            //     .bodies
            //     .get_mut_internal_with_modification_tracking(self.chassis)
            //     .unwrap();

            for wheel_e in &wheel_entities {
                let wheel = wheels.get_mut(*wheel_e).unwrap();
                let mut impulse_point = wheel.raycast_info.contact_point_ws;

                if wheel.forward_impulse != 0.0 {
                    *chassis_ext_impulse += ExternalImpulse::at_point(
                        wheel.forward_ws * wheel.forward_impulse,
                        impulse_point,
                        chassis_center_of_mass_world,
                    );
                }
                if wheel.side_impulse != 0.0 {
                    let side_impulse = wheel.axle * wheel.side_impulse;

                    let v_chassis_world_up = chassis_rotation
                        * Vec3::from(bevy_rapier3d::parry::math::Vector::ith(
                            self.index_up_axis,
                            1.0,
                        ));

                    impulse_point -= v_chassis_world_up
                        * (v_chassis_world_up.dot(impulse_point - chassis_center_of_mass_world)
                            * (1.0 - wheel.roll_influence));

                    *chassis_ext_impulse += ExternalImpulse::at_point(
                        side_impulse,
                        impulse_point,
                        chassis_center_of_mass_world,
                    );

                    // TODO: apply friction impulse on the ground
                    // let ground_object = self.wheels[wheel_id].raycast_info.ground_object;
                    // ground_object.apply_impulse_at_point(
                    //     -side_impulse,
                    //     wheels.raycast_info.contact_point_ws,
                    //     false,
                    // );
                }
            }
        }
    }
}

struct WheelContactPoint<'a> {
    body0: &'a RigidBody,
    body1: Option<&'a RigidBody>,
    friction_position_world: Point,
    friction_direction_world: Vec3,
    jac_diag_ab_inv: Real,
    max_impulse: Real,
}

impl<'a> WheelContactPoint<'a> {
    pub fn new(
        body0: &'a RigidBody,
        body1: Option<&'a RigidBody>,
        friction_position_world: Point,
        friction_direction_world: Vec3,
        max_impulse: Real,
    ) -> Self {
        fn impulse_denominator(body: &RigidBody, pos: &Point, n: &Vec3) -> Real {
            let pos = bevy_rapier3d::parry::math::Point::<f32>::from(*pos);
            let n = bevy_rapier3d::parry::math::Vector::<f32>::from(*n);

            let dpt = pos - body.center_of_mass();
            let gcross = dpt.cross(&n);
            let v = (body.mass_properties().effective_world_inv_inertia_sqrt
                * (body.mass_properties().effective_world_inv_inertia_sqrt * gcross))
                .cross(&dpt);
            // TODO: take the effective inv mass into account instead of the inv_mass?
            body.mass_properties().local_mprops.inv_mass + n.dot(&v)
        }
        let denom0 =
            impulse_denominator(body0, &friction_position_world, &friction_direction_world);
        let denom1 = body1
            .map(|body1| {
                impulse_denominator(body1, &friction_position_world, &friction_direction_world)
            })
            .unwrap_or(0.0);
        let relaxation = 1.0;
        let jac_diag_ab_inv = relaxation / (denom0 + denom1);

        Self {
            body0,
            body1,
            friction_position_world,
            friction_direction_world,
            jac_diag_ab_inv,
            max_impulse,
        }
    }

    pub fn calc_rolling_friction(&self, num_wheels_on_ground: usize) -> Real {
        let contact_pos_world = self.friction_position_world;
        let max_impulse: f32 = self.max_impulse;

        let vel1 = self.body0.velocity_at_point(&contact_pos_world.into());
        let vel2 = self
            .body1
            .map(|b| b.velocity_at_point(&contact_pos_world.into()))
            .unwrap_or_else(bevy_rapier3d::parry::math::Vector::zeros);
        let vel = vel1 - vel2;
        let vrel = self.friction_direction_world.dot(vel.into());

        // calculate friction that moves us to zero relative velocity
        (-vrel * self.jac_diag_ab_inv / (num_wheels_on_ground as Real))
            .clamp(-max_impulse, max_impulse)
    }
}

fn resolve_single_bilateral(
    body1: &RigidBody,
    pt1: &Point,
    body2: &RigidBody,
    pt2: &Point,
    normal: &Vec3,
) -> Real {
    let vel1 = body1.velocity_at_point(&(*pt1).into());
    let vel2 = body2.velocity_at_point(&(*pt2).into());
    let dvel = vel1 - vel2;

    let dpt1 = bevy_rapier3d::parry::math::Point::<f32>::from(*pt1) - body1.center_of_mass();
    let dpt2 = bevy_rapier3d::parry::math::Point::<f32>::from(*pt2) - body2.center_of_mass();
    let aj = dpt1.cross(&(*normal).into());
    let bj = dpt2.cross(&(-*normal).into());
    let iaj = body1.mass_properties().effective_world_inv_inertia_sqrt * aj;
    let ibj = body2.mass_properties().effective_world_inv_inertia_sqrt * bj;

    // TODO: take the effective_inv_mass into account?
    let im1 = body1.mass_properties().local_mprops.inv_mass;
    let im2 = body2.mass_properties().local_mprops.inv_mass;

    let jac_diag_ab = im1 + im2 + iaj.dot(&iaj) + ibj.dot(&ibj);
    let jac_diag_ab_inv = inv(jac_diag_ab);
    let rel_vel = normal.dot((dvel).into());

    //todo: move this into proper structure
    let contact_damping = 0.2;
    -contact_damping * rel_vel * jac_diag_ab_inv
}

fn resolve_single_unilateral(body1: &RigidBody, pt1: &Point, normal: &Vec3) -> Real {
    let pt1 = bevy_rapier3d::parry::math::Point::<f32>::from(*pt1);
    let normal = bevy_rapier3d::parry::math::Vector::<f32>::from(*normal);

    let vel1 = body1.velocity_at_point(&pt1);
    let dvel = vel1;
    let dpt1 = pt1 - body1.center_of_mass();
    let aj = dpt1.cross(&normal);
    let iaj = body1.mass_properties().effective_world_inv_inertia_sqrt * aj;

    // TODO: take the effective_inv_mass into account?
    let im1 = body1.mass_properties().local_mprops.inv_mass;
    let jac_diag_ab = im1 + iaj.dot(&iaj);
    let jac_diag_ab_inv = inv(jac_diag_ab);
    let rel_vel = normal.dot(&dvel);

    //todo: move this into proper structure
    let contact_damping = 0.2;
    -contact_damping * rel_vel * jac_diag_ab_inv
}

pub fn update_vehicles(
    time: Res<Time>,
    rapier_context: Res<RapierContext>,
    mut controllers: Query<(
        Entity,
        &mut DynamicRayCastVehicleController,
        &Transform,
        &mut Velocity,
        &ReadMassProperties,
        &Children,
        &mut Sleeping,
        &mut ExternalImpulse,
        &RapierRigidBodyHandle,
    )>,
    mut wheels: Query<&mut Wheel>,
    colliders: Query<(&Collider, &Transform)>,
    collider_handles: Query<&RapierColliderHandle>,
    mut gizmos: Gizmos,
) {
    for (
        chassis_entity,
        mut controller,
        transform,
        chassis_vel,
        mass_props,
        children,
        mut sleeping,
        mut external_impulse,
        rigidbody_handle,
    ) in &mut controllers
    {
        // let wheels = children.iter().find_map(wheels.g)

        controller.current_vehicle_speed = Vector3::from(chassis_vel.linvel).norm();
        let forward = -transform.forward();
        if forward.dot(chassis_vel.linvel) < 0.0 {
            controller.current_vehicle_speed *= -1.0;
        }

        let chassis_center_of_mass = transform.transform_point(mass_props.local_center_of_mass);
        for child in children {
            let Ok(mut wheel) = wheels.get_mut(*child) else {
                continue;
            };

            controller.update_wheel_transform(&transform, &mut wheel);

            controller.ray_cast(
                &rapier_context,
                QueryFilter::exclude_dynamic().exclude_rigid_body(chassis_entity),
                &mut wheel,
                &colliders,
                &chassis_vel,
                chassis_center_of_mass,
            );

            controller.update_wheel_suspension(mass_props.mass, &mut wheel);

            // ---

            if wheel.engine_force > 0.0 {
                sleeping.sleeping = false;
            }

            // apply suspension force

            let suspension_impulse = wheel.suspension_impulse(time.delta_seconds());
            *external_impulse += ExternalImpulse::at_point(
                suspension_impulse,
                wheel.raycast_info.contact_point_ws,
                chassis_center_of_mass,
            );

            // ----

            let vel = chassis_vel
                .linear_velocity_at_point(wheel.raycast_info.hard_point_ws, chassis_center_of_mass);

            if wheel.raycast_info.is_in_contact {
                // let
                // let mut fwd: Vec3 = *transform
                //     * (bevy_rapier3d::parry::math::Vector::ith(controller.index_forward_axis, 1.0)
                //         .into());
                let mut fwd: Vec3 = transform.forward().into();
                let proj = fwd.dot(wheel.raycast_info.contact_normal_ws);
                fwd -= wheel.raycast_info.contact_normal_ws * proj;

                let proj2 = fwd.dot(vel);

                wheel.delta_rotation = (proj2 * time.delta_seconds()) / (wheel.radius);
                wheel.rotation += wheel.delta_rotation;
            } else {
                wheel.rotation += wheel.delta_rotation;
            }

            wheel.delta_rotation *= 0.99; //damping of rotation when not in contact
        }

        let chassis_mass = mass_props.mass;

        // let wheel_entities = children.iter().filter(|v| wheels.chil)
        // wheels.get_mut(entity)
        let mut wheels_entities = children
            .iter()
            .filter_map(|v| wheels.contains(*v).then_some(*v))
            .collect::<Vec<_>>();
        // let mut wheels_s = wheels.get_many_mut(entities)
        // let mut wheels_s = Vec::new();
        // for child in children {
        //     if let Ok(wheel) = wheels.get_mut(*child) {
        //         wheels_s.push(wheel);
        //     }
        // }

        controller.update_friction_bevy(
            time.delta_seconds(),
            &rapier_context,
            rigidbody_handle.0,
            &mut external_impulse,
            transform.rotation,
            chassis_center_of_mass,
            wheels_entities,
            &mut wheels,
            &collider_handles,
        );
    }

    // let num_wheels = self.wheels.len();
    // let chassis = &bodies[self.chassis];

    // for i in 0..num_wheels {
    //     self.update_wheel_transform(chassis, i);
    // }

    // self.current_vehicle_speed = chassis.linvel().norm();

    // let forward_w = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);

    // if forward_w.dot(chassis.linvel()) < 0.0 {
    //     self.current_vehicle_speed *= -1.0;
    // }

    //
    // simulate suspension
    //

    // for wheel_id in 0..self.wheels.len() {
    //     self.ray_cast(bodies, colliders, queries, filter, chassis, wheel_id);
    // }

    // let chassis_mass = chassis.mass();
    // self.update_suspension(chassis_mass);

    // let chassis = bodies
    //     .get_mut_internal_with_modification_tracking(self.chassis)
    //     .unwrap();

    // for wheel in &mut self.wheels {
    //     if wheel.engine_force > 0.0 {
    //         chassis.wake_up(true);
    //     }

    //     // apply suspension force
    //     let mut suspension_force = wheel.wheel_suspension_force;

    //     if suspension_force > wheel.max_suspension_force {
    //         suspension_force = wheel.max_suspension_force;
    //     }

    //     let impulse = wheel.raycast_info.contact_normal_ws * suspension_force * dt;
    //     chassis.apply_impulse_at_point(impulse, wheel.raycast_info.contact_point_ws, false);
    // }

    // self.update_friction(bodies, colliders, dt);

    // let chassis = bodies
    //     .get_mut_internal_with_modification_tracking(self.chassis)
    //     .unwrap();

    // for wheel in &mut self.wheels {
    //     let vel = chassis.velocity_at_point(&wheel.raycast_info.hard_point_ws);

    //     if wheel.raycast_info.is_in_contact {
    //         let mut fwd = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);
    //         let proj = fwd.dot(&wheel.raycast_info.contact_normal_ws);
    //         fwd -= wheel.raycast_info.contact_normal_ws * proj;

    //         let proj2 = fwd.dot(&vel);

    //         wheel.delta_rotation = (proj2 * dt) / (wheel.radius);
    //         wheel.rotation += wheel.delta_rotation;
    //     } else {
    //         wheel.rotation += wheel.delta_rotation;
    //     }

    //     wheel.delta_rotation *= 0.99; //damping of rotation when not in contact
    // }
}

const INV_EPSILON: Real = 1.0e-20;

pub(crate) fn inv(val: Real) -> Real {
    if val >= -INV_EPSILON && val <= INV_EPSILON {
        0.0
    } else {
        1.0 / val
    }
}

#[derive(Default)]
struct DebugWheelStates {
    states: VecDeque<Vec<(Entity, Wheel)>>,
}

fn debug_wheels(
    wheels: Query<(Entity, &Wheel)>,
    mut gizmos: Gizmos,
    mut contexts: EguiContexts,
    mut state_history: Local<DebugWheelStates>,
) {
    if wheels.is_empty() {
        return;
    }

    let mut this_frame = Vec::with_capacity(10);
    for (entity, wheel) in &wheels {
        this_frame.push((entity, wheel.clone()));
    }

    state_history.states.push_back(this_frame);
    while state_history.states.len() > 1000 {
        state_history.states.pop_front();
    }

    egui::Window::new("Wheels").show(contexts.ctx_mut(), |ui| {
        let mut susp_lines: Vec<Line> = Vec::with_capacity(4);
        let mut forward_lines: Vec<Line> = Vec::with_capacity(4);

        for (entity, wheel) in &wheels {
            gizmos.line(
                wheel.center,
                wheel.center + wheel.wheel_axle_ws,
                Color::PINK,
            );

            gizmos.line(
                wheel.center,
                wheel.center + wheel.wheel_direction_ws,
                if wheel.raycast_info.ground_object.is_some() {
                    Color::GREEN
                } else {
                    Color::RED
                },
            );
            gizmos.line(wheel.center, wheel.center + wheel.forward_ws, Color::PURPLE);
            gizmos.line(
                wheel.center,
                wheel.center + (wheel.forward_impulse * wheel.forward_ws),
                Color::BLUE,
            );
            gizmos.line(
                wheel.center,
                wheel.center + (wheel.side_impulse * wheel.axle),
                Color::ORANGE,
            );

            let suspension_impulse = wheel.suspension_impulse(0.01);
            gizmos.arrow(
                wheel.raycast_info.contact_point_ws,
                wheel.raycast_info.contact_point_ws + suspension_impulse,
                Color::CYAN,
            );

            ui.vertical(|ui| {
                ui.label(format!("entity: {entity:?}"));

                let states = state_history
                    .states
                    .iter()
                    .flat_map(|v| v.iter().filter_map(|w| (w.0 == entity).then_some(&w.1)));

                let suspension_line = Line::new(PlotPoints::from_iter(
                    states
                        .clone()
                        .enumerate()
                        .map(|(i, v)| [i as f64, v.wheel_suspension_force as f64]),
                ))
                .name(format!("Susp {entity:?}"));

                let forward_line = Line::new(PlotPoints::from_iter(
                    states
                        .clone()
                        .enumerate()
                        .map(|(i, v)| [i as f64, v.forward_impulse as f64]),
                ))
                .name(format!("Forward {entity:?}"));

                susp_lines.push(suspension_line);
                forward_lines.push(forward_line);
            });
            // ui.label("world");
        }

        ui.collapsing("Suspension", |ui| {
            let plot = Plot::new("suspensions")
                .legend(Legend::default())
                // .y_axis_width(4)
                .show_axes(true)
                .show_grid(true)
                .coordinates_formatter(Corner::LeftBottom, CoordinatesFormatter::default());

            plot.show(ui, |plot_ui| {
                for line in susp_lines {
                    plot_ui.line(line);
                }
            })
        });

        ui.collapsing("Forward impulse", |ui| {
            let plot = Plot::new("forward_impulses")
                .legend(Legend::default())
                // .y_axis_width(4)
                .show_axes(true)
                .show_grid(true)
                .coordinates_formatter(Corner::LeftBottom, CoordinatesFormatter::default());

            plot.show(ui, |plot_ui| {
                for line in forward_lines {
                    plot_ui.line(line);
                }
            })
        });
    });
}

fn update_wheel_visuals(time: Res<Time>, mut wheels: Query<(&Wheel, &mut Transform)>) {
    for (wheel, mut transform) in &mut wheels {
        transform.rotation = Quat::from_rotation_y(wheel.steering);

        let new_pos = transform.translation.y.lerp(
            -wheel.raycast_info.suspension_length + wheel.chassis_connection_point_cs.y,
            time.delta_seconds() * 10.0,
        );

        transform.translation.y = new_pos;
    }
}
