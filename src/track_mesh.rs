use bevy::{
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};
use bevy_rapier3d::{
    geometry::Collider,
    math::{Rot, Vect},
};

use crate::{actions::Actions, map::Track};

const TRACK_WIDTH: f32 = 20.0;
const HALF_TRACK_WIDTH: f32 = TRACK_WIDTH / 2.0;
const TRACK_HEIGHT: f32 = 1.0;
const HALF_TRACK_HEIGHT: f32 = TRACK_HEIGHT / 2.0;

// Reusable buffers for mesh generations to avoid allocations
pub struct MeshGeneratorBuffers {
    vertices: Vec<([f32; 3], [f32; 3], [f32; 2])>,
}

impl Default for MeshGeneratorBuffers {
    fn default() -> Self {
        Self {
            vertices: Vec::with_capacity(1000),
        }
    }
}

impl MeshGeneratorBuffers {
    pub fn clear(&mut self) {
        self.vertices.clear();
    }
}

pub fn generate_track_mesh(track: &Track, gizmos: &mut Gizmos) -> Option<Mesh> {
    let Some(curve) = &track.built_curve else {
        return None;
    };

    if track.bezier_segments.is_empty() {
        return None;
    }

    let start_position = curve.position(0.0);

    let mut previous_pos = start_position;
    let mut previous_connection_points = default_connection_points(previous_pos);
    // let mut previous_connection_points_world = default_connection_points(previous_pos);

    let mut mesh_gen_buf = MeshGeneratorBuffers::default();
    let mut indices = Vec::new();

    for segment_index in 0..track.bezier_segments.len() {
        let subdivisions = 100;

        let start_rot = track.bezier_segments[segment_index][0].1;
        let end_rot = track.bezier_segments[segment_index][3].1;

        for sub_index in 0..subdivisions {
            if segment_index == 0 && sub_index == 0 {
                continue;
            }

            let local_t = sub_index as f32 / subdivisions as f32;
            let global_t = segment_index as f32 + local_t;
            let position = curve.position(global_t);

            let center = ((position - previous_pos) / 2.0) + previous_pos;

            let length = position.distance(previous_pos);
            let interpolated_rot = start_rot.lerp(end_rot, local_t);
            // gizmos.arrow(
            //     center,
            //     center + (interpolated_rot * Vec3::Y) * 10.0,
            //     Color::RED,
            // );
            let spawn_transform = Transform::from_translation(center)
                .looking_at(previous_pos, interpolated_rot * Vec3::Y);

            // let inverted = spawn_transform.compute_affine().inverse();
            // let cur_connection_points_local = [
            //     inverted.transform_point(previous_connection_points_world[0]),
            //     inverted.transform_point(previous_connection_points_world[1]),
            //     inverted.transform_point(previous_connection_points_world[2]),
            //     inverted.transform_point(previous_connection_points_world[3]),
            // ];

            let generate_back =
                segment_index == track.bezier_segments.len() - 1 && sub_index == subdivisions - 1;

            let next_connection_points = generate_segment_mesh_new(
                &mut mesh_gen_buf,
                &mut indices,
                length,
                previous_connection_points,
                spawn_transform,
                generate_back,
                segment_index == 0 && sub_index == 1,
                gizmos,
            );
            previous_connection_points = next_connection_points;

            // let (mesh, collider, next_connection_points_local) =
            //     generate_segment_mesh(length, cur_connection_points_local);

            // let collider = Collider::from_bevy_mesh(
            //     &mesh,
            //     &ComputedColliderShape::ConvexDecomposition(VHACDParameters {
            //         ..Default::default()
            //     }),
            // );

            // previous_connection_points_world[0] =
            //     spawn_transform.transform_point(next_connection_points_local[0]);
            // previous_connection_points_world[1] =
            //     spawn_transform.transform_point(next_connection_points_local[1]);
            // previous_connection_points_world[2] =
            //     spawn_transform.transform_point(next_connection_points_local[2]);
            // previous_connection_points_world[3] =
            //     spawn_transform.transform_point(next_connection_points_local[3]);

            // another_builder.insert(collider);
            // if let Some(collider) = collider {
            //     another_builder.insert(collider);
            // }

            previous_pos = position;
        }
    }

    let positions: Vec<_> = mesh_gen_buf.vertices.iter().map(|(p, _, _)| *p).collect();
    let normals: Vec<_> = mesh_gen_buf.vertices.iter().map(|(_, n, _)| *n).collect();
    let uvs: Vec<_> = mesh_gen_buf.vertices.iter().map(|(_, _, uv)| *uv).collect();

    // mesh_gen_buf.clear();
    Some(
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
        .with_inserted_indices(Indices::U32(indices)),
    )
}

pub fn generate_segment_mesh_new(
    buffers: &mut MeshGeneratorBuffers,
    indices: &mut Vec<u32>,
    length: f32,
    connection_points: [Vec3; 4],
    segment_transform: Transform,
    generate_front: bool,
    generate_back: bool,
    gizmos: &mut Gizmos,
) -> [Vec3; 4] {
    let half_size = Vec3::new(HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, length / 2.0);
    let min = -half_size;
    let max = half_size;

    // Front is technically the back?
    let front_top_right = segment_transform.transform_point(Vec3::new(max.x, max.y, max.z));
    let front_bottom_right = segment_transform.transform_point(Vec3::new(max.x, min.y, max.z));
    let front_top_left = segment_transform.transform_point(Vec3::new(min.x, max.y, max.z));
    let front_bottom_left = segment_transform.transform_point(Vec3::new(min.x, min.y, max.z));

    let next_connection_points = [
        front_top_left,
        front_top_right,
        front_bottom_right,
        front_bottom_left,
    ];

    // let back_top_left = [min.x, max.y, min.z];
    // let back_top_right = [max.x, max.y, min.z];
    // let back_bottom_left = [min.x, min.y, min.z];
    // let back_bottom_right = [max.x, min.y, min.z];
    let back_top_left = connection_points[0];
    let back_top_right = connection_points[1];
    let back_bottom_right = connection_points[2];
    let back_bottom_left = connection_points[3];

    let top_center = (front_top_left + front_top_right + back_top_left + back_top_right) / 4.0;

    // Suppose Y-up right hand, and camera look from +Z to -Z
    // let mut indices = Vec::with_capacity(100);

    let top_faces = subdivide_faces(
        front_top_left,
        front_top_right,
        back_top_right,
        back_top_left,
        15,
    );

    for face in top_faces {
        insert_face(&mut buffers.vertices, indices, face, gizmos);
    }

    // Left
    insert_face(
        &mut buffers.vertices,
        indices,
        [
            front_top_left,
            back_top_left,
            back_bottom_left,
            front_bottom_left,
        ],
        gizmos,
    );

    // Right
    insert_face(
        &mut buffers.vertices,
        indices,
        [
            back_top_right,
            front_top_right,
            front_bottom_right,
            back_bottom_right,
        ],
        gizmos,
    );

    // Front
    if generate_front {
        insert_face(
            &mut buffers.vertices,
            indices,
            [
                front_bottom_left,
                front_bottom_right,
                front_top_right,
                front_top_left,
            ],
            gizmos,
        );
    }

    // Back
    if generate_back {
        insert_face(
            &mut buffers.vertices,
            indices,
            [
                back_top_left,
                back_top_right,
                back_bottom_right,
                back_bottom_left,
            ],
            gizmos,
        );
    }

    // Bottom
    insert_face(
        &mut buffers.vertices,
        indices,
        [
            back_bottom_left,
            back_bottom_right,
            front_bottom_right,
            front_bottom_left,
        ],
        gizmos,
    );

    // let positions: Vec<_> = buffers.vertices.iter().map(|(p, _, _)| *p).collect();
    // let normals: Vec<_> = buffers.vertices.iter().map(|(_, n, _)| *n).collect();
    // let uvs: Vec<_> = buffers.vertices.iter().map(|(_, _, uv)| *uv).collect();

    // (
    //     Mesh::new(
    //         PrimitiveTopology::TriangleList,
    //         RenderAssetUsages::default(),
    //     )
    //     .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    //     .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    //     .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    //     .with_inserted_indices(Indices::U32(indices)),
    //     // collider,
    //     next_connection_points,
    // )

    next_connection_points
}

fn generate_segment_mesh(length: f32, connection_points: [Vec3; 4]) -> (Mesh, Collider, [Vec3; 4]) {
    let half_size = Vec3::new(HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, length / 2.0);
    let min = -half_size;
    let max = half_size;

    // Front is technically the back?
    let front_top_right = Vec3::new(max.x, max.y, max.z);
    let front_bottom_right = Vec3::new(max.x, min.y, max.z);
    let front_top_left = Vec3::new(min.x, max.y, max.z);
    let front_bottom_left = Vec3::new(min.x, min.y, max.z);

    let next_connection_points = [
        front_top_left,
        front_top_right,
        front_bottom_right,
        front_bottom_left,
    ];

    // let back_top_left = [min.x, max.y, min.z];
    // let back_top_right = [max.x, max.y, min.z];
    // let back_bottom_left = [min.x, min.y, min.z];
    // let back_bottom_right = [max.x, min.y, min.z];
    let back_top_left = connection_points[0];
    let back_top_right = connection_points[1];
    let back_bottom_right = connection_points[2];
    let back_bottom_left = connection_points[3];

    let top_center = (front_top_left + front_top_right + back_top_left + back_top_right) / 4.0;

    // Suppose Y-up right hand, and camera look from +Z to -Z
    let vertices = &[
        // Front
        (front_bottom_left.to_array(), [0.0, 0.0, 1.0], [0.0, 0.0]),
        (front_bottom_right.to_array(), [0.0, 0.0, 1.0], [1.0, 0.0]),
        (front_top_right.to_array(), [0.0, 0.0, 1.0], [1.0, 1.0]),
        (front_top_left.to_array(), [0.0, 0.0, 1.0], [0.0, 1.0]),
        // Back
        (back_top_left.to_array(), [0.0, 0.0, -1.0], [1.0, 0.0]),
        (back_top_right.to_array(), [0.0, 0.0, -1.0], [0.0, 0.0]),
        (back_bottom_right.to_array(), [0.0, 0.0, -1.0], [0.0, 1.0]),
        (back_bottom_left.to_array(), [0.0, 0.0, -1.0], [1.0, 1.0]),
        // Right
        (back_bottom_right.to_array(), [1.0, 0.0, 0.0], [0.0, 0.0]),
        (back_top_right.to_array(), [1.0, 0.0, 0.0], [1.0, 0.0]),
        (front_top_right.to_array(), [1.0, 0.0, 0.0], [1.0, 1.0]),
        (front_bottom_right.to_array(), [1.0, 0.0, 0.0], [0.0, 1.0]),
        // Left
        (front_bottom_left.to_array(), [-1.0, 0.0, 0.0], [1.0, 0.0]),
        (front_top_left.to_array(), [-1.0, 0.0, 0.0], [0.0, 0.0]),
        (back_top_left.to_array(), [-1.0, 0.0, 0.0], [0.0, 1.0]),
        (back_bottom_left.to_array(), [-1.0, 0.0, 0.0], [1.0, 1.0]),
        // Top
        (back_top_right.to_array(), [0.0, 1.0, 0.0], [1.0, 0.0]),
        (back_top_left.to_array(), [0.0, 1.0, 0.0], [0.0, 0.0]),
        (front_top_left.to_array(), [0.0, 1.0, 0.0], [0.0, 1.0]),
        (front_top_right.to_array(), [0.0, 1.0, 0.0], [1.0, 1.0]),
        (top_center.to_array(), [0.0, 1.0, 0.0], [1.0, 1.0]),
        // Bottom
        (front_bottom_right.to_array(), [0.0, -1.0, 0.0], [0.0, 0.0]),
        (front_bottom_left.to_array(), [0.0, -1.0, 0.0], [1.0, 0.0]),
        (back_bottom_left.to_array(), [0.0, -1.0, 0.0], [1.0, 1.0]),
        (back_bottom_right.to_array(), [0.0, -1.0, 0.0], [0.0, 1.0]),
    ];

    let idx_front = 0;
    let idx_back = idx_front + 4;
    let idx_right = idx_back + 4;
    let idx_left = idx_right + 4;
    let idx_top = idx_left + 4;
    let idx_bottom = idx_top + 5;

    let positions: Vec<_> = vertices.iter().map(|(p, _, _)| *p).collect();
    let normals: Vec<_> = vertices.iter().map(|(_, n, _)| *n).collect();
    let uvs: Vec<_> = vertices.iter().map(|(_, _, uv)| *uv).collect();

    let indices = Indices::U32(vec![
        // Front
        // 1
        idx_front,
        idx_front + 1,
        idx_front + 2,
        // 2
        idx_front + 2,
        idx_front + 3,
        idx_front,
        // Back
        // 1
        idx_back,
        idx_back + 1,
        idx_back + 2,
        // 2
        idx_back + 2,
        idx_back + 3,
        idx_back,
        // Right
        // 1
        idx_right,
        idx_right + 1,
        idx_right + 2,
        // 2
        idx_right + 2,
        idx_right + 3,
        idx_right,
        // Left
        // 1
        idx_left,
        idx_left + 1,
        idx_left + 2,
        // 2
        idx_left + 2,
        idx_left + 3,
        idx_left,
        // Top
        // 1
        idx_top + 4,
        idx_top,
        idx_top + 1,
        // 2
        idx_top + 4,
        idx_top + 1,
        idx_top + 2,
        // 3
        idx_top + 4,
        idx_top + 2,
        idx_top + 3,
        // 4
        idx_top + 4,
        idx_top + 3,
        idx_top,
        // Bottom
        // 1
        idx_bottom,
        idx_bottom + 1,
        idx_bottom + 2,
        // 2
        idx_bottom + 2,
        idx_bottom + 3,
        idx_bottom, // bottom 2
    ]);

    let collider = Collider::compound(vec![
        (
            // Top
            Vect::ZERO,
            Rot::IDENTITY,
            Collider::triangle(
                positions[idx_top as usize + 4].into(),
                positions[idx_top as usize].into(),
                positions[idx_top as usize + 1].into(),
                // positions[17].into(),
                // positions[18].into(),
                // positions[19].into(),
            ),
        ),
        (
            // Top
            Vect::ZERO,
            Rot::IDENTITY,
            Collider::triangle(
                positions[idx_top as usize + 4].into(),
                positions[idx_top as usize + 1].into(),
                positions[idx_top as usize + 2].into(),
                // positions[19].into(),
                // positions[16].into(),
                // positions[17].into(),
            ),
        ),
        (
            // Top
            Vect::ZERO,
            Rot::IDENTITY,
            Collider::triangle(
                positions[idx_top as usize + 4].into(),
                positions[idx_top as usize + 2].into(),
                positions[idx_top as usize + 3].into(),
                // positions[19].into(),
                // positions[16].into(),
                // positions[17].into(),
            ),
        ),
        (
            // Top
            Vect::ZERO,
            Rot::IDENTITY,
            Collider::triangle(
                positions[idx_top as usize + 4].into(),
                positions[idx_top as usize + 3].into(),
                positions[idx_top as usize].into(),
                // positions[19].into(),
                // positions[16].into(),
                // positions[17].into(),
            ),
        ),
        // (
        //     // Left
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[12].into(),
        //         positions[13].into(),
        //         positions[14].into(),
        //     ),
        // ),
        // (
        //     // Left
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[14].into(),
        //         positions[15].into(),
        //         positions[12].into(),
        //     ),
        // ),
        // (
        //     // Back
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[4].into(),
        //         positions[5].into(),
        //         positions[6].into(),
        //     ),
        // ),
        // (
        //     // Back
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[6].into(),
        //         positions[7].into(),
        //         positions[4].into(),
        //     ),
        // ),
        // (
        //     // half-cross
        //     Vect::ZERO,
        //     Rot::IDENTITY,
        //     Collider::triangle(
        //         positions[12].into(),
        //         positions[13].into(),
        //         positions[17].into(),
        //     ),
        // ),
    ]);

    (
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
        .with_inserted_indices(indices),
        collider,
        next_connection_points,
    )
}

#[derive(Resource, Debug)]
pub struct RotatedSeam(pub bool);

fn toggle_seam(actions: Res<Actions>, mut seam: ResMut<RotatedSeam>) {
    if !actions.toggle_seam || !actions.is_changed() {
        return;
    }

    info!("Toggled seam {seam:?}");
    seam.0 = !seam.0;
}

fn subdivide_faces(
    top_left: Vec3,
    top_right: Vec3,
    bottom_right: Vec3,
    bottom_left: Vec3,
    subdivisions_lr: u32,
) -> Vec<[Vec3; 4]> {
    let total_faces = subdivisions_lr + 1;
    let mut faces = Vec::with_capacity(total_faces as usize);

    let top_delta = top_right - top_left;
    let bottom_delta = bottom_right - bottom_left;

    let points_to_add = subdivisions_lr;
    let step_top = top_delta / (points_to_add + 1) as f32;
    let step_bottom = bottom_delta / (points_to_add + 1) as f32;

    let mut cur_top = top_right;
    let mut cur_bottom = bottom_right;

    for _ in 0..total_faces {
        let top_right = cur_top;
        let bottom_right = cur_bottom;
        let top_left = cur_top - step_top;
        let bottom_left = cur_bottom - step_bottom;

        cur_top = top_left;
        cur_bottom = bottom_left;

        faces.push([top_left, top_right, bottom_right, bottom_left]);
    }

    faces
}

fn insert_face(
    vertices: &mut Vec<([f32; 3], [f32; 3], [f32; 2])>,
    indices: &mut Vec<u32>,
    corners: [Vec3; 4],
    gizmos: &mut Gizmos,
) {
    let idx_offset = vertices.len() as u32;

    let normal_face_1 = tri_normal(corners[0], corners[1], corners[2]);
    let normal_face_2 = tri_normal(corners[2], corners[3], corners[0]);

    let combined_normal = (normal_face_1 + normal_face_2).normalize();

    let center_1 = (corners[0] + corners[1] + corners[2]) / 3.0;
    let center_2 = (corners[2] + corners[3] + corners[0]) / 3.0;
    gizmos.arrow(center_1, center_1 + normal_face_1.normalize(), Color::PINK);
    gizmos.arrow(
        center_2,
        center_2 + normal_face_2.normalize(),
        Color::TOMATO,
    );

    vertices.push((
        corners[0].to_array(),
        combined_normal.to_array(),
        [0.0, 0.0],
    ));
    vertices.push((
        corners[1].to_array(),
        normal_face_1.normalize().to_array(),
        [0.0, 0.0],
    ));
    vertices.push((
        corners[2].to_array(),
        combined_normal.to_array(),
        [0.0, 0.0],
    ));
    vertices.push((
        corners[3].to_array(),
        normal_face_2.normalize().to_array(),
        [0.0, 0.0],
    ));

    indices.push(idx_offset);
    indices.push(idx_offset + 1);
    indices.push(idx_offset + 2);

    indices.push(idx_offset + 2);
    indices.push(idx_offset + 3);
    indices.push(idx_offset);

    // 0.0 0.0
}

fn tri_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    (b - a).cross(c - a)
}

fn default_connection_points(position: Vec3) -> [Vec3; 4] {
    let next_connection_points = [
        Vec3::new(-HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, 0.0) + position,
        Vec3::new(HALF_TRACK_WIDTH, HALF_TRACK_HEIGHT, 0.0) + position,
        Vec3::new(HALF_TRACK_WIDTH, -HALF_TRACK_HEIGHT, 0.0) + position,
        Vec3::new(-HALF_TRACK_WIDTH, -HALF_TRACK_HEIGHT, 0.0) + position,
    ];
    // let transform = Transform::IDENTITY;

    // transform.transform_point(point)
    next_connection_points
}
