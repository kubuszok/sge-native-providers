// 3D Physics — wraps Rapier3D for rigid body simulation
//
// Provides C ABI functions for:
//   - Desktop JVM via Panama FFM (java.lang.foreign)
//   - Scala Native via @extern
//
// All public functions are prefixed with sge_phys3d_ to avoid symbol collisions.
// The world state is stored in a heap-allocated PhysicsWorld struct, passed
// as an opaque *mut c_void handle. Body/collider/joint handles are Rapier's
// internal indices encoded as u64.
//
// 3D differences from 2D:
//   - Positions are [x, y, z] (3 floats)
//   - Rotations are quaternions [qx, qy, qz, qw] (4 floats)
//   - Angular velocity is a vector [wx, wy, wz] (3 floats)
//   - Additional shapes: Cylinder, Cone
//   - Motor joints have 6 DOF: LinX, LinY, LinZ, AngX, AngY, AngZ

use std::ffi::c_void;
use std::slice;
use std::sync::mpsc;

use rapier3d::prelude::*;
use rapier3d::parry::query::{DefaultQueryDispatcher, ShapeCastOptions};

// ---------------------------------------------------------------------------
// World state
// ---------------------------------------------------------------------------

struct PhysicsWorld {
    gravity: Vector,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    contact_start_buf: Vec<(u64, u64)>,
    contact_stop_buf: Vec<(u64, u64)>,
}

impl PhysicsWorld {
    fn new(gx: f32, gy: f32, gz: f32) -> Self {
        PhysicsWorld {
            gravity: Vector::new(gx, gy, gz),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            contact_start_buf: Vec::new(),
            contact_stop_buf: Vec::new(),
        }
    }

    fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
        self.contact_start_buf.clear();
        self.contact_stop_buf.clear();

        let (contact_send, contact_recv) = mpsc::channel();
        let (force_send, _force_recv) = mpsc::channel();
        let event_handler = ChannelEventCollector::new(contact_send, force_send);

        self.physics_pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &event_handler,
        );

        while let Ok(event) = contact_recv.try_recv() {
            match event {
                CollisionEvent::Started(c1, c2, _flags) => {
                    self.contact_start_buf.push((collider_handle_to_u64(c1), collider_handle_to_u64(c2)));
                }
                CollisionEvent::Stopped(c1, c2, _flags) => {
                    self.contact_stop_buf.push((collider_handle_to_u64(c1), collider_handle_to_u64(c2)));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Handle encoding/decoding
// ---------------------------------------------------------------------------

fn body_handle_to_u64(h: RigidBodyHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}
fn u64_to_body_handle(v: u64) -> RigidBodyHandle {
    RigidBodyHandle::from_raw_parts(v as u32, (v >> 32) as u32)
}
fn collider_handle_to_u64(h: ColliderHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}
fn u64_to_collider_handle(v: u64) -> ColliderHandle {
    ColliderHandle::from_raw_parts(v as u32, (v >> 32) as u32)
}
fn joint_handle_to_u64(h: ImpulseJointHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}
fn u64_to_joint_handle(v: u64) -> ImpulseJointHandle {
    ImpulseJointHandle::from_raw_parts(v as u32, (v >> 32) as u32)
}

// ---------------------------------------------------------------------------
// World lifecycle
// ---------------------------------------------------------------------------

#[no_mangle]
pub extern "C" fn sge_phys3d_create_world(gx: f32, gy: f32, gz: f32) -> *mut c_void {
    Box::into_raw(Box::new(PhysicsWorld::new(gx, gy, gz))) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_world(world: *mut c_void) {
    if !world.is_null() { drop(Box::from_raw(world as *mut PhysicsWorld)); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_step(world: *mut c_void, dt: f32) {
    (&mut *(world as *mut PhysicsWorld)).step(dt);
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_set_gravity(world: *mut c_void, gx: f32, gy: f32, gz: f32) {
    (&mut *(world as *mut PhysicsWorld)).gravity = Vector::new(gx, gy, gz);
}

/// Fills `out` with [gx, gy, gz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_get_gravity(world: *mut c_void, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    arr[0] = w.gravity.x; arr[1] = w.gravity.y; arr[2] = w.gravity.z;
}

// ---------------------------------------------------------------------------
// Rigid body lifecycle
// ---------------------------------------------------------------------------

unsafe fn create_body(world: *mut c_void, body_type: RigidBodyType,
    x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32) -> u64
{
    let w = &mut *(world as *mut PhysicsWorld);
    let quat = Rotation::from_xyzw(qx, qy, qz, qw);
    let (axis, angle) = quat.to_axis_angle();
    let body = RigidBodyBuilder::new(body_type)
        .translation(Vector::new(x, y, z))
        .rotation(axis * angle)
        .build();
    body_handle_to_u64(w.rigid_body_set.insert(body))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_dynamic_body(
    world: *mut c_void, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32
) -> u64 { create_body(world, RigidBodyType::Dynamic, x, y, z, qx, qy, qz, qw) }

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_static_body(
    world: *mut c_void, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32
) -> u64 { create_body(world, RigidBodyType::Fixed, x, y, z, qx, qy, qz, qw) }

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_kinematic_body(
    world: *mut c_void, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32
) -> u64 { create_body(world, RigidBodyType::KinematicPositionBased, x, y, z, qx, qy, qz, qw) }

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_body(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.rigid_body_set.remove(u64_to_body_handle(body),
        &mut w.island_manager, &mut w.collider_set,
        &mut w.impulse_joint_set, &mut w.multibody_joint_set, true);
}

// ---------------------------------------------------------------------------
// Body accessors
// ---------------------------------------------------------------------------

/// Fills `out` with [x, y, z].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_position(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let pos = b.translation();
        let arr = slice::from_raw_parts_mut(out, 3);
        arr[0] = pos.x; arr[1] = pos.y; arr[2] = pos.z;
    }
}

/// Fills `out` with [qx, qy, qz, qw].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_rotation(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let q = b.rotation();
        let arr = slice::from_raw_parts_mut(out, 4);
        arr[0] = q.x; arr[1] = q.y; arr[2] = q.z; arr[3] = q.w;
    }
}

/// Fills `out` with [vx, vy, vz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_linear_velocity(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let vel = b.linvel();
        let arr = slice::from_raw_parts_mut(out, 3);
        arr[0] = vel.x; arr[1] = vel.y; arr[2] = vel.z;
    }
}

/// Fills `out` with [wx, wy, wz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_angular_velocity(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let ang = b.angvel();
        let arr = slice::from_raw_parts_mut(out, 3);
        arr[0] = ang.x; arr[1] = ang.y; arr[2] = ang.z;
    }
}

// ---------------------------------------------------------------------------
// Body setters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_position(world: *mut c_void, body: u64, x: f32, y: f32, z: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_translation(Vector::new(x, y, z), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_rotation(world: *mut c_void, body: u64, qx: f32, qy: f32, qz: f32, qw: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_rotation(Rotation::from_xyzw(qx, qy, qz, qw), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_linear_velocity(world: *mut c_void, body: u64, vx: f32, vy: f32, vz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_linvel(Vector::new(vx, vy, vz), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_angular_velocity(world: *mut c_void, body: u64, wx: f32, wy: f32, wz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_angvel(Vector::new(wx, wy, wz), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_force(world: *mut c_void, body: u64, fx: f32, fy: f32, fz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.add_force(Vector::new(fx, fy, fz), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_impulse(world: *mut c_void, body: u64, ix: f32, iy: f32, iz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.apply_impulse(Vector::new(ix, iy, iz), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_torque(world: *mut c_void, body: u64, tx: f32, ty: f32, tz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.add_torque(Vector::new(tx, ty, tz), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_force_at_point(world: *mut c_void, body: u64, fx: f32, fy: f32, fz: f32, px: f32, py: f32, pz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.add_force_at_point(Vector::new(fx, fy, fz), Vector::new(px, py, pz).into(), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_impulse_at_point(world: *mut c_void, body: u64, ix: f32, iy: f32, iz: f32, px: f32, py: f32, pz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.apply_impulse_at_point(Vector::new(ix, iy, iz), Vector::new(px, py, pz).into(), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_linear_damping(world: *mut c_void, body: u64, d: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_linear_damping(d); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_linear_damping(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.linear_damping()).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_angular_damping(world: *mut c_void, body: u64, d: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_angular_damping(d); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_angular_damping(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.angular_damping()).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_gravity_scale(world: *mut c_void, body: u64, s: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_gravity_scale(s, true); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_gravity_scale(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.gravity_scale()).unwrap_or(1.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_awake(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| !b.is_sleeping() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_wake_up(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.wake_up(true); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_sleep(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.sleep(); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_fixed_rotation(world: *mut c_void, body: u64, fixed: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.lock_rotations(fixed != 0, true); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_enable_ccd(world: *mut c_void, body: u64, enable: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.enable_ccd(enable != 0); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_ccd_enabled(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.is_ccd_enabled() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_enabled(world: *mut c_void, body: u64, e: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_enabled(e != 0); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_enabled(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.is_enabled() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_dominance_group(world: *mut c_void, body: u64, g: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_dominance_group(g as i8); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_dominance_group(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.dominance_group() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_mass(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.mass()).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_recompute_mass_properties(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.recompute_mass_properties_from_colliders(&w.collider_set);
    }
}

// ---------------------------------------------------------------------------
// Collider creation (3D shapes)
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_sphere_collider(world: *mut c_void, body: u64, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::ball(radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_box_collider(world: *mut c_void, body: u64, hx: f32, hy: f32, hz: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::cuboid(hx, hy, hz).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_capsule_collider(world: *mut c_void, body: u64, half_height: f32, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::capsule_y(half_height, radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_cylinder_collider(world: *mut c_void, body: u64, half_height: f32, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::cylinder(half_height, radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_cone_collider(world: *mut c_void, body: u64, half_height: f32, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::cone(half_height, radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_convex_hull_collider(world: *mut c_void, body: u64, vertices: *const f32, vertex_count: i32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let verts = slice::from_raw_parts(vertices, (vertex_count * 3) as usize);
    let points: Vec<Vector> = (0..vertex_count as usize)
        .map(|i| Vector::new(verts[i*3], verts[i*3+1], verts[i*3+2]))
        .collect();
    let c = ColliderBuilder::convex_hull(&points.iter().map(|v| (*v).into()).collect::<Vec<_>>())
        .unwrap_or_else(|| ColliderBuilder::ball(0.1))
        .build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_trimesh_collider(
    world: *mut c_void, body: u64,
    vertices: *const f32, vertex_count: i32,
    indices: *const u32, index_count: i32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let verts = slice::from_raw_parts(vertices, (vertex_count * 3) as usize);
    let idxs  = slice::from_raw_parts(indices, index_count as usize);
    let points: Vec<_> = (0..vertex_count as usize)
        .map(|i| Vector::new(verts[i*3], verts[i*3+1], verts[i*3+2]).into())
        .collect();
    let tris: Vec<[u32; 3]> = idxs.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
    let c = ColliderBuilder::trimesh(points, tris).unwrap().build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_collider(world: *mut c_void, collider: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.collider_set.remove(u64_to_collider_handle(collider), &mut w.island_manager, &mut w.rigid_body_set, true);
}

// ---------------------------------------------------------------------------
// Collider properties
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_density(world: *mut c_void, c: u64, v: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_density(v); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_friction(world: *mut c_void, c: u64, v: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_friction(v); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_restitution(world: *mut c_void, c: u64, v: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_restitution(v); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_sensor(world: *mut c_void, c: u64, v: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_sensor(v != 0); }
}

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_fixed_joint(world: *mut c_void, body1: u64, body2: u64) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let joint = FixedJointBuilder::new().build();
    joint_handle_to_u64(w.impulse_joint_set.insert(u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_rope_joint(world: *mut c_void, body1: u64, body2: u64, max_dist: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let joint = RopeJointBuilder::new(max_dist).build();
    joint_handle_to_u64(w.impulse_joint_set.insert(u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_joint(world: *mut c_void, joint: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.impulse_joint_set.remove(u64_to_joint_handle(joint), true);
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

/// Ray cast (closest hit). Fills `out` with [hitX, hitY, hitZ, normalX, normalY, normalZ, toi, colliderLo, colliderHi] = 9 floats.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_ray_cast(
    world: *mut c_void, ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32, max_dist: f32, out: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let ray = Ray::new(Vector::new(ox, oy, oz).into(), Vector::new(dx, dy, dz));
    let qp = w.broad_phase.as_query_pipeline(&DefaultQueryDispatcher, &w.rigid_body_set, &w.collider_set, QueryFilter::default());
    if let Some((handle, intersection)) = qp.cast_ray_and_get_normal(&ray, max_dist, true) {
        let hit = ray.point_at(intersection.time_of_impact);
        let arr = slice::from_raw_parts_mut(out, 9);
        arr[0] = hit.x; arr[1] = hit.y; arr[2] = hit.z;
        arr[3] = intersection.normal.x; arr[4] = intersection.normal.y; arr[5] = intersection.normal.z;
        arr[6] = intersection.time_of_impact;
        let ch = collider_handle_to_u64(handle);
        arr[7] = f32::from_bits(ch as u32);
        arr[8] = f32::from_bits((ch >> 32) as u32);
        1
    } else { 0 }
}

// ---------------------------------------------------------------------------
// Contact events
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_poll_contact_start_events(world: *mut c_void, out1: *mut u64, out2: *mut u64, max: i32) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = slice::from_raw_parts_mut(out1, max as usize);
    let c2 = slice::from_raw_parts_mut(out2, max as usize);
    let count = w.contact_start_buf.len().min(max as usize);
    for i in 0..count { c1[i] = w.contact_start_buf[i].0; c2[i] = w.contact_start_buf[i].1; }
    count as i32
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_poll_contact_stop_events(world: *mut c_void, out1: *mut u64, out2: *mut u64, max: i32) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = slice::from_raw_parts_mut(out1, max as usize);
    let c2 = slice::from_raw_parts_mut(out2, max as usize);
    let count = w.contact_stop_buf.len().min(max as usize);
    for i in 0..count { c1[i] = w.contact_stop_buf[i].0; c2[i] = w.contact_stop_buf[i].1; }
    count as i32
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn world_create_destroy() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, -9.81, 0.0);
            assert!(!world.is_null());
            let mut g = [0.0f32; 3];
            sge_phys3d_world_get_gravity(world, g.as_mut_ptr());
            assert!((g[1] - (-9.81)).abs() < 1e-5);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn body_falls_under_gravity() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, -9.81, 0.0);
            let body = sge_phys3d_create_dynamic_body(world, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _col = sge_phys3d_create_box_collider(world, body, 0.5, 0.5, 0.5);
            let mut pos = [0.0f32; 3];
            sge_phys3d_body_get_position(world, body, pos.as_mut_ptr());
            let y0 = pos[1];
            for _ in 0..60 { sge_phys3d_world_step(world, 1.0 / 60.0); }
            sge_phys3d_body_get_position(world, body, pos.as_mut_ptr());
            assert!(pos[1] < y0, "body should fall: y0={}, y={}", y0, pos[1]);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn static_body_stays() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, -9.81, 0.0);
            let body = sge_phys3d_create_static_body(world, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 1.0);
            let _col = sge_phys3d_create_box_collider(world, body, 1.0, 1.0, 1.0);
            for _ in 0..60 { sge_phys3d_world_step(world, 1.0 / 60.0); }
            let mut pos = [0.0f32; 3];
            sge_phys3d_body_get_position(world, body, pos.as_mut_ptr());
            assert!((pos[0] - 5.0).abs() < 1e-5);
            assert!((pos[1] - 5.0).abs() < 1e-5);
            assert!((pos[2] - 5.0).abs() < 1e-5);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn raycast_3d() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, 0.0, 0.0);
            let body = sge_phys3d_create_static_body(world, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _col = sge_phys3d_create_box_collider(world, body, 1.0, 1.0, 1.0);
            sge_phys3d_world_step(world, 1.0 / 60.0);
            let mut out = [0.0f32; 9];
            let hit = sge_phys3d_ray_cast(world, 0.0, 10.0, 0.0, 0.0, -1.0, 0.0, 100.0, out.as_mut_ptr());
            assert_eq!(hit, 1, "ray should hit the box");
            assert!((out[1] - 1.0).abs() < 0.1, "hit Y should be ~1.0, got {}", out[1]);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn rope_joint_3d() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, 0.0, 0.0);
            let b1 = sge_phys3d_create_dynamic_body(world, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _c1 = sge_phys3d_create_box_collider(world, b1, 0.5, 0.5, 0.5);
            let b2 = sge_phys3d_create_dynamic_body(world, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _c2 = sge_phys3d_create_box_collider(world, b2, 0.5, 0.5, 0.5);
            let _joint = sge_phys3d_create_rope_joint(world, b1, b2, 5.0);
            sge_phys3d_world_step(world, 1.0 / 60.0);
            sge_phys3d_destroy_world(world);
        }
    }
}
