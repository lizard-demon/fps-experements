const std = @import("std");
const math = @import("math.zig");
const bvh = @import("bvh.zig");
const Vec3 = math.Vec3;
const AABB = bvh.AABB;

// Ultra-simple Quake movement - 90% feel, 20% code
pub fn quake_movement(
    pos: *Vec3,
    vel: *Vec3,
    on_ground: *bool,
    forward: f32,
    strafe: f32,
    jump: bool,
    yaw: f32,
    dt: f32,
    world_collision_fn: fn(AABB) bool
) void {
    const size = Vec3.new(0.98, 1.8, 0.98);
    
    // Gravity
    vel.data[1] -= 12.0 * dt;
    
    // Jump
    if (jump and on_ground.*) {
        vel.data[1] = 4.0;
    }
    
    // Movement
    var dir = Vec3.zero();
    if (strafe != 0) dir = Vec3.add(dir, Vec3.scale(Vec3.new(@cos(yaw), 0, @sin(yaw)), strafe));
    if (forward != 0) dir = Vec3.add(dir, Vec3.scale(Vec3.new(@sin(yaw), 0, -@cos(yaw)), forward));
    
    const len = @sqrt(dir.data[0] * dir.data[0] + dir.data[2] * dir.data[2]);
    if (len > 0.001) {
        const wish = Vec3.scale(dir, 1.0 / len);
        const speed = if (on_ground.*) 4.0 * len else @min(4.0 * len, 0.7);
        const current = Vec3.dot(Vec3.new(vel.data[0], 0, vel.data[2]), wish);
        const add = @max(0, speed - current);
        if (add > 0) {
            const accel = @min(70.0 * dt, add);
            vel.data[0] += wish.data[0] * accel;
            vel.data[2] += wish.data[2] * accel;
        }
    }
    
    // Friction
    if (on_ground.*) {
        const speed = @sqrt(vel.data[0] * vel.data[0] + vel.data[2] * vel.data[2]);
        if (speed > 0.1) {
            const factor = @max(0, speed - @max(speed, 0.1) * 5.0 * dt) / speed;
            vel.data[0] *= factor;
            vel.data[2] *= factor;
        } else {
            vel.data[0] = 0;
            vel.data[2] = 0;
        }
    }
    
    // Move and collide
    const result = move(world_collision_fn, pos.*, vel.*, size, dt);
    pos.* = result.pos;
    vel.* = result.vel;
    on_ground.* = result.hit and vel.data[1] <= 0.01 and onGround(world_collision_fn, pos.*, size);
}

// Ultra-simple collision - just slide along surfaces
fn move(
    world_collision_fn: fn(AABB) bool,
    pos: Vec3,
    vel: Vec3,
    size: Vec3,
    dt: f32
) struct { pos: Vec3, vel: Vec3, hit: bool } {
    var p = pos;
    var v = vel;
    var hit = false;
    
    // Try each axis separately - simple and effective
    inline for (0..3) |axis| {
        const old_pos = p.data[axis];
        p.data[axis] += v.data[axis] * dt;
        
        const bbox = AABB{
            .min = Vec3.sub(p, Vec3.scale(size, 0.5)),
            .max = Vec3.add(p, Vec3.scale(size, 0.5))
        };
        
        if (world_collision_fn(bbox)) {
            p.data[axis] = old_pos;
            v.data[axis] = 0;
            hit = true;
        }
    }
    
    return .{ .pos = p, .vel = v, .hit = hit };
}

// Simple ground check
fn onGround(world_collision_fn: fn(AABB) bool, pos: Vec3, size: Vec3) bool {
    const test_pos = Vec3.new(pos.data[0], pos.data[1] - 0.01, pos.data[2]);
    const bbox = AABB{
        .min = Vec3.sub(test_pos, Vec3.scale(size, 0.5)),
        .max = Vec3.add(test_pos, Vec3.scale(size, 0.5))
    };
    return world_collision_fn(bbox);
}