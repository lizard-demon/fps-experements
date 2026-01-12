const std = @import("std");
const math = @import("../lib/math.zig");
const collision = @import("collision.zig");
const config = @import("../lib/config.zig");

const Vec3 = math.Vec3;
const World = collision.World;
const HullType = collision.HullType;
const Collision = collision.Collision;
const EPSILON = math.EPSILON;
const GROUND_SNAP = math.GROUND_SNAP;

pub const Move = struct { pos: Vec3, hit: ?collision.Collision = null };

pub const Config = struct {
    gravity: f32 = config.Physics.Movement.gravity,
    jump_velocity: f32 = config.Physics.Movement.jump_velocity,
    max_speed: f32 = config.Physics.Movement.max_speed,
    air_speed: f32 = config.Physics.Movement.air_speed,
    acceleration: f32 = config.Physics.Movement.acceleration,
    friction: f32 = config.Physics.Movement.friction,
    jump_sound_duration: f32 = config.Audio.jump_sound_duration,
};

pub const Transform = struct { pos: Vec3 = Vec3.zero() };
pub const Physics = struct { vel: Vec3 = Vec3.zero(), on_ground: bool = false };
pub const Input = struct { 
    yaw: f32 = 0, pitch: f32 = 0, mdx: f32 = 0, mdy: f32 = 0,
    keys: packed struct { w: bool = false, a: bool = false, s: bool = false, d: bool = false, sp: bool = false } = .{}, 
    lock: bool = false 
};
pub const Audio = struct { timer: f32 = 0, active: bool = false };

pub fn update(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, collision_world: *const World, dt: f32) void {
    const physics_config = Config{};
    for (transforms, physics, inputs, audios) |*t, *p, *i, *a| {
        // Ground check and physics
        p.on_ground = isOnGround(collision_world, t.pos);
        if (!p.on_ground) p.vel.data[1] -= physics_config.gravity * dt;
        if (i.keys.sp and p.on_ground) { 
            p.vel.data[1] = physics_config.jump_velocity; 
            a.timer = physics_config.jump_sound_duration; 
            a.active = true; 
        }
        
        // Movement
        const input_dir = getInputDirection(i.*);
        accelerate(&p.vel, input_dir, p.on_ground, dt, physics_config);
        if (p.on_ground) applyFriction(&p.vel, dt, physics_config.friction);
        
        // Move and handle collisions
        const move_result = move(collision_world, t.pos, Vec3.scale(p.vel, dt));
        t.pos = move_result.pos;
        
        if (move_result.hit) |hit_result| {
            const into_surface = Vec3.dot(p.vel, hit_result.normal);
            if (into_surface < 0) {
                p.vel = Vec3.sub(p.vel, Vec3.scale(hit_result.normal, into_surface));
            }
        }
    }
}

fn getInputDirection(input: Input) Vec3 {
    const fwd: f32 = if (input.keys.w) 1 else if (input.keys.s) -1 else 0;
    const side: f32 = if (input.keys.d) 1 else if (input.keys.a) -1 else 0;
    const fwd_vec = Vec3.new(@sin(input.yaw), 0, -@cos(input.yaw));
    const side_vec = Vec3.new(@cos(input.yaw), 0, @sin(input.yaw));
    return Vec3.add(Vec3.scale(fwd_vec, fwd), Vec3.scale(side_vec, side));
}

fn accelerate(vel: *Vec3, wish_dir: Vec3, on_ground: bool, dt: f32, physics_config: Config) void {
    const len = @sqrt(wish_dir.data[0] * wish_dir.data[0] + wish_dir.data[2] * wish_dir.data[2]);
    if (len < 0.001) return;
    
    const wish = Vec3.scale(wish_dir, 1.0 / len);
    const max_vel = if (on_ground) physics_config.max_speed * len else @min(physics_config.max_speed * len, physics_config.air_speed);
    const current_vel = Vec3.dot(Vec3.new(vel.data[0], 0, vel.data[2]), wish);
    const accel = @min(physics_config.acceleration * dt, @max(0, max_vel - current_vel));
    
    vel.data[0] += wish.data[0] * accel;
    vel.data[2] += wish.data[2] * accel;
}

fn applyFriction(vel: *Vec3, dt: f32, friction: f32) void {
    const speed = @sqrt(vel.data[0] * vel.data[0] + vel.data[2] * vel.data[2]);
    if (speed > 0.1) {
        const factor = @max(0, speed - speed * friction * dt) / speed;
        vel.data[0] *= factor; 
        vel.data[2] *= factor;
    } else { 
        vel.data[0] = 0; 
        vel.data[2] = 0; 
    }
}

fn isOnGround(world_collision: *const World, pos: Vec3) bool {
    const test_pos = Vec3.new(pos.data[0], pos.data[1] - GROUND_SNAP, pos.data[2]);
    const capsule = collision.Capsule.fromHull(test_pos, .standing);
    if (world_collision.checkCapsule(capsule)) |hit_result| {
        return hit_result.normal.data[1] > config.Physics.Collision.slope_limit;
    }
    return false;
}

pub fn move(world_collision: *const World, start: Vec3, delta: Vec3) Move {
    if (Vec3.length(delta) < EPSILON) return .{ .pos = start };
    
    // Try direct movement
    const end_capsule = collision.Capsule.fromHull(Vec3.add(start, delta), .standing);
    if (world_collision.checkCapsule(end_capsule) == null) {
        return .{ .pos = Vec3.add(start, delta) };
    }
    
    // Slide movement
    return slide(world_collision, start, delta);
}

const TraceResult = struct { end_pos: Vec3, hit: ?collision.Collision, time: f32 };

fn trace(world_collision: *const World, start: Vec3, delta: Vec3) TraceResult {
    const move_length = Vec3.length(delta);
    if (move_length < EPSILON) return .{ .end_pos = start, .hit = null, .time = 1.0 };
    
    var best_time: f32 = 1.0;
    var hit: ?collision.Collision = null;
    
    const samples = @min(config.Physics.Collision.trace_samples_max, @max(config.Physics.Collision.trace_samples_min, @as(u32, @intFromFloat(move_length * config.Physics.Collision.trace_samples_multiplier))));
    for (0..samples) |i| {
        const t = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(samples - 1));
        const test_pos = Vec3.add(start, Vec3.scale(delta, t));
        const capsule = collision.Capsule.fromHull(test_pos, .standing);
        
        if (world_collision.checkCapsule(capsule)) |hit_result| {
            best_time = binarySearch(world_collision, start, delta, 
                                   if (i > 0) @as(f32, @floatFromInt(i - 1)) / @as(f32, @floatFromInt(samples - 1)) else 0.0, t);
            hit = hit_result;
            break;
        }
    }
    
    if (hit != null) {
        best_time = @max(0, best_time - config.Physics.Collision.margin / move_length);
    }
    
    return .{ .end_pos = Vec3.add(start, Vec3.scale(delta, best_time)), .hit = hit, .time = best_time };
}

fn slide(world_collision: *const World, start: Vec3, velocity: Vec3) Move {
    var pos = start;
    var vel = velocity;
    var remaining_time: f32 = 1.0;
    var first_hit: ?collision.Collision = null;
    var planes: [config.Physics.Collision.max_slide_iterations]Vec3 = undefined;
    var num_planes: u32 = 0;
    
    for (0..config.Physics.Collision.max_slide_iterations) |bump| {
        if (Vec3.length(vel) < EPSILON or remaining_time <= 0.001) break;
        
        const result = trace(world_collision, pos, Vec3.scale(vel, remaining_time));
        pos = result.end_pos;
        
        if (result.hit) |hit_result| {
            if (first_hit == null) first_hit = hit_result;
            remaining_time *= (1.0 - result.time);
            
            const into_surface = Vec3.dot(vel, hit_result.normal);
            if (into_surface >= 0) break;
            
            if (num_planes < planes.len) {
                planes[num_planes] = hit_result.normal;
                num_planes += 1;
            }
            
            for (planes[0..num_planes]) |plane| {
                const dot = Vec3.dot(vel, plane);
                if (dot < 0) vel = Vec3.sub(vel, Vec3.scale(plane, dot));
            }
            
            vel = Vec3.scale(vel, config.Physics.Collision.slide_damping - (@as(f32, @floatFromInt(bump)) * config.Physics.Collision.slide_damping_step));
        } else break;
    }
    
    return .{ .pos = pos, .hit = first_hit };
}

fn binarySearch(world_collision: *const World, start: Vec3, delta: Vec3, low: f32, high: f32) f32 {
    var l = low;
    var h = high;
    for (0..config.Physics.Collision.binary_search_iterations) |_| {
        const mid = (l + h) * 0.5;
        const test_pos = Vec3.add(start, Vec3.scale(delta, mid));
        const capsule = collision.Capsule.fromHull(test_pos, .standing);
        
        if (world_collision.checkCapsule(capsule) != null) {
            h = mid;
        } else {
            l = mid;
        }
    }
    return l;
}

