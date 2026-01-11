const std = @import("std");
const math = @import("math.zig");
const world = @import("world.zig");

const Vec3 = math.Vec3;
const World = world.World;
const HullType = world.HullType;
const Collision = world.Collision;
const EPSILON = math.EPSILON;
const STEP_HEIGHT = math.STEP_HEIGHT;
const GROUND_SNAP = math.GROUND_SNAP;
const COLLISION_MARGIN: f32 = 0.02;

pub const Move = struct { pos: Vec3, collision: ?Collision = null };

pub const Config = struct {
    gravity: f32 = 12.0,
    jump_velocity: f32 = 4.0,
    max_speed: f32 = 4.0,
    air_speed: f32 = 0.7,
    acceleration: f32 = 70.0,
    friction: f32 = 5.0,
    jump_sound_duration: f32 = 0.15,
};

pub const Transform = struct { pos: Vec3 = Vec3.zero() };
pub const Physics = struct { vel: Vec3 = Vec3.zero(), on_ground: bool = false, crouching: bool = false };
pub const Input = struct { 
    yaw: f32 = 0, pitch: f32 = 0, mdx: f32 = 0, mdy: f32 = 0,
    keys: packed struct { w: bool = false, a: bool = false, s: bool = false, d: bool = false, sp: bool = false } = .{}, 
    lock: bool = false 
};
pub const Audio = struct { timer: f32 = 0, active: bool = false };

const default_config = Config{};

pub fn update(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, collision_world: *const World, dt: f32) void {
    updateWithConfig(transforms, physics, inputs, audios, collision_world, dt, default_config);
}

pub fn updateWithConfig(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, collision_world: *const World, dt: f32, config: Config) void {
    for (transforms, physics, inputs, audios) |*t, *p, *i, *a| {
        const hull_type: HullType = if (p.crouching) .crouching else .standing;
        
        // Ground check with slope limit
        p.on_ground = isOnGround(collision_world, t.pos, hull_type);
        
        // Physics
        if (!p.on_ground) p.vel.data[1] -= config.gravity * dt;
        if (i.keys.sp and p.on_ground) { 
            p.vel.data[1] = config.jump_velocity; 
            a.timer = config.jump_sound_duration; 
            a.active = true; 
        }
        
        // Movement
        const input_dir = getInputDirection(i.*);
        accelerate(&p.vel, input_dir, p.on_ground, dt, config);
        if (p.on_ground) applyFriction(&p.vel, dt, config.friction);
        
        // Move and handle collisions
        const move_result = move(collision_world, t.pos, Vec3.scale(p.vel, dt), hull_type);
        t.pos = move_result.pos;
        
        if (move_result.collision) |collision| {
            const into_surface = Vec3.dot(p.vel, collision.normal);
            if (into_surface < 0) {
                p.vel = Vec3.sub(p.vel, Vec3.scale(collision.normal, into_surface));
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

fn accelerate(vel: *Vec3, wish_dir: Vec3, on_ground: bool, dt: f32, config: Config) void {
    const len = @sqrt(wish_dir.data[0] * wish_dir.data[0] + wish_dir.data[2] * wish_dir.data[2]);
    if (len < 0.001) return;
    
    const wish = Vec3.scale(wish_dir, 1.0 / len);
    const max_vel = if (on_ground) config.max_speed * len else @min(config.max_speed * len, config.air_speed);
    const current_vel = Vec3.dot(Vec3.new(vel.data[0], 0, vel.data[2]), wish);
    const accel = @min(config.acceleration * dt, @max(0, max_vel - current_vel));
    
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

fn isOnGround(world_collision: *const World, pos: Vec3, hull_type: HullType) bool {
    const test_pos = Vec3.new(pos.data[0], pos.data[1] - GROUND_SNAP, pos.data[2]);
    if (world_collision.check(test_pos, hull_type)) |collision| {
        return collision.normal.data[1] > 0.707; // 45 degree slope limit
    }
    return false;
}

pub fn move(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) Move {
    if (Vec3.length(delta) < EPSILON) return .{ .pos = start };
    
    // Try direct movement
    if (world_collision.check(Vec3.add(start, delta), hull_type) == null) {
        return .{ .pos = Vec3.add(start, delta) };
    }
    
    // Try step-up
    if (tryStepUp(world_collision, start, delta, hull_type)) |step_result| {
        return step_result;
    }
    
    // Slide movement
    return slideMove(world_collision, start, delta, hull_type);
}

fn tryStepUp(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) ?Move {
    const horizontal = Vec3.new(delta.data[0], 0, delta.data[2]);
    if (Vec3.length(horizontal) < EPSILON) return null;
    
    const step_up = Vec3.add(Vec3.add(start, horizontal), Vec3.new(0, STEP_HEIGHT, 0));
    if (world_collision.check(step_up, hull_type) != null) return null;
    
    const drop_trace = traceMove(world_collision, step_up, Vec3.new(0, -(STEP_HEIGHT + GROUND_SNAP), 0), hull_type);
    const final_pos = Vec3.add(drop_trace.end_pos, Vec3.new(0, delta.data[1], 0));
    
    return if (world_collision.check(final_pos, hull_type) == null) 
        .{ .pos = final_pos } else .{ .pos = drop_trace.end_pos };
}

fn slideMove(world_collision: *const World, start: Vec3, velocity: Vec3, hull_type: HullType) Move {
    var pos = start;
    var vel = velocity;
    var remaining_time: f32 = 1.0;
    var first_collision: ?Collision = null;
    var planes: [3]Vec3 = undefined;
    var num_planes: u32 = 0;
    
    for (0..3) |bump| {
        if (Vec3.length(vel) < EPSILON or remaining_time <= 0.001) break;
        
        const trace = traceMove(world_collision, pos, Vec3.scale(vel, remaining_time), hull_type);
        pos = trace.end_pos;
        
        if (trace.collision) |collision| {
            if (first_collision == null) first_collision = collision;
            remaining_time *= (1.0 - trace.time);
            
            const into_surface = Vec3.dot(vel, collision.normal);
            if (into_surface >= 0) break;
            
            // Store and clip against collision planes
            if (num_planes < planes.len) {
                planes[num_planes] = collision.normal;
                num_planes += 1;
            }
            
            for (planes[0..num_planes]) |plane| {
                const dot = Vec3.dot(vel, plane);
                if (dot < 0) vel = Vec3.sub(vel, Vec3.scale(plane, dot));
            }
            
            vel = Vec3.scale(vel, 0.95 - (@as(f32, @floatFromInt(bump)) * 0.05));
        } else break;
    }
    
    return .{ .pos = pos, .collision = first_collision };
}

const TraceResult = struct { end_pos: Vec3, collision: ?Collision, time: f32 };

fn traceMove(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) TraceResult {
    const move_length = Vec3.length(delta);
    if (move_length < EPSILON) return .{ .end_pos = start, .collision = null, .time = 1.0 };
    
    var best_time: f32 = 1.0;
    var collision: ?Collision = null;
    
    // Sample-based collision detection
    const samples = @min(16, @max(4, @as(u32, @intFromFloat(move_length * 10))));
    for (0..samples) |i| {
        const t = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(samples - 1));
        if (world_collision.check(Vec3.add(start, Vec3.scale(delta, t)), hull_type)) |hit| {
            best_time = binarySearch(world_collision, start, delta, hull_type, 
                                   if (i > 0) @as(f32, @floatFromInt(i - 1)) / @as(f32, @floatFromInt(samples - 1)) else 0.0, t);
            collision = hit;
            break;
        }
    }
    
    // Apply margin
    if (collision != null) {
        best_time = @max(0, best_time - COLLISION_MARGIN / move_length);
    }
    
    return .{ .end_pos = Vec3.add(start, Vec3.scale(delta, best_time)), .collision = collision, .time = best_time };
}

fn binarySearch(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType, low: f32, high: f32) f32 {
    var l = low;
    var h = high;
    for (0..8) |_| {
        const mid = (l + h) * 0.5;
        if (world_collision.check(Vec3.add(start, Vec3.scale(delta, mid)), hull_type) != null) {
            h = mid;
        } else {
            l = mid;
        }
    }
    return l;
}

