const std = @import("std");
const math = @import("math.zig");
const world = @import("world.zig");

const Vec3 = math.Vec3;
const AABB = math.AABB;
const World = world.World;
const HullType = world.HullType;
const CollisionResult = world.CollisionResult;
const EPSILON = math.EPSILON;
const STEP_HEIGHT = math.STEP_HEIGHT;
const GROUND_SNAP = math.GROUND_SNAP;

// Movement result
pub const MoveResult = struct {
    pos: Vec3,
    hit: bool = false,
};

// Physics configuration - all tunable constants in one place
pub const PhysicsConfig = struct {
    gravity: f32 = 12.0,           // Downward acceleration (units/s²)
    jump_velocity: f32 = 4.0,      // Initial upward velocity when jumping (units/s)
    max_speed: f32 = 4.0,          // Maximum horizontal movement speed (units/s)
    air_speed: f32 = 0.7,          // Maximum speed when airborne (units/s)
    acceleration: f32 = 70.0,      // Horizontal acceleration (units/s²)
    friction: f32 = 5.0,           // Ground friction coefficient
    wall_friction: f32 = 2.0,      // Wall sliding friction coefficient
    jump_sound_duration: f32 = 0.15, // Duration of jump sound effect (seconds)
    
    // Player collision box size
    player_size: Vec3 = Vec3{ .data = .{ 0.98, 1.8, 0.98 } },
};

// Component and input types
pub const Transform = struct { pos: Vec3 = Vec3.zero() };

pub const Physics = struct {
    vel: Vec3 = Vec3.zero(),
    on_ground: bool = false,
    crouching: bool = false, // Add crouching state
};

pub const Input = struct { 
    yaw: f32 = 0, pitch: f32 = 0, mdx: f32 = 0, mdy: f32 = 0,
    keys: packed struct { w: bool = false, a: bool = false, s: bool = false, d: bool = false, sp: bool = false } = .{}, 
    lock: bool = false 
};

pub const Audio = struct { timer: f32 = 0, active: bool = false };

// Default physics configuration
pub const default_config = PhysicsConfig{};

pub fn update(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, collision_world: *const World, dt: f32) void {
    updateWithConfig(transforms, physics, inputs, audios, collision_world, dt, default_config);
}

pub fn updateWithConfig(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, collision_world: *const World, dt: f32, config: PhysicsConfig) void {
    for (transforms, physics, inputs, audios) |*t, *p, *i, *a| {
        const fwd: f32 = if (i.keys.w) 1 else if (i.keys.s) -1 else 0;
        const side: f32 = if (i.keys.d) 1 else if (i.keys.a) -1 else 0;
        const jump = i.keys.sp;
        
        // Determine hull type based on crouching state
        const hull_type: HullType = if (p.crouching) .crouching else .standing;
        
        // Gravity
        p.vel.data[1] -= config.gravity * dt;
        
        // Jump
        if (jump and p.on_ground) { 
            p.vel.data[1] = config.jump_velocity; 
            a.timer = config.jump_sound_duration; 
            a.active = true; 
        }
        
        // Movement direction
        var dir = Vec3.zero();
        if (side != 0) dir = Vec3.add(dir, Vec3.scale(Vec3.new(@cos(i.yaw), 0, @sin(i.yaw)), side));
        if (fwd != 0) dir = Vec3.add(dir, Vec3.scale(Vec3.new(@sin(i.yaw), 0, -@cos(i.yaw)), fwd));
        
        // Acceleration
        const len = @sqrt(dir.data[0] * dir.data[0] + dir.data[2] * dir.data[2]);
        if (len > 0.001) {
            const wish = Vec3.scale(dir, 1.0 / len);
            const speed = if (p.on_ground) config.max_speed * len else @min(config.max_speed * len, config.air_speed);
            const current = Vec3.dot(Vec3.new(p.vel.data[0], 0, p.vel.data[2]), wish);
            const add = @max(0, speed - current);
            if (add > 0) {
                const accel = @min(config.acceleration * dt, add);
                p.vel.data[0] += wish.data[0] * accel;
                p.vel.data[2] += wish.data[2] * accel;
            }
        }
        
        // Friction
        if (p.on_ground) {
            const speed = @sqrt(p.vel.data[0] * p.vel.data[0] + p.vel.data[2] * p.vel.data[2]);
            if (speed > 0.1) {
                const factor = @max(0, speed - @max(speed, 0.1) * config.friction * dt) / speed;
                p.vel.data[0] *= factor; 
                p.vel.data[2] *= factor;
            } else { 
                p.vel.data[0] = 0; 
                p.vel.data[2] = 0; 
            }
        }
        
        // Movement with collision using new point-based system
        const delta = Vec3.scale(p.vel, dt);
        const move_result = movePlayerPoint(collision_world, t.pos, delta, hull_type);
        
        // Update position
        t.pos = move_result.pos;
        
        // Check ground state
        p.on_ground = isOnGroundPoint(collision_world, t.pos, hull_type);
        
        // Improved collision response for surfing
        if (move_result.hit) {
            // Get surface normal at collision point
            if (collision_world.getPointCollisionNormal(t.pos, hull_type)) |surface_normal| {
                applySurfingCollisionResponse(&p.vel, surface_normal, dt, config);
            }
        }
    }
}

// Modern point-based collision system using expanded geometry
pub fn movePlayerPoint(collision_world: *const World, start: Vec3, delta: Vec3, hull_type: HullType) MoveResult {
    if (Vec3.length(delta) < EPSILON) return .{ .pos = start };
    
    return tryDirectPoint(collision_world, start, delta, hull_type) orelse 
           tryStepPoint(collision_world, start, delta, hull_type) orelse 
           trySlidePoint(collision_world, start, delta, hull_type) orelse 
           .{ .pos = start, .hit = true };
}

fn tryDirectPoint(collision_world: *const World, pos: Vec3, delta: Vec3, hull_type: HullType) ?MoveResult {
    const target = Vec3.add(pos, delta);
    return if (!collision_world.testPointCollision(target, hull_type)) .{ .pos = target } else null;
}

fn tryStepPoint(collision_world: *const World, pos: Vec3, delta: Vec3, hull_type: HullType) ?MoveResult {
    const horizontal = Vec3.new(delta.data[0], 0, delta.data[2]);
    if (Vec3.length(horizontal) < EPSILON) return null;
    
    const step_pos = Vec3.add(Vec3.add(pos, horizontal), Vec3.new(0, STEP_HEIGHT, 0));
    
    if (!collision_world.testPointCollision(step_pos, hull_type)) {
        const ground_pos = findGroundPoint(collision_world, step_pos, hull_type);
        const final_pos = Vec3.new(ground_pos.data[0], ground_pos.data[1] + delta.data[1], ground_pos.data[2]);
        return if (!collision_world.testPointCollision(final_pos, hull_type)) .{ .pos = final_pos } else .{ .pos = ground_pos };
    }
    return null;
}

fn trySlidePoint(collision_world: *const World, pos: Vec3, delta: Vec3, hull_type: HullType) ?MoveResult {
    var current_pos = pos;
    var remaining_delta = delta;
    var moved = false;
    
    for (0..3) |_| {
        if (Vec3.length(remaining_delta) < EPSILON) break;
        
        const target_pos = Vec3.add(current_pos, remaining_delta);
        
        if (!collision_world.testPointCollision(target_pos, hull_type)) {
            current_pos = target_pos;
            moved = true;
            break;
        }
        
        if (collision_world.getPointCollisionNormal(target_pos, hull_type)) |surface_normal| {
            const dot_product = Vec3.dot(remaining_delta, surface_normal);
            const slide_delta = Vec3.sub(remaining_delta, Vec3.scale(surface_normal, dot_product));
            
            if (Vec3.length(slide_delta) < EPSILON) break;
            
            const slide_target = Vec3.add(current_pos, slide_delta);
            
            if (!collision_world.testPointCollision(slide_target, hull_type)) {
                current_pos = slide_target;
                moved = true;
                break;
            } else {
                const step_size = Vec3.length(slide_delta) * 0.5;
                if (step_size < EPSILON) break;
                
                const step_direction = Vec3.normalize(slide_delta);
                remaining_delta = Vec3.scale(step_direction, step_size);
            }
        } else {
            break;
        }
    }
    
    return if (moved) .{ .pos = current_pos } else null;
}

fn findGroundPoint(collision_world: *const World, start: Vec3, hull_type: HullType) Vec3 {
    var step_down: f32 = GROUND_SNAP;
    while (step_down <= STEP_HEIGHT + GROUND_SNAP) {
        const test_pos = Vec3.new(start.data[0], start.data[1] - step_down, start.data[2]);
        
        if (collision_world.testPointCollision(test_pos, hull_type)) {
            return Vec3.new(start.data[0], start.data[1] - (step_down - GROUND_SNAP), start.data[2]);
        }
        step_down += GROUND_SNAP;
    }
    return start;
}

pub fn isOnGroundPoint(collision_world: *const World, position: Vec3, hull_type: HullType) bool {
    const ground_test = Vec3.new(position.data[0], position.data[1] - GROUND_SNAP, position.data[2]);
    return collision_world.testPointCollision(ground_test, hull_type);
}

fn applySurfingCollisionResponse(velocity: *Vec3, surface_normal: Vec3, dt: f32, config: PhysicsConfig) void {
    const up_vector = Vec3.new(0, 1, 0);
    const surface_angle = std.math.acos(@abs(Vec3.dot(surface_normal, up_vector)));
    
    const velocity_into_surface = Vec3.dot(velocity.*, surface_normal);
    if (velocity_into_surface < 0) {
        velocity.* = Vec3.sub(velocity.*, Vec3.scale(surface_normal, velocity_into_surface));
    }
    
    const horizontal_velocity = Vec3.new(velocity.data[0], 0, velocity.data[2]);
    const horizontal_speed = Vec3.length(horizontal_velocity);
    
    if (horizontal_speed > 0.1) {
        var friction_factor: f32 = 0;
        
        if (surface_angle < 0.7) {
            friction_factor = config.friction;
        } else if (surface_angle < 1.2) {
            friction_factor = config.wall_friction * 0.1;
        }
        
        if (friction_factor > 0) {
            const factor = @max(0, horizontal_speed - horizontal_speed * friction_factor * dt) / horizontal_speed;
            velocity.data[0] *= factor;
            velocity.data[2] *= factor;
        }
    }
}