const std = @import("std");
const math = @import("math.zig");
const world = @import("world.zig");

const Vec3 = math.Vec3;
const AABB = math.AABB;
const World = world.World;
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
    const player_aabb = AABB.fromCenterSize(Vec3.zero(), config.player_size);
    
    for (transforms, physics, inputs, audios) |*t, *p, *i, *a| {
        const fwd: f32 = if (i.keys.w) 1 else if (i.keys.s) -1 else 0;
        const side: f32 = if (i.keys.d) 1 else if (i.keys.a) -1 else 0;
        const jump = i.keys.sp;
        
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
        
        // Movement with collision
        const delta = Vec3.scale(p.vel, dt);
        const move_result = movePlayer(collision_world, t.pos, delta, player_aabb);
        
        // Update position
        t.pos = move_result.pos;
        
        // Check ground state
        p.on_ground = isOnGround(collision_world, t.pos, player_aabb);
        
        // Apply collision response - if we hit something, adjust velocity
        if (move_result.hit) {
            const actual_delta = Vec3.sub(move_result.pos, t.pos);
            const intended_delta = delta;
            
            // Calculate what part of velocity was blocked
            const blocked_delta = Vec3.sub(intended_delta, actual_delta);
            
            // Remove blocked velocity components
            if (Vec3.length(blocked_delta) > EPSILON) {
                // If we moved less than intended, we hit something
                
                // Remove velocity in the direction we couldn't move
                if (@abs(blocked_delta.data[1]) > EPSILON) {
                    // Vertical collision - stop vertical movement
                    if (blocked_delta.data[1] > 0) {
                        p.vel.data[1] = @min(p.vel.data[1], 0); // Hit ceiling
                    } else {
                        p.vel.data[1] = @max(p.vel.data[1], 0); // Hit ground
                    }
                }
                
                // Horizontal collision - apply wall friction
                const horizontal_blocked = @sqrt(blocked_delta.data[0] * blocked_delta.data[0] + blocked_delta.data[2] * blocked_delta.data[2]);
                if (horizontal_blocked > EPSILON and p.on_ground) {
                    const horizontal_speed = @sqrt(p.vel.data[0] * p.vel.data[0] + p.vel.data[2] * p.vel.data[2]);
                    if (horizontal_speed > 0.1) {
                        const factor = @max(0, horizontal_speed - horizontal_speed * config.wall_friction * dt) / horizontal_speed;
                        p.vel.data[0] *= factor;
                        p.vel.data[2] *= factor;
                    }
                }
            }
        }
    }
}

// Pike-style movement system - belongs in physics, not world
pub fn movePlayer(collision_world: *const World, start: Vec3, delta: Vec3, player_aabb: AABB) MoveResult {
    if (Vec3.length(delta) < EPSILON) return .{ .pos = start };
    
    return tryDirect(collision_world, start, delta, player_aabb) orelse 
           tryStep(collision_world, start, delta, player_aabb) orelse 
           trySlide(collision_world, start, delta, player_aabb) orelse 
           .{ .pos = start, .hit = true };
}

// Try direct movement
fn tryDirect(collision_world: *const World, pos: Vec3, delta: Vec3, aabb: AABB) ?MoveResult {
    const target = Vec3.add(pos, delta);
    const test_aabb = AABB{ .min = Vec3.add(target, aabb.min), .max = Vec3.add(target, aabb.max) };
    return if (!collision_world.testCollision(test_aabb)) .{ .pos = target } else null;
}

// Try step-up
fn tryStep(collision_world: *const World, pos: Vec3, delta: Vec3, aabb: AABB) ?MoveResult {
    const horizontal = Vec3.new(delta.data[0], 0, delta.data[2]);
    if (Vec3.length(horizontal) < EPSILON) return null;
    
    const step_pos = Vec3.add(Vec3.add(pos, horizontal), Vec3.new(0, STEP_HEIGHT, 0));
    const step_aabb = AABB{ .min = Vec3.add(step_pos, aabb.min), .max = Vec3.add(step_pos, aabb.max) };
    
    if (!collision_world.testCollision(step_aabb)) {
        const ground_pos = findGround(collision_world, step_pos, aabb);
        const final_pos = Vec3.new(ground_pos.data[0], ground_pos.data[1] + delta.data[1], ground_pos.data[2]);
        const final_aabb = AABB{ .min = Vec3.add(final_pos, aabb.min), .max = Vec3.add(final_pos, aabb.max) };
        return if (!collision_world.testCollision(final_aabb)) .{ .pos = final_pos } else .{ .pos = ground_pos };
    }
    return null;
}

// Try wall sliding
fn trySlide(collision_world: *const World, pos: Vec3, delta: Vec3, aabb: AABB) ?MoveResult {
    var result = pos;
    var moved = false;
    
    inline for ([_]usize{ 0, 1, 2 }) |axis| {
        if (@abs(delta.data[axis]) > EPSILON) {
            var test_pos = result;
            test_pos.data[axis] += delta.data[axis];
            const test_aabb = AABB{ .min = Vec3.add(test_pos, aabb.min), .max = Vec3.add(test_pos, aabb.max) };
            if (!collision_world.testCollision(test_aabb)) {
                result.data[axis] = test_pos.data[axis];
                moved = true;
            }
        }
    }
    
    return if (moved) .{ .pos = result } else null;
}

fn findGround(collision_world: *const World, start: Vec3, aabb: AABB) Vec3 {
    var step_down: f32 = GROUND_SNAP;
    while (step_down <= STEP_HEIGHT + GROUND_SNAP) {
        const test_pos = Vec3.new(start.data[0], start.data[1] - step_down, start.data[2]);
        const test_aabb = AABB{ .min = Vec3.add(test_pos, aabb.min), .max = Vec3.add(test_pos, aabb.max) };
        
        if (collision_world.testCollision(test_aabb)) {
            return Vec3.new(start.data[0], start.data[1] - (step_down - GROUND_SNAP), start.data[2]);
        }
        step_down += GROUND_SNAP;
    }
    return start;
}

pub fn isOnGround(collision_world: *const World, position: Vec3, aabb: AABB) bool {
    const ground_test = Vec3.new(position.data[0], position.data[1] - GROUND_SNAP, position.data[2]);
    const ground_aabb = AABB{ .min = Vec3.add(ground_test, aabb.min), .max = Vec3.add(ground_test, aabb.max) };
    return collision_world.testCollision(ground_aabb);
}