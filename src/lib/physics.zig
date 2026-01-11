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
        const fwd: f32 = if (i.keys.w) 1 else if (i.keys.s) -1 else 0;
        const side: f32 = if (i.keys.d) 1 else if (i.keys.a) -1 else 0;
        const jump = i.keys.sp;
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
        
        // Simple friction - only when on ground
        if (p.on_ground) {
            const speed = @sqrt(p.vel.data[0] * p.vel.data[0] + p.vel.data[2] * p.vel.data[2]);
            if (speed > 0.1) {
                const factor = @max(0, speed - speed * config.friction * dt) / speed;
                p.vel.data[0] *= factor; 
                p.vel.data[2] *= factor;
            } else { 
                p.vel.data[0] = 0; 
                p.vel.data[2] = 0; 
            }
        }
        
        // Movement
        const delta = Vec3.scale(p.vel, dt);
        const move_result = move(collision_world, t.pos, delta, hull_type);
        t.pos = move_result.pos;
        // check if on ground
        p.on_ground = blk: {
            const ground_test = Vec3.new(t.pos.data[0], t.pos.data[1] - GROUND_SNAP, t.pos.data[2]);
            break :blk collision_world.check(ground_test, hull_type) != null;
        };
        
        // Simple collision response - only redirect when being pushed into surface
        if (move_result.collision) |collision| {
            const velocity_into_surface = Vec3.dot(p.vel, collision.normal);
            
            // Only redirect velocity if moving into the surface
            if (velocity_into_surface < 0) {
                p.vel = Vec3.sub(p.vel, Vec3.scale(collision.normal, velocity_into_surface));
            }
        }
    }
}

pub fn move(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) Move {
    if (Vec3.length(delta) < EPSILON) return .{ .pos = start };
    
    // try direct movement
    {
        const target = Vec3.add(start, delta);
        if (world_collision.check(target, hull_type) == null) return Move{ .pos = target };
    }
    
    // try step movement
    {
        const horizontal = Vec3.new(delta.data[0], 0, delta.data[2]);
        if (Vec3.length(horizontal) >= EPSILON) {
            const step_pos = Vec3.add(Vec3.add(start, horizontal), Vec3.new(0, STEP_HEIGHT, 0));
            if (world_collision.check(step_pos, hull_type) == null) {
                // find ground
                const ground_pos = blk: {
                    var step_down: f32 = GROUND_SNAP;
                    while (step_down <= STEP_HEIGHT + GROUND_SNAP) {
                        const test_pos = Vec3.new(step_pos.data[0], step_pos.data[1] - step_down, step_pos.data[2]);
                        if (world_collision.check(test_pos, hull_type) != null) {
                            break :blk Vec3.new(step_pos.data[0], step_pos.data[1] - (step_down - GROUND_SNAP), step_pos.data[2]);
                        }
                        step_down += GROUND_SNAP;
                    }
                    break :blk step_pos;
                };
                
                const final_pos = Vec3.new(ground_pos.data[0], ground_pos.data[1] + delta.data[1], ground_pos.data[2]);
                if (world_collision.check(final_pos, hull_type) == null) {
                    return Move{ .pos = final_pos };
                } else {
                    return Move{ .pos = ground_pos };
                }
            }
        }
    }
    
    // try slide movement - simplified to prevent surfing
    {
        var current_pos = start;
        var remaining_delta = delta;
        var collision: ?Collision = null;
        
        // Only allow 2 iterations to prevent complex sliding chains
        for (0..2) |_| {
            if (Vec3.length(remaining_delta) < EPSILON) break;
            
            const target_pos = Vec3.add(current_pos, remaining_delta);
            
            if (world_collision.check(target_pos, hull_type)) |response| {
                collision = response;
                
                // Simple slide: remove component along normal
                const dot_product = Vec3.dot(remaining_delta, response.normal);
                if (dot_product < 0) { // Only slide if moving into surface
                    remaining_delta = Vec3.sub(remaining_delta, Vec3.scale(response.normal, dot_product));
                    
                    // Reduce slide distance to prevent excessive momentum preservation
                    remaining_delta = Vec3.scale(remaining_delta, 0.8);
                    
                    if (Vec3.length(remaining_delta) < EPSILON) break;
                    
                    const slide_target = Vec3.add(current_pos, remaining_delta);
                    if (world_collision.check(slide_target, hull_type) == null) {
                        current_pos = slide_target;
                    }
                }
                break; // Stop after first collision to prevent chaining
            } else {
                current_pos = target_pos;
                break;
            }
        }
        
        return Move{ .pos = current_pos, .collision = collision };
    }
}

