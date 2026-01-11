const std = @import("std");
const math = @import("math.zig");
const world = @import("world.zig");

const Vec3 = math.Vec3;
const World = world.World;
const HullType = world.HullType;
const Collision = world.Collision;
// Global constants - single source of truth
pub const EPSILON: f32 = 0.001;
pub const STEP_HEIGHT: f32 = 0.25;
pub const GROUND_SNAP: f32 = 0.05;
pub const COLLISION_MARGIN: f32 = 0.02; // Small margin to prevent seam catching

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
        
        // Check ground status with slope angle limit (45 degrees = 0.707 dot product with up vector)
        const ground_check = checkGround(collision_world, t.pos, hull_type);
        p.on_ground = ground_check.is_ground;
        
        // Apply gravity only when not on valid ground (slopes steeper than 45° get gravity)
        if (!p.on_ground) {
            p.vel.data[1] -= config.gravity * dt;
        }
        
        // Jump
        if (jump and p.on_ground) { 
            p.vel.data[1] = config.jump_velocity; 
            a.timer = config.jump_sound_duration; 
            a.active = true; 
        }
        
        // Movement direction
        const fwd_vec = Vec3.new(@sin(i.yaw), 0, -@cos(i.yaw));
        const side_vec = Vec3.new(@cos(i.yaw), 0, @sin(i.yaw));
        const dir = Vec3.add(Vec3.scale(fwd_vec, fwd), Vec3.scale(side_vec, side));
        
        // Acceleration
        const len = @sqrt(dir.data[0] * dir.data[0] + dir.data[2] * dir.data[2]);
        if (len > 0.001) {
            const wish = Vec3.scale(dir, 1.0 / len);
            const max_vel = if (p.on_ground) config.max_speed * len else @min(config.max_speed * len, config.air_speed);
            const current_vel = Vec3.dot(Vec3.new(p.vel.data[0], 0, p.vel.data[2]), wish);
            const accel = @min(config.acceleration * dt, @max(0, max_vel - current_vel));
            p.vel.data[0] += wish.data[0] * accel;
            p.vel.data[2] += wish.data[2] * accel;
        }
        
        // Friction when on ground
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

const GroundCheck = struct {
    is_ground: bool,
    normal: ?Vec3 = null,
};

fn checkGround(world_collision: *const World, pos: Vec3, hull_type: HullType) GroundCheck {
    const ground_test_pos = Vec3.new(pos.data[0], pos.data[1] - GROUND_SNAP, pos.data[2]);
    
    if (world_collision.check(ground_test_pos, hull_type)) |collision| {
        // Check if surface is walkable (less than 45 degrees from horizontal)
        // cos(45°) = 0.707, so normal.y should be > 0.707 for walkable surface
        const is_walkable = collision.normal.data[1] > 0.707;
        return .{ .is_ground = is_walkable, .normal = collision.normal };
    }
    
    return .{ .is_ground = false };
}

pub fn move(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) Move {
    return slideMove(world_collision, start, delta, hull_type);
}

fn slideMove(world_collision: *const World, start: Vec3, velocity: Vec3, hull_type: HullType) Move {
    if (Vec3.length(velocity) < EPSILON) return .{ .pos = start };
    
    var pos = start;
    var vel = velocity;
    var first_collision: ?Collision = null;
    var planes: [4]Vec3 = undefined; // Store collision planes to avoid oscillation
    var num_planes: u32 = 0;
    
    // Try direct movement first
    const direct_target = Vec3.add(pos, vel);
    if (world_collision.check(direct_target, hull_type) == null) {
        return .{ .pos = direct_target };
    }
    
    // Try step-up for horizontal movement
    const horizontal = Vec3.new(vel.data[0], 0, vel.data[2]);
    if (Vec3.length(horizontal) >= EPSILON) {
        const step_up = Vec3.add(Vec3.add(pos, horizontal), Vec3.new(0, STEP_HEIGHT, 0));
        if (world_collision.check(step_up, hull_type) == null) {
            // Find ground and apply vertical movement
            const drop_trace = traceMove(world_collision, step_up, Vec3.new(0, -(STEP_HEIGHT + GROUND_SNAP), 0), hull_type);
            const final_pos = Vec3.add(drop_trace.end_pos, Vec3.new(0, vel.data[1], 0));
            if (world_collision.check(final_pos, hull_type) == null) {
                return .{ .pos = final_pos };
            } else {
                return .{ .pos = drop_trace.end_pos };
            }
        }
    }
    
    // Slide along surfaces (up to 3 bumps)
    for (0..3) |bump| {
        if (Vec3.length(vel) < EPSILON) break;
        
        const trace = traceMove(world_collision, pos, vel, hull_type);
        pos = trace.end_pos;
        
        if (trace.collision) |collision| {
            if (first_collision == null) first_collision = collision;
            
            const into_surface = Vec3.dot(vel, collision.normal);
            if (into_surface >= 0) break;
            
            // Store this plane to prevent oscillation
            if (num_planes < planes.len) {
                planes[num_planes] = collision.normal;
                num_planes += 1;
            }
            
            // Check if we're hitting a plane we've already hit (oscillation detection)
            var is_oscillating = false;
            if (num_planes > 1) {
                for (planes[0..num_planes-1]) |plane| {
                    if (Vec3.dot(collision.normal, plane) > 0.99) { // Nearly same plane
                        is_oscillating = true;
                        break;
                    }
                }
            }
            
            if (is_oscillating) {
                // Stop movement to prevent oscillation
                break;
            }
            
            // Clip velocity against all planes we've hit
            var clipped_vel = vel;
            for (planes[0..num_planes]) |plane| {
                const dot = Vec3.dot(clipped_vel, plane);
                if (dot < 0) {
                    clipped_vel = Vec3.sub(clipped_vel, Vec3.scale(plane, dot));
                }
            }
            
            vel = clipped_vel;
            if (bump > 0) vel = Vec3.scale(vel, 0.95); // Reduce bouncing
        } else {
            break;
        }
    }
    
    return .{ .pos = pos, .collision = first_collision };
}

const TraceResult = struct {
    end_pos: Vec3,
    collision: ?Collision,
    fraction: f32, // 0.0 = start, 1.0 = end
};

fn traceMove(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) TraceResult {
    // Binary search for collision point
    var low: f32 = 0.0;
    var high: f32 = 1.0;
    var best_fraction: f32 = 1.0;
    var collision: ?Collision = null;
    
    // 8 iterations gives us 1/256 precision
    for (0..8) |_| {
        const mid = (low + high) * 0.5;
        const test_pos = Vec3.add(start, Vec3.scale(delta, mid));
        
        if (world_collision.check(test_pos, hull_type)) |hit| {
            high = mid;
            best_fraction = mid;
            collision = hit;
        } else {
            low = mid;
        }
    }
    
    // Apply collision margin to prevent seam catching
    if (collision != null and best_fraction > 0) {
        // Calculate margin based on movement direction
        const move_length = Vec3.length(delta);
        const margin_fraction = if (move_length > 0) COLLISION_MARGIN / move_length else 0;
        best_fraction = @max(0, best_fraction - margin_fraction);
    }
    
    const final_pos = Vec3.add(start, Vec3.scale(delta, best_fraction));
    return .{
        .end_pos = final_pos,
        .collision = collision,
        .fraction = best_fraction,
    };
}

