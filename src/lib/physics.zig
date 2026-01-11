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
        
        // Movement with step-up
        const delta = Vec3.scale(p.vel, dt);
        const move_result = moveWithStepUp(collision_world, t.pos, delta, hull_type);
        t.pos = move_result.pos;
        p.on_ground = collision_world.check(Vec3.new(t.pos.data[0], t.pos.data[1] - GROUND_SNAP, t.pos.data[2]), hull_type) != null;
        
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

fn moveWithStepUp(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) Move {
    // Try normal movement first
    const normal_move = move(world_collision, start, delta, hull_type);
    
    // If we moved successfully or there's no horizontal movement, we're done
    const horizontal_dist = @sqrt(delta.data[0] * delta.data[0] + delta.data[2] * delta.data[2]);
    if (normal_move.collision == null or horizontal_dist < EPSILON) {
        return normal_move;
    }
    
    // Try step-up movement
    const horizontal = Vec3.new(delta.data[0], 0, delta.data[2]);
    const step_start = Vec3.add(start, Vec3.new(0, STEP_HEIGHT, 0));
    
    // Check if we can fit at step height
    if (world_collision.check(step_start, hull_type) != null) {
        return normal_move; // Can't step up
    }
    
    // Move horizontally at step height
    const step_move = move(world_collision, step_start, horizontal, hull_type);
    
    // Drop down to find ground
    const drop_start = Vec3.add(step_move.pos, Vec3.new(0, delta.data[1], 0));
    const drop_delta = Vec3.new(0, -(STEP_HEIGHT + GROUND_SNAP), 0);
    const drop_result = traceMove(world_collision, drop_start, drop_delta, hull_type);
    
    // Use step-up result if we moved further horizontally
    const normal_horizontal = @sqrt((normal_move.pos.data[0] - start.data[0]) * (normal_move.pos.data[0] - start.data[0]) + 
                                   (normal_move.pos.data[2] - start.data[2]) * (normal_move.pos.data[2] - start.data[2]));
    const step_horizontal = @sqrt((drop_result.end_pos.data[0] - start.data[0]) * (drop_result.end_pos.data[0] - start.data[0]) + 
                                 (drop_result.end_pos.data[2] - start.data[2]) * (drop_result.end_pos.data[2] - start.data[2]));
    
    if (step_horizontal > normal_horizontal + EPSILON) {
        return .{ .pos = drop_result.end_pos, .collision = normal_move.collision };
    }
    
    return normal_move;
}

pub fn move(world_collision: *const World, start: Vec3, delta: Vec3, hull_type: HullType) Move {
    return slideMove(world_collision, start, delta, hull_type);
}

fn slideMove(world_collision: *const World, start: Vec3, velocity: Vec3, hull_type: HullType) Move {
    if (Vec3.length(velocity) < EPSILON) return .{ .pos = start };
    
    var pos = start;
    var vel = velocity;
    var first_collision: ?Collision = null;
    
    // Quake allows up to 4 bumps per move
    for (0..4) |bump| {
        if (Vec3.length(vel) < EPSILON) break;
        
        const target = Vec3.add(pos, vel);
        
        // Check if we can move directly
        if (world_collision.check(target, hull_type) == null) {
            pos = target;
            break;
        }
        
        // Find collision point using binary search
        const trace = traceMove(world_collision, pos, vel, hull_type);
        pos = trace.end_pos;
        
        if (trace.collision) |collision| {
            if (first_collision == null) first_collision = collision;
            
            // Remove velocity component into the surface
            const into_surface = Vec3.dot(vel, collision.normal);
            if (into_surface >= 0) break; // Moving away from surface
            
            vel = Vec3.sub(vel, Vec3.scale(collision.normal, into_surface));
            
            // If this is our second+ bump, check for corner case
            if (bump > 0) {
                // Reduce velocity to prevent infinite bouncing
                vel = Vec3.scale(vel, 0.95);
                
                // Stop if velocity becomes too small
                if (Vec3.length(vel) < EPSILON * 10) break;
            }
        } else {
            // No collision found but we couldn't move - shouldn't happen
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
    
    // Move slightly away from the surface to prevent getting stuck
    if (collision != null and best_fraction > 0) {
        best_fraction = @max(0, best_fraction - EPSILON);
    }
    
    const final_pos = Vec3.add(start, Vec3.scale(delta, best_fraction));
    return .{
        .end_pos = final_pos,
        .collision = collision,
        .fraction = best_fraction,
    };
}

