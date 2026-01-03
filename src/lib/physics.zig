const std = @import("std");
const math = @import("math.zig");
const collision = @import("collision.zig");

const Vec3 = math.Vec3;
const AABB = math.AABB;
const World = collision.World;

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

pub fn update(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, world: *const World, dt: f32) void {
    updateWithConfig(transforms, physics, inputs, audios, world, dt, default_config);
}

pub fn updateWithConfig(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, world: *const World, dt: f32, config: PhysicsConfig) void {
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
        const move_result = world.movePlayer(t.pos, delta, player_aabb);
        
        // Apply collision response
        p.vel = Vec3.add(p.vel, Vec3.scale(move_result.velocity_adjustment, 1.0 / dt));
        p.on_ground = move_result.on_ground;
        if (move_result.hit_ceiling) p.vel.data[1] = @min(p.vel.data[1], 0);
        
        // Wall friction
        if (move_result.hit_wall and p.on_ground) {
            const horizontal_speed = @sqrt(p.vel.data[0] * p.vel.data[0] + p.vel.data[2] * p.vel.data[2]);
            if (horizontal_speed > 0.1) {
                const factor = @max(0, horizontal_speed - horizontal_speed * config.wall_friction * dt) / horizontal_speed;
                p.vel.data[0] *= factor;
                p.vel.data[2] *= factor;
            }
        }
        
        t.pos = move_result.position;
    }
}