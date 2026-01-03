const std = @import("std");
const math = @import("math.zig");
const collision = @import("collision.zig");

const Vec3 = math.Vec3;
const AABB = math.AABB;
const BrushWorld = collision.BrushWorld;

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

pub fn update(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, world: *const BrushWorld, dt: f32) void {
    const size = Vec3.new(0.98, 1.8, 0.98);
    const player_aabb = AABB.fromCenterSize(Vec3.zero(), size);
    
    for (transforms, physics, inputs, audios) |*t, *p, *i, *a| {
        const fwd: f32 = if (i.keys.w) 1 else if (i.keys.s) -1 else 0;
        const side: f32 = if (i.keys.d) 1 else if (i.keys.a) -1 else 0;
        const jump = i.keys.sp;
        
        // Gravity
        p.vel.data[1] -= 12.0 * dt;
        
        // Jump
        if (jump and p.on_ground) { 
            p.vel.data[1] = 4.0; 
            a.timer = 0.15; 
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
            const speed = if (p.on_ground) 4.0 * len else @min(4.0 * len, 0.7);
            const current = Vec3.dot(Vec3.new(p.vel.data[0], 0, p.vel.data[2]), wish);
            const add = @max(0, speed - current);
            if (add > 0) {
                const accel = @min(70.0 * dt, add);
                p.vel.data[0] += wish.data[0] * accel;
                p.vel.data[2] += wish.data[2] * accel;
            }
        }
        
        // Friction
        if (p.on_ground) {
            const speed = @sqrt(p.vel.data[0] * p.vel.data[0] + p.vel.data[2] * p.vel.data[2]);
            if (speed > 0.1) {
                const friction_factor: f32 = 5.0;
                const factor = @max(0, speed - @max(speed, 0.1) * friction_factor * dt) / speed;
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
                const wall_friction: f32 = 2.0;
                const factor = @max(0, horizontal_speed - horizontal_speed * wall_friction * dt) / horizontal_speed;
                p.vel.data[0] *= factor;
                p.vel.data[2] *= factor;
            }
        }
        
        t.pos = move_result.position;
    }
}