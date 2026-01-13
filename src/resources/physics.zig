const std = @import("std");
const math = @import("../lib/math.zig");
const bvh = @import("../lib/bvh.zig");

const Vec3 = math.Vec3;

pub const World = struct {
    bvh_tree: bvh.BVH(bvh.Brush),
    allocator: std.mem.Allocator,
    
    pub fn init(brushes: []const bvh.Brush, allocator: std.mem.Allocator) !World {
        return World{
            .bvh_tree = try bvh.BVH(bvh.Brush).init(brushes, allocator, bvh.Brush.getBounds),
            .allocator = allocator,
        };
    }
    
    pub fn deinit(self: *World) void {
        self.bvh_tree.deinit();
    }
    
    pub fn check(self: *const World, capsule: bvh.Capsule) ?bvh.CollisionResult {
        const capsule_bounds = capsule.bounds();
        var best_collision: ?bvh.CollisionResult = null;
        var best_distance: f32 = -std.math.floatMax(f32);
        
        // Use BVH config for stack size
        var stack: [64]u32 = undefined;
        var stack_ptr: u32 = 1;
        stack[0] = 0;
        
        while (stack_ptr > 0) {
            stack_ptr -= 1;
            const node_idx = stack[stack_ptr];
            
            if (node_idx >= self.bvh_tree.nodes.items.len) continue;
            const node = self.bvh_tree.nodes.items[node_idx];
            
            if (!node.bounds().intersects(capsule_bounds)) continue;
            
            if (node.isLeaf()) {
                const end_idx = @min(node.first + node.count(), self.bvh_tree.indices.items.len);
                for (node.first..end_idx) |i| {
                    if (self.bvh_tree.items[self.bvh_tree.indices.items[i]].checkCapsule(capsule)) |collision| {
                        if (collision.distance > best_distance) {
                            best_distance = collision.distance;
                            best_collision = collision;
                            if (collision.distance > 0) return collision;
                        }
                    }
                }
            } else {
                const left_child = node.left();
                const right_child = left_child + 1;
                
                if (right_child < self.bvh_tree.nodes.items.len and stack_ptr < stack.len) {
                    stack[stack_ptr] = right_child;
                    stack_ptr += 1;
                }
                if (left_child < self.bvh_tree.nodes.items.len and stack_ptr < stack.len) {
                    stack[stack_ptr] = left_child;
                    stack_ptr += 1;
                }
            }
        }
        
        return best_collision;
    }
};

pub const Config = struct {
    // Movement physics
    gravity: f32 = 6.0,
    jump_velocity: f32 = 2.0,
    max_speed: f32 = 4.0,
    air_speed: f32 = 0.7,
    acceleration: f32 = 70.0,
    friction: f32 = 5.0,
    
    // Collision detection
    epsilon: f32 = 0.001,
    step_height: f32 = 0.25,
    ground_snap: f32 = 0.05,
    margin: f32 = 0.02,
    slope_limit: f32 = 0.707, // 45 degrees
    slide_damping: f32 = 0.95,
    slide_damping_step: f32 = 0.05,
    max_slide_iterations: u32 = 3,
    trace_samples_min: u32 = 4,
    trace_samples_max: u32 = 16,
    trace_samples_multiplier: f32 = 10.0,
    binary_search_iterations: u32 = 8,
    
    // Hull dimensions
    standing_height: f32 = 0.7,
    standing_radius: f32 = 0.3,
    
    // Audio
    jump_sound_duration: f32 = 0.15,
};

pub fn Physics(comptime config: Config) type {
    return struct {
        const Self = @This();
        
        // Internal state
        const State = struct {
            pos: Vec3 = Vec3.zero(),
            vel: Vec3 = Vec3.zero(),
            on_ground: bool = false,
            yaw: f32 = 0,
            pitch: f32 = 0,
            mdx: f32 = 0,
            mdy: f32 = 0,
            keys: packed struct { 
                w: bool = false, 
                a: bool = false, 
                s: bool = false, 
                d: bool = false, 
                sp: bool = false 
            } = .{},
            lock: bool = false,
            jump_timer: f32 = 0,
            jump_active: bool = false,
        };
        
        pub const Move = struct { pos: Vec3, hit: ?bvh.CollisionResult = null };
        
        allocator: std.mem.Allocator,
        state: State = .{},
        
        pub fn init(allocator: std.mem.Allocator) Self {
            return Self{ .allocator = allocator };
        }
        
        pub fn deinit(self: *Self) void {
            _ = self;
        }
        
        pub fn setPosition(self: *Self, pos: Vec3) void {
            self.state.pos = pos;
        }
        
        pub fn getPosition(self: *const Self) Vec3 {
            return self.state.pos;
        }
        
        pub fn getViewMatrix(self: *const Self, eye_height: f32) math.Mat4 {
            const eye = Vec3.add(self.state.pos, Vec3.new(0, eye_height, 0));
            const cy, const sy = .{ @cos(self.state.yaw), @sin(self.state.yaw) };
            const cp, const sp = .{ @cos(self.state.pitch), @sin(self.state.pitch) };
            return math.Mat4{ .data = .{ 
                cy, sy*sp, -sy*cp, 0, 
                0, cp, sp, 0, 
                sy, -cy*sp, cy*cp, 0, 
                -eye.data[0]*cy - eye.data[2]*sy, 
                -eye.data[0]*sy*sp - eye.data[1]*cp + eye.data[2]*cy*sp, 
                eye.data[0]*sy*cp - eye.data[1]*sp - eye.data[2]*cy*cp, 
                1 
            }};
        }
        
        pub fn handleInput(self: *Self, mouse_sensitivity: f32, pitch_limit: f32) void {
            self.state.yaw += self.state.mdx * mouse_sensitivity;
            self.state.pitch = std.math.clamp(self.state.pitch + self.state.mdy * mouse_sensitivity, -pitch_limit, pitch_limit);
            self.state.mdx = 0; 
            self.state.mdy = 0;
        }
        
        pub fn update(self: *Self, collision_world: *const World, dt: f32) void {
            // Ground check and physics
            self.state.on_ground = self.isOnGround(collision_world, self.state.pos);
            if (!self.state.on_ground) self.state.vel.data[1] -= config.gravity * dt;
            if (self.state.keys.sp and self.state.on_ground) { 
                self.state.vel.data[1] = config.jump_velocity; 
                self.state.jump_timer = config.jump_sound_duration; 
                self.state.jump_active = true; 
            }
            
            // Movement
            const input_dir = self.getInputDirection();
            self.accelerate(input_dir, dt);
            if (self.state.on_ground) self.applyFriction(dt);
            
            // Move and handle collisions
            const move_result = move(collision_world, self.state.pos, Vec3.scale(self.state.vel, dt));
            self.state.pos = move_result.pos;
            
            if (move_result.hit) |hit_result| {
                const into_surface = Vec3.dot(self.state.vel, hit_result.normal);
                if (into_surface < 0) {
                    self.state.vel = Vec3.sub(self.state.vel, Vec3.scale(hit_result.normal, into_surface));
                }
            }
        }
        
        pub fn getAudioSample(self: *Self, sample_rate: f32) f32 {
            if (self.state.jump_active and self.state.jump_timer > 0) {
                const t = 1.0 - self.state.jump_timer / config.jump_sound_duration;
                self.state.jump_timer -= 1.0 / sample_rate;
                const sample = @sin((config.jump_sound_duration - self.state.jump_timer) * 500.0 * 3.14159265359) * @exp(-t * 8.0) * 0.3;
                if (self.state.jump_timer <= 0) self.state.jump_active = false;
                return sample;
            }
            return 0.0;
        }
        
        pub fn onKeyEvent(self: *Self, key: enum { W, A, S, D, SPACE, ESCAPE }, pressed: bool) void {
            switch (key) {
                .W => self.state.keys.w = pressed,
                .A => self.state.keys.a = pressed,
                .S => self.state.keys.s = pressed,
                .D => self.state.keys.d = pressed,
                .SPACE => self.state.keys.sp = pressed,
                .ESCAPE => if (pressed and self.state.lock) {
                    self.state.lock = false;
                },
            }
        }
        
        pub fn onMouseEvent(self: *Self, button: enum { LEFT }, pressed: bool) void {
            switch (button) {
                .LEFT => if (pressed and !self.state.lock) {
                    self.state.lock = true;
                },
            }
        }
        
        pub fn onMouseMove(self: *Self, dx: f32, dy: f32) void {
            if (self.state.lock) {
                self.state.mdx += dx;
                self.state.mdy += dy;
            }
        }
        
        pub fn isMouseLocked(self: *const Self) bool {
            return self.state.lock;
        }
        
        fn getInputDirection(self: *const Self) Vec3 {
            const fwd: f32 = if (self.state.keys.w) 1 else if (self.state.keys.s) -1 else 0;
            const side: f32 = if (self.state.keys.d) 1 else if (self.state.keys.a) -1 else 0;
            const fwd_vec = Vec3.new(@sin(self.state.yaw), 0, -@cos(self.state.yaw));
            const side_vec = Vec3.new(@cos(self.state.yaw), 0, @sin(self.state.yaw));
            return Vec3.add(Vec3.scale(fwd_vec, fwd), Vec3.scale(side_vec, side));
        }
        
        fn accelerate(self: *Self, wish_dir: Vec3, dt: f32) void {
            const len = @sqrt(wish_dir.data[0] * wish_dir.data[0] + wish_dir.data[2] * wish_dir.data[2]);
            if (len < 0.001) return;
            
            const wish = Vec3.scale(wish_dir, 1.0 / len);
            const max_vel = if (self.state.on_ground) config.max_speed * len else @min(config.max_speed * len, config.air_speed);
            const current_vel = Vec3.dot(Vec3.new(self.state.vel.data[0], 0, self.state.vel.data[2]), wish);
            const accel = @min(config.acceleration * dt, @max(0, max_vel - current_vel));
            
            self.state.vel.data[0] += wish.data[0] * accel;
            self.state.vel.data[2] += wish.data[2] * accel;
        }
        
        fn applyFriction(self: *Self, dt: f32) void {
            const speed = @sqrt(self.state.vel.data[0] * self.state.vel.data[0] + self.state.vel.data[2] * self.state.vel.data[2]);
            if (speed > 0.1) {
                const factor = @max(0, speed - speed * config.friction * dt) / speed;
                self.state.vel.data[0] *= factor; 
                self.state.vel.data[2] *= factor;
            } else { 
                self.state.vel.data[0] = 0; 
                self.state.vel.data[2] = 0; 
            }
        }
        
        fn isOnGround(self: *Self, world_collision: *const World, pos: Vec3) bool {
            _ = self;
            const test_pos = Vec3.new(pos.data[0], pos.data[1] - config.ground_snap, pos.data[2]);
            const capsule = bvh.Capsule.fromHull(test_pos, .standing);
            if (world_collision.check(capsule)) |hit_result| {
                return hit_result.normal.data[1] > config.slope_limit;
            }
            return false;
        }
        
        pub fn move(world_collision: *const World, start: Vec3, delta: Vec3) Move {
            if (Vec3.length(delta) < config.epsilon) return .{ .pos = start };
            
            // Try direct movement
            const end_capsule = bvh.Capsule.fromHull(Vec3.add(start, delta), .standing);
            if (world_collision.check(end_capsule) == null) {
                return .{ .pos = Vec3.add(start, delta) };
            }
            
            // Slide movement
            return slide(world_collision, start, delta);
        }
        
        const TraceResult = struct { end_pos: Vec3, hit: ?bvh.CollisionResult, time: f32 };
        
        fn trace(world_collision: *const World, start: Vec3, delta: Vec3) TraceResult {
            const move_length = Vec3.length(delta);
            if (move_length < config.epsilon) return .{ .end_pos = start, .hit = null, .time = 1.0 };
            
            var best_time: f32 = 1.0;
            var hit: ?bvh.CollisionResult = null;
            
            const samples = @min(config.trace_samples_max, @max(config.trace_samples_min, @as(u32, @intFromFloat(move_length * config.trace_samples_multiplier))));
            for (0..samples) |i| {
                const t = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(samples - 1));
                const test_pos = Vec3.add(start, Vec3.scale(delta, t));
                const capsule = bvh.Capsule.fromHull(test_pos, .standing);
                
                if (world_collision.check(capsule)) |hit_result| {
                    best_time = binarySearch(world_collision, start, delta, 
                                       if (i > 0) @as(f32, @floatFromInt(i - 1)) / @as(f32, @floatFromInt(samples - 1)) else 0.0, t);
                    hit = hit_result;
                    break;
                }
            }
            
            if (hit != null) {
                best_time = @max(0, best_time - config.margin / move_length);
            }
            
            return .{ .end_pos = Vec3.add(start, Vec3.scale(delta, best_time)), .hit = hit, .time = best_time };
        }
        
        fn slide(world_collision: *const World, start: Vec3, velocity: Vec3) Move {
            var pos = start;
            var vel = velocity;
            var remaining_time: f32 = 1.0;
            var first_hit: ?bvh.CollisionResult = null;
            var planes: [config.max_slide_iterations]Vec3 = undefined;
            var num_planes: u32 = 0;
            
            for (0..config.max_slide_iterations) |bump| {
                if (Vec3.length(vel) < config.epsilon or remaining_time <= 0.001) break;
                
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
                    
                    vel = Vec3.scale(vel, config.slide_damping - (@as(f32, @floatFromInt(bump)) * config.slide_damping_step));
                } else break;
            }
            
            return .{ .pos = pos, .hit = first_hit };
        }
        
        fn binarySearch(world_collision: *const World, start: Vec3, delta: Vec3, low: f32, high: f32) f32 {
            var l = low;
            var h = high;
            for (0..config.binary_search_iterations) |_| {
                const mid = (l + h) * 0.5;
                const test_pos = Vec3.add(start, Vec3.scale(delta, mid));
                const capsule = bvh.Capsule.fromHull(test_pos, .standing);
                
                if (world_collision.check(capsule) != null) {
                    h = mid;
                } else {
                    l = mid;
                }
            }
            return l;
        }
    };
}