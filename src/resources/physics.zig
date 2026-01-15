const std = @import("std");
const math = @import("math");
const bvh = @import("../lib/bvh.zig");
const brush = @import("../lib/brush.zig");

const Vec3 = math.Vec3;

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
    margin: f32 = 0.02,
    slope_limit: f32 = 0.707, // 45 degrees
    slide_damping: f32 = 0.95,
    slide_damping_step: f32 = 0.05,
    max_slide_iterations: u32 = 3,
    
    // Player dimensions for brush expansion
    player_radius: f32 = 0.3,
    
    // BVH stack sizing
    min_stack_size: u32 = 32,
    stack_size_multiplier: u32 = 2,
    stack_size_padding: u32 = 8,
    
    // Audio
    jump_sound_duration: f32 = 0.15,
    audio_frequency: f32 = 500.0,
    audio_decay: f32 = 8.0,
    audio_amplitude: f32 = 0.3,
    
    // Thresholds
    input_deadzone: f32 = 0.001,
    friction_threshold: f32 = 0.1,
};

pub const World = struct {
    bvh_tree: bvh.BVH(brush.Brush),
    stack: std.ArrayListUnmanaged(u32),
    allocator: std.mem.Allocator,
    brushes: []brush.Brush,
    
    const config = Config{};
    
    pub fn init(original_brushes: []const brush.Brush, allocator: std.mem.Allocator) !World {
        if (original_brushes.len == 0) return error.NoBrushes;
        
        // Expand brushes by player radius for point-based collision
        var brushes = try allocator.alloc(brush.Brush, original_brushes.len);
        for (original_brushes, 0..) |b, i| {
            brushes[i] = try b.expand(config.player_radius, allocator);
        }
        
        const bvh_tree = try bvh.BVH(brush.Brush).init(brushes, allocator, brush.Brush.getBounds);
        const max_stack_size = @max(config.min_stack_size, config.stack_size_multiplier * @as(u32, @intFromFloat(@log2(@as(f32, @floatFromInt(bvh_tree.nodes.len))))) + config.stack_size_padding);
        
        var stack = std.ArrayListUnmanaged(u32){};
        try stack.ensureTotalCapacity(allocator, max_stack_size);
        
        return World{ 
            .bvh_tree = bvh_tree, 
            .stack = stack, 
            .allocator = allocator,
            .brushes = brushes,
        };
    }
    
    pub fn deinit(self: *World) void {
        // Free expanded brush plane data
        for (self.brushes) |b| {
            self.allocator.free(b.plane_data.normals);
            self.allocator.free(b.plane_data.distances);
        }
        self.allocator.free(self.brushes);
        
        self.bvh_tree.deinit();
        self.stack.deinit(self.allocator);
    }
    
    pub fn raycast(self: *World, ray_start: Vec3, ray_dir: Vec3, max_distance: f32) ?brush.CollisionResult {
        var best_hit: ?brush.CollisionResult = null;
        var best_distance: f32 = max_distance;
        
        // Create ray AABB for BVH traversal
        const ray_end = Vec3.add(ray_start, Vec3.scale(ray_dir, max_distance));
        const ray_bounds = math.AABB{
            .min = Vec3.min(ray_start, ray_end),
            .max = Vec3.max(ray_start, ray_end),
        };
        
        self.stack.clearRetainingCapacity();
        self.stack.appendAssumeCapacity(0);
        
        while (self.stack.items.len > 0) {
            const node_idx = self.stack.items[self.stack.items.len - 1];
            self.stack.items.len -= 1;
            if (node_idx >= self.bvh_tree.nodes.len) continue;
            
            const node_bounds = self.bvh_tree.nodes.get(node_idx).bounds;
            const node_type = self.bvh_tree.nodes.get(node_idx).node_type;
            if (!node_bounds.intersects(ray_bounds)) continue;
            
            if (node_type == .leaf) {
                const first = self.bvh_tree.nodes.get(node_idx).first;
                const count = self.bvh_tree.nodes.get(node_idx).count;
                const end_idx = @min(first + count, self.bvh_tree.indices.items.len);
                for (first..end_idx) |i| {
                    if (self.bvh_tree.items[self.bvh_tree.indices.items[i]].rayIntersect(ray_start, ray_dir, best_distance)) |hit| {
                        if (hit.distance < best_distance) {
                            best_distance = hit.distance;
                            best_hit = hit;
                        }
                    }
                }
            } else {
                const left = self.bvh_tree.nodes.get(node_idx).first;
                if (left + 1 < self.bvh_tree.nodes.len) self.stack.appendAssumeCapacity(left + 1);
                if (left < self.bvh_tree.nodes.len) self.stack.appendAssumeCapacity(left);
            }
        }
        
        return best_hit;
    }
};

pub fn Physics(comptime config: Config) type {
    return struct {
        const Self = @This();
        
        pub const State = struct {
            pos: Vec3 = Vec3.zero(),
            vel: Vec3 = Vec3.zero(),
            on_ground: bool = false,
            yaw: f32 = 0,
            pitch: f32 = 0,
            jump_timer: f32 = 0,
            jump_active: bool = false,
        };
        
        state: State = .{},
        
        pub fn update(self: *Self, world: *World, wish_dir: Vec3, jump: bool, dt: f32) void {
            // Apply forces
            if (!self.state.on_ground) self.state.vel.data[1] -= config.gravity * dt;
            if (jump and self.state.on_ground) {
                self.state.vel.data[1] = config.jump_velocity;
                self.state.jump_timer = config.jump_sound_duration;
                self.state.jump_active = true;
            }
            
            self.accelerate(wish_dir, dt);
            if (self.state.on_ground) self.friction(dt);
            
            // Move and handle collision
            self.move(world, Vec3.scale(self.state.vel, dt));
        }
        
        pub fn getAudioSample(self: *Self, sample_rate: f32) f32 {
            if (self.state.jump_active and self.state.jump_timer > 0) {
                const t = 1.0 - self.state.jump_timer / config.jump_sound_duration;
                self.state.jump_timer -= 1.0 / sample_rate;
                const sample = @sin((config.jump_sound_duration - self.state.jump_timer) * config.audio_frequency * std.math.pi) * @exp(-t * config.audio_decay) * config.audio_amplitude;
                if (self.state.jump_timer <= 0) self.state.jump_active = false;
                return sample;
            }
            return 0.0;
        }
        
        fn accelerate(self: *Self, wish_dir: Vec3, dt: f32) void {
            const len = @sqrt(wish_dir.data[0] * wish_dir.data[0] + wish_dir.data[2] * wish_dir.data[2]);
            if (len < config.input_deadzone) return;
            
            const wish = Vec3.scale(wish_dir, 1.0 / len);
            const max_vel = if (self.state.on_ground) config.max_speed * len else @min(config.max_speed * len, config.air_speed);
            const current_vel = Vec3.dot(Vec3.new(self.state.vel.data[0], 0, self.state.vel.data[2]), wish);
            const accel = @min(config.acceleration * dt, @max(0, max_vel - current_vel));
            
            self.state.vel.data[0] += wish.data[0] * accel;
            self.state.vel.data[2] += wish.data[2] * accel;
        }
        
        fn friction(self: *Self, dt: f32) void {
            const speed = @sqrt(self.state.vel.data[0] * self.state.vel.data[0] + self.state.vel.data[2] * self.state.vel.data[2]);
            if (speed <= config.friction_threshold) {
                self.state.vel.data[0] = 0;
                self.state.vel.data[2] = 0;
            } else {
                const factor = @max(0, speed - speed * config.friction * dt) / speed;
                self.state.vel.data[0] *= factor;
                self.state.vel.data[2] *= factor;
            }
        }
        
        fn move(self: *Self, world: *World, delta: Vec3) void {
            if (Vec3.length(delta) < config.epsilon) return;
            var vel = delta;
            self.state.on_ground = false;
            
            for (0..config.max_slide_iterations) |_| {
                const len = Vec3.length(vel);
                if (len < config.epsilon) break;
                
                if (world.raycast(self.state.pos, Vec3.scale(vel, 1.0 / len), len)) |hit| {
                    self.state.pos = Vec3.add(self.state.pos, Vec3.scale(vel, @max(0, hit.distance - config.margin) / len));
                    
                    const dot_vel = Vec3.dot(vel, hit.normal);
                    if (dot_vel >= 0) break;
                    
                    if (hit.normal.data[1] > config.slope_limit) self.state.on_ground = true;
                    
                    vel = Vec3.sub(vel, Vec3.scale(hit.normal, dot_vel));
                    
                    const dot_state = Vec3.dot(self.state.vel, hit.normal);
                    if (dot_state < 0) self.state.vel = Vec3.sub(self.state.vel, Vec3.scale(hit.normal, dot_state));
                } else {
                    self.state.pos = Vec3.add(self.state.pos, vel);
                    break;
                }
            }
        }
    };
}