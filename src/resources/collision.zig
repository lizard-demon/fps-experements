const std = @import("std");
const math = @import("../lib/math.zig");
const config = @import("../lib/config.zig");
const bvh = @import("../lib/bvh.zig");
const Vec3 = math.Vec3;
const AABB = math.AABB;

// Internal BVH node structure for direct access
const Node = packed struct {
    first: u32, 
    count_and_axis: u32,
    min_x: f32, min_y: f32, min_z: f32, 
    max_x: f32, max_y: f32, max_z: f32,
};

pub const HullType = enum { point, standing };
pub const Collision = struct { normal: Vec3, distance: f32 };

pub const Capsule = struct {
    start: Vec3, end: Vec3, radius: f32,
    
    pub fn fromHull(pos: Vec3, hull_type: HullType) Capsule {
        return switch (hull_type) {
            .point => .{ .start = pos, .end = pos, .radius = 0.0 },
            .standing => .{ 
                .start = Vec3.add(pos, Vec3.new(0, -config.Physics.Hull.standing_height, 0)), 
                .end = Vec3.add(pos, Vec3.new(0, config.Physics.Hull.standing_height, 0)), 
                .radius = config.Physics.Hull.standing_radius 
            },
        };
    }
    
    pub inline fn center(self: Capsule) Vec3 {
        return Vec3.scale(Vec3.add(self.start, self.end), 0.5);
    }
    
    // Pre-compute AABB for better performance
    pub inline fn bounds(self: Capsule) AABB {
        const radius_vec = Vec3.new(self.radius, self.radius, self.radius);
        return AABB{
            .min = Vec3.sub(Vec3.min(self.start, self.end), radius_vec),
            .max = Vec3.add(Vec3.max(self.start, self.end), radius_vec),
        };
    }
};

pub const Plane = struct {
    normal: Vec3, distance: f32,
    
    pub fn distanceToPoint(self: Plane, point: Vec3) f32 { 
        return Vec3.dot(self.normal, point) + self.distance; 
    }
};

pub const Brush = struct {
    planes: []const Plane, 
    bounds: AABB,
    
    pub fn checkCapsule(self: Brush, capsule: Capsule) ?Collision {
        var closest_normal: ?Vec3 = null;
        var closest_distance: f32 = std.math.floatMax(f32);
        
        for (self.planes) |plane| {
            const distance = plane.distanceToPoint(capsule.center()) - capsule.radius;
            if (distance > math.EPSILON) return null;
            if (distance > -closest_distance) {
                closest_distance = -distance;
                closest_normal = plane.normal;
            }
        }
        
        return if (closest_normal) |normal| 
            Collision{ .normal = normal, .distance = closest_distance } 
        else null;
    }
    
    pub fn getBounds(self: Brush) AABB {
        return self.bounds;
    }
};

pub const World = struct {
    bvh: bvh.BVH(Brush),
    original_brushes: []const Brush,
    
    pub fn init(brushes: []const Brush, allocator: std.mem.Allocator) !World {
        return World{
            .original_brushes = brushes,
            .bvh = try bvh.BVH(Brush).init(brushes, allocator, Brush.getBounds),
        };
    }
    
    pub fn deinit(self: *World) void {
        self.bvh.deinit();
    }
    
    pub fn check(self: *const World, point: Vec3) ?Collision {
        const capsule = Capsule.fromHull(point, .standing);
        return self.checkCapsule(capsule);
    }
    
    pub fn checkCapsule(self: *const World, capsule: Capsule) ?Collision {
        if (self.bvh.items.len == 0) {
            for (self.original_brushes) |brush| if (brush.checkCapsule(capsule)) |collision| return collision;
            return null;
        }
        
        // Use pre-computed capsule bounds
        const capsule_bounds = capsule.bounds();
        
        var best_collision: ?Collision = null;
        var best_distance: f32 = -std.math.floatMax(f32);
        
        // Use pre-allocated stack for traversal - breadth-first layout benefits
        var stack_ptr: u32 = 0;
        var stack: [64]u32 = undefined; // Use config.World.BVH.max_stack_depth if available
        stack[0] = 0;
        stack_ptr = 1;
        
        while (stack_ptr > 0) {
            stack_ptr -= 1;
            const node_idx = stack[stack_ptr];
            
            if (node_idx >= self.bvh.nodes.items.len) continue;
            
            // Cast to our internal Node structure for direct access
            const node_data = @as(*const Node, @ptrCast(&self.bvh.nodes.items[node_idx]));
            
            const node_bounds = AABB{ 
                .min = Vec3.new(node_data.min_x, node_data.min_y, node_data.min_z), 
                .max = Vec3.new(node_data.max_x, node_data.max_y, node_data.max_z) 
            };
            
            if (!node_bounds.intersects(capsule_bounds)) continue;
            
            const is_leaf = (node_data.count_and_axis & 0x3) == 3;
            if (is_leaf) {
                const count = node_data.count_and_axis >> 2;
                const end_idx = @min(node_data.first + count, self.bvh.indices.items.len);
                for (node_data.first..end_idx) |i| {
                    if (self.bvh.items[self.bvh.indices.items[i]].checkCapsule(capsule)) |collision| {
                        if (collision.distance > best_distance) {
                            best_distance = collision.distance;
                            best_collision = collision;
                            
                            // Early termination: if we found a collision with positive distance,
                            // we can return immediately for most game physics use cases
                            if (collision.distance > 0) return collision;
                        }
                    }
                }
            } else {
                // In breadth-first layout, children are stored sequentially
                const left_child = node_data.first;
                const right_child = left_child + 1;
                
                // Add children to stack - breadth-first layout ensures they're cache-friendly
                if (right_child < self.bvh.nodes.items.len and stack_ptr < stack.len) {
                    stack[stack_ptr] = right_child;
                    stack_ptr += 1;
                }
                if (left_child < self.bvh.nodes.items.len and stack_ptr < stack.len) {
                    stack[stack_ptr] = left_child;
                    stack_ptr += 1;
                }
            }
        }
        
        return best_collision;
    }
};