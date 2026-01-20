const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;
const simgui = sokol.imgui;
const ig = @import("cimgui");

const math = @import("math");
const shader = @import("shader");
const bvh = @import("../lib/bvh.zig");
const brush = @import("../lib/brush.zig");
const obj = @import("../lib/obj.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = math.AABB;

pub const Config = struct {
    // Rendering settings
    fov: f32 = 90.0,
    aspect_ratio: f32 = 1.33,
    near_plane: f32 = 0.1,
    far_plane: f32 = 500.0,
    
    // Clear color
    clear_color: struct {
        r: f32 = 0.15,
        g: f32 = 0.15,
        b: f32 = 0.18,
        a: f32 = 1.0,
    } = .{},
    
    // Crosshair
    crosshair_size: f32 = 8.0,
    crosshair_color: u32 = 0xFF00FF00,
    
    // Default brush color
    brush_color: [4]f32 = .{ 0.4, 0.4, 0.4, 1.0 },
};

pub fn Renderer(comptime config: Config) type {
    return struct {
        const Self = @This();
        
        // Vertex format for rendering
        const Vertex = extern struct { pos: [3]f32, col: [4]f32 };
        
        // Renderable object with bounds for BVH acceleration
        const RenderObject = struct {
            index_offset: u32,
            index_count: u32,
            bounds: AABB,
        };
        
        allocator: std.mem.Allocator,
        
        // GPU resources
        pipeline: sg.Pipeline = undefined,
        bindings: sg.Bindings = undefined,
        pass_action: sg.PassAction = undefined,
        
        // Geometry data
        vertices: std.ArrayListUnmanaged(Vertex) = .{},
        indices: std.ArrayListUnmanaged(u16) = .{},
        
        // BVH acceleration for frustum culling
        render_objects: std.ArrayListUnmanaged(RenderObject) = .{},
        render_bvh: ?bvh.BVH(RenderObject) = null,
        
        pub fn init(allocator: std.mem.Allocator) Self {
            var renderer = Self{ .allocator = allocator };
            renderer.initializePipeline();
            return renderer;
        }
        
        pub fn deinit(self: *Self) void {
            self.vertices.deinit(self.allocator);
            self.indices.deinit(self.allocator);
            self.render_objects.deinit(self.allocator);
            if (self.render_bvh) |*bvh_instance| {
                bvh_instance.deinit();
            }
        }
        
        fn initializePipeline(self: *Self) void {
            var layout = sg.VertexLayoutState{};
            layout.attrs[0].format = .FLOAT3;
            layout.attrs[1].format = .FLOAT4;
            
            self.pipeline = sg.makePipeline(.{ 
                .shader = sg.makeShader(shader.shaderShaderDesc(sg.queryBackend())), 
                .layout = layout, 
                .index_type = .UINT16, 
                .depth = .{ .compare = .LESS_EQUAL, .write_enabled = true }, 
                .cull_mode = .NONE 
            });
            
            self.pass_action = .{ 
                .colors = .{ 
                    .{ .load_action = .CLEAR, .clear_value = .{ 
                        .r = config.clear_color.r, 
                        .g = config.clear_color.g, 
                        .b = config.clear_color.b, 
                        .a = config.clear_color.a 
                    } }, 
                    .{}, .{}, .{}, .{}, .{}, .{}, .{} 
                } 
            };
        }
        
        pub fn addBrush(self: *Self, b: brush.Brush, color: [4]f32) !void {
            const index_offset = @as(u32, @intCast(self.indices.items.len));
            
            try self.generateBrushMesh(b, color);
            
            const index_count = @as(u32, @intCast(self.indices.items.len)) - index_offset;
            if (index_count > 0) {
                try self.render_objects.append(self.allocator, .{
                    .index_offset = index_offset,
                    .index_count = index_count,
                    .bounds = b.bounds,
                });
            }
        }
        
        pub fn addObjModelAutoScale(self: *Self, obj_path: []const u8, color: [4]f32, position: Vec3, target_size: f32) !void {
            var mesh = try obj.loadOBJ(obj_path, self.allocator);
            defer mesh.deinit();
            
            // Calculate original bounds
            var bounds_min = Vec3.new(std.math.floatMax(f32), std.math.floatMax(f32), std.math.floatMax(f32));
            var bounds_max = Vec3.new(-std.math.floatMax(f32), -std.math.floatMax(f32), -std.math.floatMax(f32));
            
            for (mesh.vertices) |vertex| {
                bounds_min = Vec3.min(bounds_min, vertex);
                bounds_max = Vec3.max(bounds_max, vertex);
            }
            
            // Calculate center and scale
            const center = Vec3.scale(Vec3.add(bounds_min, bounds_max), 0.5);
            const size = Vec3.sub(bounds_max, bounds_min);
            const max_dimension = @max(@max(size.data[0], size.data[1]), size.data[2]);
            const scale_factor = if (max_dimension > 0) target_size / max_dimension else 1.0;
            
            std.log.info("Auto-scaling OBJ: {s}", .{obj_path});
            std.log.info("  Original size: {d:.1}, scale factor: {d:.3}", .{ max_dimension, scale_factor });
            
            // Create transform: translate to origin, scale, then translate to position
            const center_to_origin = Mat4.translate(Vec3.scale(center, -1.0));
            const scale_transform = Mat4.scale(Vec3.new(scale_factor, scale_factor, scale_factor));
            const translate_to_position = Mat4.translate(position);
            
            const transform = Mat4.mul(Mat4.mul(translate_to_position, scale_transform), center_to_origin);
            
            try self.addObjModel(obj_path, color, transform);
        }
        
        pub fn addObjModel(self: *Self, obj_path: []const u8, color: [4]f32, transform: Mat4) !void {
            var mesh = try obj.loadOBJ(obj_path, self.allocator);
            defer mesh.deinit();
            
            const index_offset = @as(u32, @intCast(self.indices.items.len));
            const base_vertex = @as(u16, @intCast(self.vertices.items.len));
            
            // Calculate original bounds to understand the model scale
            var bounds_min = Vec3.new(std.math.floatMax(f32), std.math.floatMax(f32), std.math.floatMax(f32));
            var bounds_max = Vec3.new(-std.math.floatMax(f32), -std.math.floatMax(f32), -std.math.floatMax(f32));
            
            for (mesh.vertices) |vertex| {
                bounds_min = Vec3.min(bounds_min, vertex);
                bounds_max = Vec3.max(bounds_max, vertex);
            }
            
            // Log model bounds for debugging
            std.log.info("Loading OBJ: {s}", .{obj_path});
            std.log.info("  Original bounds: ({d:.1},{d:.1},{d:.1}) to ({d:.1},{d:.1},{d:.1})", .{
                bounds_min.data[0], bounds_min.data[1], bounds_min.data[2],
                bounds_max.data[0], bounds_max.data[1], bounds_max.data[2]
            });
            
            const size = Vec3.sub(bounds_max, bounds_min);
            const max_dimension = @max(@max(size.data[0], size.data[1]), size.data[2]);
            std.log.info("  Size: ({d:.1},{d:.1},{d:.1}), max dimension: {d:.1}", .{
                size.data[0], size.data[1], size.data[2], max_dimension
            });
            
            // Calculate transformed bounds for BVH
            var transformed_bounds_min = Vec3.new(std.math.floatMax(f32), std.math.floatMax(f32), std.math.floatMax(f32));
            var transformed_bounds_max = Vec3.new(-std.math.floatMax(f32), -std.math.floatMax(f32), -std.math.floatMax(f32));
            
            // Add transformed vertices
            for (mesh.vertices) |vertex| {
                const transformed = transform.mulVec3(vertex);
                transformed_bounds_min = Vec3.min(transformed_bounds_min, transformed);
                transformed_bounds_max = Vec3.max(transformed_bounds_max, transformed);
                try self.vertices.append(self.allocator, .{ 
                    .pos = .{ transformed.data[0], transformed.data[1], transformed.data[2] }, 
                    .col = color 
                });
            }
            
            std.log.info("  Transformed bounds: ({d:.1},{d:.1},{d:.1}) to ({d:.1},{d:.1},{d:.1})", .{
                transformed_bounds_min.data[0], transformed_bounds_min.data[1], transformed_bounds_min.data[2],
                transformed_bounds_max.data[0], transformed_bounds_max.data[1], transformed_bounds_max.data[2]
            });
            
            // Add indices
            for (mesh.indices) |index| {
                try self.indices.append(self.allocator, base_vertex + @as(u16, @intCast(index)));
            }
            
            const index_count = @as(u32, @intCast(mesh.indices.len));
            if (index_count > 0) {
                try self.render_objects.append(self.allocator, .{
                    .index_offset = index_offset,
                    .index_count = index_count,
                    .bounds = AABB.new(transformed_bounds_min, transformed_bounds_max),
                });
            }
        }
        
        fn generateBrushMesh(self: *Self, b: brush.Brush, color: [4]f32) !void {
            // Find all vertices by intersecting three planes
            var hull_vertices = std.ArrayListUnmanaged(Vec3){};
            defer hull_vertices.deinit(self.allocator);
            
            for (0..b.planes.len()) |i| {
                for (i + 1..b.planes.len()) |j| {
                    for (j + 1..b.planes.len()) |k| {
                        if (self.intersectThreePlanes(b.planes, i, j, k)) |vertex| {
                            // Check if vertex is inside brush and not duplicate
                            if (self.isVertexInsideBrush(vertex, b.planes) and !self.isDuplicateVertex(vertex, hull_vertices.items)) {
                                try hull_vertices.append(self.allocator, vertex);
                            }
                        }
                    }
                }
            }
            
            if (hull_vertices.items.len < 4) return;
            
            // Generate convex hull faces
            const hull_faces = try self.generateConvexHullFaces(hull_vertices.items);
            defer self.allocator.free(hull_faces);
            
            const base_vertex = @as(u16, @intCast(self.vertices.items.len));
            
            // Add vertices
            for (hull_vertices.items) |v| {
                try self.vertices.append(self.allocator, .{ 
                    .pos = .{ v.data[0], v.data[1], v.data[2] }, 
                    .col = color 
                });
            }
            
            // Add indices
            for (hull_faces) |face| {
                try self.indices.append(self.allocator, base_vertex + face[0]);
                try self.indices.append(self.allocator, base_vertex + face[1]);
                try self.indices.append(self.allocator, base_vertex + face[2]);
            }
        }
        
        fn intersectThreePlanes(self: *Self, planes: brush.Planes, i: usize, j: usize, k: usize) ?Vec3 {
            _ = self;
            const n1 = planes.normals[i];
            const n2 = planes.normals[j];
            const n3 = planes.normals[k];
            const d1 = planes.distances[i];
            const d2 = planes.distances[j];
            const d3 = planes.distances[k];
            
            const det = Vec3.dot(n1, Vec3.cross(n2, n3));
            if (@abs(det) < 0.001) return null;
            
            const c1 = Vec3.cross(n2, n3);
            const c2 = Vec3.cross(n3, n1);
            const c3 = Vec3.cross(n1, n2);
            
            return Vec3.scale(Vec3.add(Vec3.add(Vec3.scale(c1, -d1), Vec3.scale(c2, -d2)), Vec3.scale(c3, -d3)), 1.0 / det);
        }
        
        fn isVertexInsideBrush(self: *Self, vertex: Vec3, planes: brush.Planes) bool {
            _ = self;
            for (0..planes.len()) |plane_idx| {
                if (planes.distanceToPoint(plane_idx, vertex) > 0.001) return false;
            }
            return true;
        }
        
        fn isDuplicateVertex(self: *Self, vertex: Vec3, existing: []Vec3) bool {
            _ = self;
            for (existing) |existing_vertex| {
                if (Vec3.dist(vertex, existing_vertex) < 0.01) return true;
            }
            return false;
        }
        
        fn generateConvexHullFaces(self: *Self, vertices: []Vec3) ![][3]u16 {
            if (vertices.len < 4) return &[_][3]u16{};
            
            var faces = std.ArrayListUnmanaged([3]u16){};
            
            // Simple approach: test all triangles, keep those that are hull faces
            for (0..vertices.len) |i| {
                for (i + 1..vertices.len) |j| {
                    for (j + 1..vertices.len) |k| {
                        const v0 = vertices[i];
                        const v1 = vertices[j]; 
                        const v2 = vertices[k];
                        
                        const edge1 = Vec3.sub(v1, v0);
                        const edge2 = Vec3.sub(v2, v0);
                        const normal = Vec3.cross(edge1, edge2);
                        
                        if (Vec3.length(normal) < 0.001) continue;
                        
                        // Check if all other vertices are on one side of this triangle
                        var is_hull_face = true;
                        var side_sign: ?f32 = null;
                        
                        for (vertices, 0..) |test_vertex, idx| {
                            if (idx == i or idx == j or idx == k) continue;
                            
                            const dot = Vec3.dot(normal, Vec3.sub(test_vertex, v0));
                            if (@abs(dot) > 0.001) {
                                if (side_sign == null) {
                                    side_sign = dot;
                                } else if (side_sign.? * dot < 0) {
                                    is_hull_face = false;
                                    break;
                                }
                            }
                        }
                        
                        if (is_hull_face) {
                            const face: [3]u16 = if ((side_sign orelse 1) > 0) 
                                .{ @intCast(i), @intCast(k), @intCast(j) }
                            else 
                                .{ @intCast(i), @intCast(j), @intCast(k) };
                            try faces.append(self.allocator, face);
                        }
                    }
                }
            }
            
            return try faces.toOwnedSlice(self.allocator);
        }
        
        pub fn buildBuffers(self: *Self) !void {
            // Clean up old buffers
            if (self.bindings.vertex_buffers[0].id != 0) sg.destroyBuffer(self.bindings.vertex_buffers[0]);
            if (self.bindings.index_buffer.id != 0) sg.destroyBuffer(self.bindings.index_buffer);
            
            // Create new buffers
            if (self.vertices.items.len > 0) {
                self.bindings.vertex_buffers[0] = sg.makeBuffer(.{ 
                    .data = .{ 
                        .ptr = self.vertices.items.ptr, 
                        .size = self.vertices.items.len * @sizeOf(Vertex) 
                    } 
                });
                self.bindings.index_buffer = sg.makeBuffer(.{ 
                    .usage = .{ .index_buffer = true }, 
                    .data = .{ 
                        .ptr = self.indices.items.ptr, 
                        .size = self.indices.items.len * @sizeOf(u16) 
                    } 
                });
            }
            
            // Build BVH for frustum culling
            if (self.render_objects.items.len > 0) {
                self.render_bvh = try bvh.BVH(RenderObject).init(
                    self.render_objects.items, 
                    self.allocator, 
                    getRenderObjectBounds
                );
            }
        }
        
        fn getRenderObjectBounds(render_obj: RenderObject) AABB {
            return render_obj.bounds;
        }
        
        pub fn render(self: *const Self, view: Mat4) void {
            const mvp = Mat4.mul(
                math.perspective(config.fov, config.aspect_ratio, config.near_plane, config.far_plane), 
                view
            );
            
            sg.beginPass(.{ .action = self.pass_action, .swapchain = sokol.glue.swapchain() });
            sg.applyPipeline(self.pipeline);
            sg.applyBindings(self.bindings);
            sg.applyUniforms(0, sg.asRange(&mvp));
            
            if (self.render_bvh) |*bvh_instance| {
                self.renderWithFrustumCulling(bvh_instance, mvp);
            } else {
                // Fallback: render all indices
                const total_indices = @as(u32, @intCast(self.indices.items.len));
                sg.draw(0, total_indices, 1);
            }
        }
        
        fn renderWithFrustumCulling(self: *const Self, bvh_instance: *const bvh.BVH(RenderObject), mvp: Mat4) void {
            _ = self;
            const frustum = extractFrustumPlanes(mvp);
            
            var stack: [64]u32 = undefined;
            var stack_ptr: u32 = 1;
            stack[0] = 0;
            
            while (stack_ptr > 0) {
                stack_ptr -= 1;
                const node_idx = stack[stack_ptr];
                if (node_idx >= bvh_instance.nodes.len) continue;
                
                const node_bounds = bvh_instance.nodes.get(node_idx).bounds;
                const node_type = bvh_instance.nodes.get(node_idx).node_type;
                
                if (!isAABBInFrustum(node_bounds, frustum)) continue;
                
                if (node_type == .leaf) {
                    const first = bvh_instance.nodes.get(node_idx).first;
                    const count = bvh_instance.nodes.get(node_idx).count;
                    const end_idx = @min(first + count, bvh_instance.indices.items.len);
                    
                    for (first..end_idx) |i| {
                        const render_obj = bvh_instance.items[bvh_instance.indices.items[i]];
                        sg.draw(render_obj.index_offset, render_obj.index_count, 1);
                    }
                } else {
                    const left_child = bvh_instance.nodes.get(node_idx).first;
                    const right_child = left_child + 1;
                    
                    if (right_child < bvh_instance.nodes.len and stack_ptr < stack.len) {
                        stack[stack_ptr] = right_child;
                        stack_ptr += 1;
                    }
                    if (left_child < bvh_instance.nodes.len and stack_ptr < stack.len) {
                        stack[stack_ptr] = left_child;
                        stack_ptr += 1;
                    }
                }
            }
        }
        
        // Frustum culling implementation
        const FrustumPlane = struct { 
            normal: Vec3, 
            distance: f32,
            
            inline fn testPoint(self: @This(), point: Vec3) f32 {
                return Vec3.dot(self.normal, point) + self.distance;
            }
        };
        const Frustum = [6]FrustumPlane;
        
        fn extractFrustumPlanes(mvp: Mat4) Frustum {
            const m = mvp.data;
            
            var planes: [6]FrustumPlane = undefined;
            
            // Extract frustum planes from clip-space MVP matrix (column-major)
            planes[0] = normalizePlane(m[3] + m[0], m[7] + m[4], m[11] + m[8], m[15] + m[12]); // Left
            planes[1] = normalizePlane(m[3] - m[0], m[7] - m[4], m[11] - m[8], m[15] - m[12]); // Right
            planes[2] = normalizePlane(m[3] + m[1], m[7] + m[5], m[11] + m[9], m[15] + m[13]); // Bottom
            planes[3] = normalizePlane(m[3] - m[1], m[7] - m[5], m[11] - m[9], m[15] - m[13]); // Top
            planes[4] = normalizePlane(m[3] + m[2], m[7] + m[6], m[11] + m[10], m[15] + m[14]); // Near
            planes[5] = normalizePlane(m[3] - m[2], m[7] - m[6], m[11] - m[10], m[15] - m[14]); // Far
            
            return planes;
        }
        
        inline fn normalizePlane(a: f32, b: f32, c: f32, d: f32) FrustumPlane {
            const normal = Vec3.new(a, b, c);
            const length = Vec3.length(normal);
            if (length > 0.0001) {
                return .{
                    .normal = Vec3.scale(normal, 1.0 / length),
                    .distance = d / length,
                };
            } else {
                return .{ .normal = Vec3.new(1, 0, 0), .distance = 0 };
            }
        }
        
        fn isAABBInFrustum(bounds: AABB, frustum: Frustum) bool {
            for (frustum) |plane| {
                const positive_vertex = Vec3.new(
                    if (plane.normal.data[0] >= 0) bounds.max.data[0] else bounds.min.data[0],
                    if (plane.normal.data[1] >= 0) bounds.max.data[1] else bounds.min.data[1],
                    if (plane.normal.data[2] >= 0) bounds.max.data[2] else bounds.min.data[2],
                );
                
                if (plane.testPoint(positive_vertex) < 0) {
                    return false;
                }
            }
            return true;
        }
        
        pub fn renderCrosshair(self: *const Self) void {
            _ = self;
            const cx = @as(f32, @floatFromInt(sokol.app.width())) * 0.5;
            const cy = @as(f32, @floatFromInt(sokol.app.height())) * 0.5;
            const crosshair_size = config.crosshair_size;
            const dl = ig.igGetBackgroundDrawList();
            
            ig.ImDrawList_AddLine(dl, 
                .{ .x = cx - crosshair_size, .y = cy }, 
                .{ .x = cx + crosshair_size, .y = cy }, 
                config.crosshair_color
            );
            ig.ImDrawList_AddLine(dl, 
                .{ .x = cx, .y = cy - crosshair_size }, 
                .{ .x = cx, .y = cy + crosshair_size }, 
                config.crosshair_color
            );
        }
    };
}