const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;
const simgui = sokol.imgui;
const ig = @import("cimgui");

const math = @import("math");
const shader = @import("shader");
const bvh = @import("../primitives/bvh.zig");
const brush = @import("../primitives/brush.zig");
const obj = @import("models");
const texture_lib = @import("texture");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = math.AABB;
const TextureRegistry = texture_lib.Registry;

pub const Config = struct {
    // Rendering settings
    fov: f32 = 90.0,
    aspect_ratio: f32 = 1.33,
    near_plane: f32 = 0.1,
    far_plane: f32 = 500.0,
    
    // Clear color
    clear_color: [4]f32 = .{ 0.15, 0.15, 0.18, 1.0 },
    
    // Crosshair
    crosshair_size: f32 = 8.0,
    crosshair_color: u32 = 0xFF00FF00,
    
    // Culling
    enable_frustum_culling: bool = true,
    
    // Debug
    log_stats: bool = false,
};

// Simplified vertex format
const Vertex = extern struct { 
    pos: [3]f32, 
    col: [4]f32, 
    uv: [2]f32,
    atlas_uv: [4]f32, // Now stores offset + size: [x, y, width, height]
};

// Render batch for efficient drawing
const RenderBatch = struct {
    index_offset: u32,
    index_count: u32,
    bounds: AABB,
    
    fn render(self: @This()) void {
        sg.draw(self.index_offset, self.index_count, 1);
    }
};

// Frustum for culling
const Frustum = struct {
    planes: [6]Vec4,
    
    const Vec4 = struct { x: f32, y: f32, z: f32, w: f32 };
    
    fn fromMVP(mvp: Mat4) Frustum {
        const m = mvp.data;
        return Frustum{
            .planes = .{
                .{ .x = m[3] + m[0], .y = m[7] + m[4], .z = m[11] + m[8], .w = m[15] + m[12] }, // Left
                .{ .x = m[3] - m[0], .y = m[7] - m[4], .z = m[11] - m[8], .w = m[15] - m[12] }, // Right
                .{ .x = m[3] + m[1], .y = m[7] + m[5], .z = m[11] + m[9], .w = m[15] + m[13] }, // Bottom
                .{ .x = m[3] - m[1], .y = m[7] - m[5], .z = m[11] - m[9], .w = m[15] - m[13] }, // Top
                .{ .x = m[3] + m[2], .y = m[7] + m[6], .z = m[11] + m[10], .w = m[15] + m[14] }, // Near
                .{ .x = m[3] - m[2], .y = m[7] - m[6], .z = m[11] - m[10], .w = m[15] - m[14] }, // Far
            },
        };
    }
    
    fn testAABB(self: @This(), bounds: AABB) bool {
        for (self.planes) |plane| {
            const normal = Vec3.new(plane.x, plane.y, plane.z);
            const length = Vec3.length(normal);
            if (length < 0.0001) continue;
            
            const norm = Vec3.scale(normal, 1.0 / length);
            const dist = plane.w / length;
            
            // Test positive vertex (farthest point in plane normal direction)
            const positive_vertex = Vec3.new(
                if (norm.data[0] >= 0) bounds.max.data[0] else bounds.min.data[0],
                if (norm.data[1] >= 0) bounds.max.data[1] else bounds.min.data[1],
                if (norm.data[2] >= 0) bounds.max.data[2] else bounds.min.data[2],
            );
            
            if (Vec3.dot(norm, positive_vertex) + dist < 0) {
                return false; // Outside this plane
            }
        }
        return true; // Inside all planes
    }
};

// Simple stats for debugging
const Stats = struct {
    total_batches: u32 = 0,
    rendered_batches: u32 = 0,
    culled_batches: u32 = 0,
    
    fn reset(self: *@This()) void {
        self.* = .{};
    }
    
    fn log(self: @This()) void {
        if (self.total_batches > 0) {
            const cull_rate = (@as(f32, @floatFromInt(self.culled_batches)) / @as(f32, @floatFromInt(self.total_batches))) * 100.0;
            std.log.info("Render: {}/{} batches ({d:.1}% culled)", .{ self.rendered_batches, self.total_batches, cull_rate });
        }
    }
};

pub fn Renderer(comptime config: Config) type {
    return struct {
        const Self = @This();
        
        allocator: std.mem.Allocator,
        
        // GPU resources
        pipeline: sg.Pipeline = undefined,
        bindings: sg.Bindings = undefined,
        pass_action: sg.PassAction = undefined,
        
        // Geometry data
        vertices: std.ArrayListUnmanaged(Vertex) = .{},
        indices: std.ArrayListUnmanaged(u16) = .{},
        batches: std.ArrayListUnmanaged(RenderBatch) = .{},
        
        // Acceleration structure
        batch_bvh: ?bvh.BVH(RenderBatch) = null,
        
        // Texture system
        texture_registry: TextureRegistry,
        
        // Model system
        // (removed - models loaded directly)
        
        // Debug stats
        stats: Stats = .{},
        frame_count: u32 = 0,
        
        pub fn init(allocator: std.mem.Allocator) !Self {
            var texture_registry = TextureRegistry.init(allocator);
            texture_registry.loadDirectory("assets/textures") catch |err| {
                std.log.err("Failed to load textures: {}", .{err});
                return err;
            };
            
            var renderer = Self{ 
                .allocator = allocator,
                .texture_registry = texture_registry,
            };
            renderer.initPipeline();
            return renderer;
        }
        
        pub fn deinit(self: *Self) void {
            self.vertices.deinit(self.allocator);
            self.indices.deinit(self.allocator);
            self.batches.deinit(self.allocator);
            self.texture_registry.deinit();
            if (self.batch_bvh) |*bvh_instance| {
                bvh_instance.deinit();
            }
        }
        
        fn initPipeline(self: *Self) void {
            var layout = sg.VertexLayoutState{};
            layout.attrs[0].format = .FLOAT3; // position
            layout.attrs[1].format = .FLOAT4; // color
            layout.attrs[2].format = .FLOAT2; // uv
            layout.attrs[3].format = .FLOAT4; // atlas_uv (offset + size)
            
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
                        .r = config.clear_color[0], 
                        .g = config.clear_color[1], 
                        .b = config.clear_color[2], 
                        .a = config.clear_color[3] 
                    } }, 
                    .{}, .{}, .{}, .{}, .{}, .{}, .{} 
                } 
            };
        }
        
        pub fn addBrush(self: *Self, b: brush.Brush, color: [4]f32, texture_name: []const u8) !void {
            const atlas_info = self.texture_registry.getAtlasUV(texture_name);
            const index_offset = @as(u32, @intCast(self.indices.items.len));
            
            // Generate mesh from brush
            var brush_mesh = try b.generateMesh(self.allocator);
            defer brush_mesh.deinit();
            
            if (brush_mesh.vertices.len == 0) return;
            
            const base_vertex = @as(u16, @intCast(self.vertices.items.len));
            
            // Add vertices
            for (brush_mesh.vertices) |vertex| {
                try self.vertices.append(self.allocator, .{ 
                    .pos = .{ vertex.data[0], vertex.data[1], vertex.data[2] }, 
                    .col = color,
                    .uv = .{ 0.0, 0.0 },
                    .atlas_uv = atlas_info,
                });
            }
            
            // Add indices
            for (brush_mesh.indices) |index| {
                try self.indices.append(self.allocator, base_vertex + index);
            }
            
            const index_count = @as(u32, @intCast(brush_mesh.indices.len));
            if (index_count > 0) {
                try self.batches.append(self.allocator, .{
                    .index_offset = index_offset,
                    .index_count = index_count,
                    .bounds = b.bounds,
                });
            }
        }
        
        pub fn addObjModelDirect(self: *Self, mesh: *const obj.Mesh, color: [4]f32, position: Vec3, target_size: f32, texture_name: []const u8) !void {
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
            
            // Create transform: translate to origin, scale, then translate to position
            const center_to_origin = Mat4.translate(Vec3.scale(center, -1.0));
            const scale_transform = Mat4.scale(Vec3.new(scale_factor, scale_factor, scale_factor));
            const translate_to_position = Mat4.translate(position);
            
            const transform = Mat4.mul(Mat4.mul(translate_to_position, scale_transform), center_to_origin);
            
            // Apply transform and add to renderer
            const atlas_info = self.texture_registry.getAtlasUV(texture_name);
            const index_offset = @as(u32, @intCast(self.indices.items.len));
            const base_vertex = @as(u16, @intCast(self.vertices.items.len));
            
            // Calculate transformed bounds
            var transformed_bounds_min = Vec3.new(std.math.floatMax(f32), std.math.floatMax(f32), std.math.floatMax(f32));
            var transformed_bounds_max = Vec3.new(-std.math.floatMax(f32), -std.math.floatMax(f32), -std.math.floatMax(f32));
            
            // Add transformed vertices
            for (mesh.vertices) |vertex| {
                const transformed = transform.mulVec3(vertex);
                transformed_bounds_min = Vec3.min(transformed_bounds_min, transformed);
                transformed_bounds_max = Vec3.max(transformed_bounds_max, transformed);
                try self.vertices.append(self.allocator, .{ 
                    .pos = .{ transformed.data[0], transformed.data[1], transformed.data[2] }, 
                    .col = color,
                    .uv = .{ 0.0, 0.0 },
                    .atlas_uv = atlas_info,
                });
            }
            
            // Add indices
            for (mesh.indices) |index| {
                try self.indices.append(self.allocator, base_vertex + @as(u16, @intCast(index)));
            }
            
            const index_count = @as(u32, @intCast(mesh.indices.len));
            if (index_count > 0) {
                try self.batches.append(self.allocator, .{
                    .index_offset = index_offset,
                    .index_count = index_count,
                    .bounds = AABB.new(transformed_bounds_min, transformed_bounds_max),
                });
            }
        }
        
        pub fn buildBuffers(self: *Self) !void {
            // Clean up old buffers
            if (self.bindings.vertex_buffers[0].id != 0) sg.destroyBuffer(self.bindings.vertex_buffers[0]);
            if (self.bindings.index_buffer.id != 0) sg.destroyBuffer(self.bindings.index_buffer);
            
            // Create new buffers
            if (self.vertices.items.len > 0) {
                self.bindings.vertex_buffers[0] = sg.makeBuffer(.{ 
                    .data = sg.asRange(self.vertices.items)
                });
                self.bindings.index_buffer = sg.makeBuffer(.{ 
                    .usage = .{ .index_buffer = true }, 
                    .data = sg.asRange(self.indices.items)
                });
            }
            
            // Build BVH for culling
            if (self.batches.items.len > 0) {
                self.batch_bvh = try bvh.BVH(RenderBatch).init(
                    self.batches.items, 
                    self.allocator, 
                    getBatchBounds
                );
            }
            
            // Set up texture binding
            self.bindings.views[0] = self.texture_registry.atlas_view;
            self.bindings.samplers[0] = sg.makeSampler(.{
                .min_filter = .NEAREST,
                .mag_filter = .NEAREST,
                .wrap_u = .REPEAT,
                .wrap_v = .REPEAT,
            });
        }
        
        fn getBatchBounds(batch: RenderBatch) AABB {
            return batch.bounds;
        }
        
        pub fn render(self: *Self, view: Mat4) void {
            const mvp = Mat4.mul(
                math.perspective(config.fov, config.aspect_ratio, config.near_plane, config.far_plane), 
                view
            );
            
            self.stats.reset();
            self.stats.total_batches = @intCast(self.batches.items.len);
            
            sg.beginPass(.{ .action = self.pass_action, .swapchain = sokol.glue.swapchain() });
            sg.applyPipeline(self.pipeline);
            sg.applyBindings(self.bindings);
            sg.applyUniforms(0, sg.asRange(&mvp));
            
            if (config.enable_frustum_culling and self.batch_bvh != null) {
                const frustum = Frustum.fromMVP(mvp);
                self.renderWithCulling(&self.batch_bvh.?, frustum);
            } else {
                // Render all batches without culling
                for (self.batches.items) |batch| {
                    batch.render();
                    self.stats.rendered_batches += 1;
                }
            }
            
            // Log stats periodically
            if (config.log_stats) {
                self.frame_count += 1;
                if (self.frame_count % 60 == 0) {
                    self.stats.log();
                }
            }
        }
        
        fn renderWithCulling(self: *Self, batch_bvh: *const bvh.BVH(RenderBatch), frustum: Frustum) void {
            if (batch_bvh.nodes.len == 0) {
                // No BVH, test all batches individually
                for (self.batches.items) |batch| {
                    if (frustum.testAABB(batch.bounds)) {
                        batch.render();
                        self.stats.rendered_batches += 1;
                    } else {
                        self.stats.culled_batches += 1;
                    }
                }
                return;
            }
            
            // BVH traversal with frustum culling
            var stack: [64]u32 = undefined;
            var stack_ptr: u32 = 1;
            stack[0] = 0;
            
            while (stack_ptr > 0) {
                stack_ptr -= 1;
                const node_idx = stack[stack_ptr];
                if (node_idx >= batch_bvh.nodes.len) continue;
                
                const node_bounds = batch_bvh.nodes.get(node_idx).bounds;
                const node_type = batch_bvh.nodes.get(node_idx).node_type;
                
                // Early rejection: if node is outside frustum, skip entire subtree
                if (!frustum.testAABB(node_bounds)) {
                    // Count all batches in this subtree as culled
                    if (node_type == .leaf) {
                        const count = batch_bvh.nodes.get(node_idx).count;
                        self.stats.culled_batches += count;
                    }
                    continue;
                }
                
                if (node_type == .leaf) {
                    // Render batches in this leaf
                    const first = batch_bvh.nodes.get(node_idx).first;
                    const count = batch_bvh.nodes.get(node_idx).count;
                    const end_idx = @min(first + count, batch_bvh.indices.items.len);
                    
                    for (first..end_idx) |i| {
                        const batch = batch_bvh.items[batch_bvh.indices.items[i]];
                        // Additional per-batch test for tighter culling
                        if (frustum.testAABB(batch.bounds)) {
                            batch.render();
                            self.stats.rendered_batches += 1;
                        } else {
                            self.stats.culled_batches += 1;
                        }
                    }
                } else {
                    // Add children to stack
                    const left_child = batch_bvh.nodes.get(node_idx).first;
                    const right_child = left_child + 1;
                    
                    if (right_child < batch_bvh.nodes.len and stack_ptr < stack.len) {
                        stack[stack_ptr] = right_child;
                        stack_ptr += 1;
                    }
                    if (left_child < batch_bvh.nodes.len and stack_ptr < stack.len) {
                        stack[stack_ptr] = left_child;
                        stack_ptr += 1;
                    }
                }
            }
        }
        
        pub fn renderCrosshair(self: *const Self) void {
            _ = self;
            const cx = @as(f32, @floatFromInt(sokol.app.width())) * 0.5;
            const cy = @as(f32, @floatFromInt(sokol.app.height())) * 0.5;
            const dl = ig.igGetBackgroundDrawList();
            
            ig.ImDrawList_AddLine(dl, 
                .{ .x = cx - config.crosshair_size, .y = cy }, 
                .{ .x = cx + config.crosshair_size, .y = cy }, 
                config.crosshair_color
            );
            ig.ImDrawList_AddLine(dl, 
                .{ .x = cx, .y = cy - config.crosshair_size }, 
                .{ .x = cx, .y = cy + config.crosshair_size }, 
                config.crosshair_color
            );
        }
    };
}