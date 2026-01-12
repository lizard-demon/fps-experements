const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;
const simgui = sokol.imgui;
const ig = @import("cimgui");

const math = @import("../lib/math.zig");
const shader = @import("../shader/cube.glsl.zig");
const world = @import("world.zig");
const config = @import("../lib/config.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;

pub const Renderer = struct {
    allocator: std.mem.Allocator,
    
    // Rendering
    pipeline: sg.Pipeline = undefined,
    bindings: sg.Bindings = undefined,
    pass_action: sg.PassAction = undefined,
    vertex_count: u32 = 0,
    
    // World mesh building
    mesh_builder: world.MeshBuilder = undefined,
    
    pub fn init(allocator: std.mem.Allocator) Renderer {
        var renderer = Renderer{
            .allocator = allocator,
            .mesh_builder = world.MeshBuilder.init(allocator),
        };
        
        renderer.initializePipeline();
        return renderer;
    }
    
    pub fn deinit(self: *Renderer) void {
        self.mesh_builder.deinit();
    }
    
    fn initializePipeline(self: *Renderer) void {
        var layout = sg.VertexLayoutState{};
        layout.attrs[0].format = .FLOAT3;
        layout.attrs[1].format = .FLOAT4;
        
        self.pipeline = sg.makePipeline(.{ 
            .shader = sg.makeShader(shader.cubeShaderDesc(sg.queryBackend())), 
            .layout = layout, 
            .index_type = .UINT16, 
            .depth = .{ .compare = .LESS_EQUAL, .write_enabled = true }, 
            .cull_mode = .NONE 
        });
        
        self.pass_action = .{ 
            .colors = .{ 
                .{ .load_action = .CLEAR, .clear_value = .{ 
                    .r = config.Rendering.ClearColor.r, 
                    .g = config.Rendering.ClearColor.g, 
                    .b = config.Rendering.ClearColor.b, 
                    .a = config.Rendering.ClearColor.a 
                } }, 
                .{}, .{}, .{}, .{}, .{}, .{}, .{} 
            } 
        };
    }
    
    pub fn buildWorldMesh(self: *Renderer, brushes: []const world.Brush) !void {
        // Build visual representation using brushes for rendering
        for (brushes) |brush| {
            self.mesh_builder.addBrush(brush, .{ 
                config.Rendering.BrushColor.r, 
                config.Rendering.BrushColor.g, 
                config.Rendering.BrushColor.b, 
                config.Rendering.BrushColor.a 
            }) catch continue;
        }
        
        try self.buildBuffers();
    }
    
    pub fn buildBuffers(self: *Renderer) !void {
        if (self.bindings.vertex_buffers[0].id != 0) sg.destroyBuffer(self.bindings.vertex_buffers[0]);
        if (self.bindings.index_buffer.id != 0) sg.destroyBuffer(self.bindings.index_buffer);
        
        if (self.mesh_builder.vertices.items.len > 0) {
            self.bindings.vertex_buffers[0] = sg.makeBuffer(.{ 
                .data = .{ 
                    .ptr = self.mesh_builder.vertices.items.ptr, 
                    .size = self.mesh_builder.vertices.items.len * @sizeOf(world.Vertex) 
                } 
            });
            self.bindings.index_buffer = sg.makeBuffer(.{ 
                .usage = .{ .index_buffer = true }, 
                .data = .{ 
                    .ptr = self.mesh_builder.indices.items.ptr, 
                    .size = self.mesh_builder.indices.items.len * @sizeOf(u16) 
                } 
            });
        }
        
        // Use saturating cast to prevent overflow
        self.vertex_count = std.math.cast(u32, self.mesh_builder.indices.items.len) orelse std.math.maxInt(u32);
    }
    
    pub fn render(self: *const Renderer, view: Mat4) void {
        const mvp = Mat4.mul(
            math.perspective(
                config.Rendering.fov, 
                config.Rendering.aspect_ratio, 
                config.Rendering.near_plane, 
                config.Rendering.far_plane
            ), 
            view
        );
        
        sg.beginPass(.{ .action = self.pass_action, .swapchain = sokol.glue.swapchain() });
        sg.applyPipeline(self.pipeline);
        sg.applyBindings(self.bindings);
        sg.applyUniforms(0, sg.asRange(&mvp));
        sg.draw(0, self.vertex_count, 1);
    }
    
    pub fn renderCrosshair() void {
        const cx = @as(f32, @floatFromInt(sokol.app.width())) * 0.5;
        const cy = @as(f32, @floatFromInt(sokol.app.height())) * 0.5;
        const crosshair_size = config.Rendering.crosshair_size;
        const dl = ig.igGetBackgroundDrawList();
        
        ig.ImDrawList_AddLine(dl, 
            .{ .x = cx - crosshair_size, .y = cy }, 
            .{ .x = cx + crosshair_size, .y = cy }, 
            config.Rendering.CrosshairColor.rgba
        );
        ig.ImDrawList_AddLine(dl, 
            .{ .x = cx, .y = cy - crosshair_size }, 
            .{ .x = cx, .y = cy + crosshair_size }, 
            config.Rendering.CrosshairColor.rgba
        );
    }
};