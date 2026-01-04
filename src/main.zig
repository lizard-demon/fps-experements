const std = @import("std");
const sokol = @import("sokol");
const sapp = sokol.app;
const sg = sokol.gfx;
const saudio = sokol.audio;
const simgui = sokol.imgui;
const ig = @import("cimgui");

const math = @import("lib/math.zig");
const shader = @import("shader/cube.glsl.zig");
const ecs = @import("lib/ecs.zig");
const world = @import("lib/world.zig");
const physics = @import("lib/physics.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = math.AABB;

// Components
const Transform = physics.Transform;
const Physics = physics.Physics;
const Input = physics.Input;
const Audio = physics.Audio;

// Archetype
const Player = struct { transform: Transform, physics: Physics, input: Input, audio: Audio };

// Registry
const Registry = struct { players: ecs.List(Player) };

// Resources
const GameResources = struct {
    delta_time: f32 = 0,
    allocator: std.mem.Allocator,
    player_entity: ecs.Entity(Player) = undefined,
    
    // Rendering
    pipeline: sg.Pipeline = undefined,
    bindings: sg.Bindings = undefined,
    pass_action: sg.PassAction = undefined,
    vertex_count: u32 = 0,
    
    // World - now just holds brush data
    world: world.World = undefined,
    mesh_builder: world.MeshBuilder = undefined,
    
    // Static brush storage for complex demo geometry
    brush_planes: [80]world.Plane = undefined, // Increased for complex shapes
    brushes: [8]world.Brush = undefined,
    
    fn init(self: *@This(), allocator: std.mem.Allocator) void {
        self.allocator = allocator;
        self.mesh_builder = world.MeshBuilder.init(allocator);
        
        // Simplified demo world for debugging
        var plane_idx: usize = 0;
        var brush_idx: usize = 0;
        
        // 1. Ground platform (large base)
        self.brush_planes[plane_idx + 0] = .{ .normal = Vec3.new( 1,  0,  0), .distance = -15 };  // Right
        self.brush_planes[plane_idx + 1] = .{ .normal = Vec3.new(-1,  0,  0), .distance = -15 };  // Left
        self.brush_planes[plane_idx + 2] = .{ .normal = Vec3.new( 0,  1,  0), .distance = -0.5 }; // Top
        self.brush_planes[plane_idx + 3] = .{ .normal = Vec3.new( 0, -1,  0), .distance = -2 };   // Bottom
        self.brush_planes[plane_idx + 4] = .{ .normal = Vec3.new( 0,  0,  1), .distance = -15 };  // Front
        self.brush_planes[plane_idx + 5] = .{ .normal = Vec3.new( 0,  0, -1), .distance = -15 };  // Back
        self.brushes[brush_idx] = .{ .planes = self.brush_planes[plane_idx..plane_idx+6], .bounds = AABB.new(Vec3.new(-15, -2, -15), Vec3.new(15, 0.5, 15)) };
        plane_idx += 6; brush_idx += 1;
        
        // 2. Simple test box
        const box_center = Vec3.new(5, 1, 0);
        const box_size = Vec3.new(2, 2, 2);
        self.brush_planes[plane_idx + 0] = .{ .normal = Vec3.new( 1,  0,  0), .distance = -(box_center.data[0] + box_size.data[0]/2) };
        self.brush_planes[plane_idx + 1] = .{ .normal = Vec3.new(-1,  0,  0), .distance = box_center.data[0] - box_size.data[0]/2 };
        self.brush_planes[plane_idx + 2] = .{ .normal = Vec3.new( 0,  1,  0), .distance = -(box_center.data[1] + box_size.data[1]/2) };
        self.brush_planes[plane_idx + 3] = .{ .normal = Vec3.new( 0, -1,  0), .distance = box_center.data[1] - box_size.data[1]/2 };
        self.brush_planes[plane_idx + 4] = .{ .normal = Vec3.new( 0,  0,  1), .distance = -(box_center.data[2] + box_size.data[2]/2) };
        self.brush_planes[plane_idx + 5] = .{ .normal = Vec3.new( 0,  0, -1), .distance = box_center.data[2] - box_size.data[2]/2 };
        self.brushes[brush_idx] = .{ .planes = self.brush_planes[plane_idx..plane_idx+6], .bounds = AABB.new(Vec3.sub(box_center, Vec3.scale(box_size, 0.5)), Vec3.add(box_center, Vec3.scale(box_size, 0.5))) };
        plane_idx += 6; brush_idx += 1;
        
        // 3. Simple ramp (wedge)
        const ramp_center = Vec3.new(-5, 1, 0);
        self.brush_planes[plane_idx + 0] = .{ .normal = Vec3.new( 0, -1,  0), .distance = 0.5 };   // Bottom (on ground)
        self.brush_planes[plane_idx + 1] = .{ .normal = Vec3.new( 1,  0,  0), .distance = -(ramp_center.data[0] + 1) };  // Right
        self.brush_planes[plane_idx + 2] = .{ .normal = Vec3.new(-1,  0,  0), .distance = ramp_center.data[0] - 1 };     // Left
        self.brush_planes[plane_idx + 3] = .{ .normal = Vec3.new( 0,  0,  1), .distance = -(ramp_center.data[2] + 1) };  // Front
        self.brush_planes[plane_idx + 4] = .{ .normal = Vec3.new( 0,  0, -1), .distance = ramp_center.data[2] - 1 };     // Back
        // Fixed sloped plane: goes from high on left to low on right
        const slope_normal = Vec3.normalize(Vec3.new(1, 1, 0));  // 45 degree slope
        const slope_point = Vec3.new(-6, 2.5, 0);  // High point on left side
        self.brush_planes[plane_idx + 5] = .{ .normal = slope_normal, .distance = -Vec3.dot(slope_normal, slope_point) };
        self.brushes[brush_idx] = .{ .planes = self.brush_planes[plane_idx..plane_idx+6], .bounds = AABB.new(Vec3.new(ramp_center.data[0] - 1, 0.5, ramp_center.data[2] - 1), Vec3.new(ramp_center.data[0] + 1, 2.5, ramp_center.data[2] + 1)) };
        plane_idx += 6; brush_idx += 1;
        
        // Initialize world with all brushes
        self.world = world.World.init(self.brushes[0..brush_idx], allocator) catch world.World{ 
            .brushes = self.brushes[0..brush_idx], 
            .bvh_nodes = &[_]world.BVHNode{}, 
            .brush_indices = &[_]u32{}, 
            .allocator = allocator 
        };
        
        // Debug: Print brush information
        std.log.info("Created {} brushes:", .{brush_idx});
        for (self.brushes[0..brush_idx], 0..) |brush, i| {
            std.log.info("  Brush {}: {} planes, bounds: ({d:.1},{d:.1},{d:.1}) to ({d:.1},{d:.1},{d:.1})", .{
                i, brush.planes.len,
                brush.bounds.min.data[0], brush.bounds.min.data[1], brush.bounds.min.data[2],
                brush.bounds.max.data[0], brush.bounds.max.data[1], brush.bounds.max.data[2]
            });
        }
        
        // Init rendering
        var layout = sg.VertexLayoutState{};
        layout.attrs[0].format = .FLOAT3;
        layout.attrs[1].format = .FLOAT4;
        self.pipeline = sg.makePipeline(.{ 
            .shader = sg.makeShader(shader.cubeShaderDesc(sg.queryBackend())), 
            .layout = layout, .index_type = .UINT16, 
            .depth = .{ .compare = .LESS_EQUAL, .write_enabled = true }, 
            .cull_mode = .NONE 
        });
        self.pass_action = .{ .colors = .{ .{ .load_action = .CLEAR, .clear_value = .{ .r = 0.15, .g = 0.15, .b = 0.18, .a = 1.0 } }, .{}, .{}, .{}, .{}, .{}, .{}, .{} } };
    }
    
    fn deinit(self: *@This()) void {
        self.world.deinit();
        self.mesh_builder.deinit();
    }
    
    fn build(self: *@This()) !void {
        if (self.bindings.vertex_buffers[0].id != 0) sg.destroyBuffer(self.bindings.vertex_buffers[0]);
        if (self.bindings.index_buffer.id != 0) sg.destroyBuffer(self.bindings.index_buffer);
        if (self.mesh_builder.vertices.items.len > 0) {
            self.bindings.vertex_buffers[0] = sg.makeBuffer(.{ .data = .{ .ptr = self.mesh_builder.vertices.items.ptr, .size = self.mesh_builder.vertices.items.len * @sizeOf(world.Vertex) } });
            self.bindings.index_buffer = sg.makeBuffer(.{ .usage = .{ .index_buffer = true }, .data = .{ .ptr = self.mesh_builder.indices.items.ptr, .size = self.mesh_builder.indices.items.len * @sizeOf(u16) } });
        }
        // Use saturating cast to prevent overflow
        self.vertex_count = std.math.cast(u32, self.mesh_builder.indices.items.len) orelse std.math.maxInt(u32);
    }
    
    fn render(self: *const @This(), view: Mat4) void {
        const mvp = Mat4.mul(math.perspective(90, 1.33, 0.1, 200), view);
        sg.beginPass(.{ .action = self.pass_action, .swapchain = sokol.glue.swapchain() });
        sg.applyPipeline(self.pipeline);
        sg.applyBindings(self.bindings);
        sg.applyUniforms(0, sg.asRange(&mvp));
        sg.draw(0, self.vertex_count, 1);
    }
};

// Systems
fn sys_input(inputs: []Input, resources: *GameResources) void {
    _ = resources;
    for (inputs) |*i| {
        i.yaw += i.mdx * 0.002;
        i.pitch = std.math.clamp(i.pitch + i.mdy * 0.002, -1.5, 1.5);
        i.mdx = 0; i.mdy = 0;
    }
}

fn sys_physics(transforms: []Transform, physics_comps: []Physics, inputs: []Input, audios: []Audio, resources: *GameResources) void {
    physics.update(transforms, physics_comps, inputs, audios, &resources.world, resources.delta_time);
}

fn sys_render(transforms: []Transform, inputs: []Input, resources: *GameResources) void {
    for (transforms, inputs) |t, i| {
        const eye = Vec3.add(t.pos, Vec3.new(0, 0.6, 0));
        const cy, const sy = .{ @cos(i.yaw), @sin(i.yaw) };
        const cp, const sp = .{ @cos(i.pitch), @sin(i.pitch) };
        const view = Mat4{ .data = .{ cy, sy*sp, -sy*cp, 0, 0, cp, sp, 0, sy, -cy*sp, cy*cp, 0, -eye.data[0]*cy - eye.data[2]*sy, -eye.data[0]*sy*sp - eye.data[1]*cp + eye.data[2]*cy*sp, eye.data[0]*sy*cp - eye.data[1]*sp - eye.data[2]*cy*cp, 1 } };
        resources.render(view);
        break;
    }
}

// Global state
var store: ecs.Store(Registry, GameResources) = undefined;
var initialized: bool = false;

export fn init() void {
    const allocator = std.heap.c_allocator;
    sg.setup(.{ .environment = sokol.glue.environment() });
    saudio.setup(.{ .stream_cb = audio });
    simgui.setup(.{});
    
    store = ecs.Store(Registry, GameResources){ .registry = .{ .players = .{} }, .resources = undefined };
    store.resources.init(allocator);
    
    // Build visual representation using actual brush geometry
    for (store.resources.brushes) |brush| {
        const vertex_count_before = store.resources.mesh_builder.vertices.items.len;
        store.resources.mesh_builder.addBrush(brush, .{ 0.4, 0.4, 0.4, 1 }) catch continue;
        const vertex_count_after = store.resources.mesh_builder.vertices.items.len;
        std.log.info("Brush mesh: {} vertices added (total: {})", .{ vertex_count_after - vertex_count_before, vertex_count_after });
    }
    
    store.resources.build() catch |err| {
        std.log.err("Failed to build mesh: {}", .{err});
    };
    
    // Create player
    store.resources.player_entity = store.create(Player, allocator, .{
        .transform = .{ .pos = Vec3.new(0, 2, 0) },
        .physics = .{}, .input = .{}, .audio = .{},
    }) catch |err| {
        std.log.err("Failed to create player entity: {}", .{err});
        return; // Graceful failure - game will not initialize but won't crash
    };
    
    initialized = true;
}

export fn frame() void {
    store.resources.delta_time = @as(f32, @floatCast(sapp.frameDuration()));
    store.run(sys_input);
    store.run(sys_physics);
    
    simgui.newFrame(.{ .width = sapp.width(), .height = sapp.height(), .delta_time = sapp.frameDuration() });
    store.run(sys_render);
    
    const cx = @as(f32, @floatFromInt(sapp.width())) * 0.5;
    const cy = @as(f32, @floatFromInt(sapp.height())) * 0.5;
    const dl = ig.igGetBackgroundDrawList();
    ig.ImDrawList_AddLine(dl, .{ .x = cx - 8, .y = cy }, .{ .x = cx + 8, .y = cy }, 0xFF00FF00);
    ig.ImDrawList_AddLine(dl, .{ .x = cx, .y = cy - 8 }, .{ .x = cx, .y = cy + 8 }, 0xFF00FF00);
    
    simgui.render(); sg.endPass(); sg.commit();
}

export fn cleanup() void {
    initialized = false;
    saudio.shutdown();
    store.resources.deinit();
    store.registry.players.deinit(store.resources.allocator);
    simgui.shutdown(); sg.shutdown();
}

export fn event(e: [*c]const sapp.Event) void {
    // Validate event pointer
    if (e == null) return;
    
    _ = simgui.handleEvent(e.*);
    
    // Ensure player entity exists before accessing input
    if (!initialized) return;
    const inp = store.get(store.resources.player_entity.id, *Input) orelse return;
    
    const d = e.*.type == .KEY_DOWN;
    
    switch (e.*.type) {
        .KEY_DOWN, .KEY_UP => switch (e.*.key_code) {
            .W => inp.keys.w = d, .A => inp.keys.a = d, .S => inp.keys.s = d, .D => inp.keys.d = d, .SPACE => inp.keys.sp = d,
            .ESCAPE => if (d and inp.lock) { inp.lock = false; sapp.showMouse(true); sapp.lockMouse(false); },
            else => {},
        },
        .MOUSE_DOWN => if (e.*.mouse_button == .LEFT and !inp.lock) { inp.lock = true; sapp.showMouse(false); sapp.lockMouse(true); },
        .MOUSE_MOVE => if (inp.lock) { inp.mdx += e.*.mouse_dx; inp.mdy += e.*.mouse_dy; },
        else => {},
    }
}

fn audio(buf: [*c]f32, n: i32, c: i32) callconv(.c) void {
    // Validate parameters to prevent overflow and invalid access
    if (n <= 0 or c <= 0) return;
    
    // Check for potential overflow in multiplication
    const n_usize: usize = @intCast(n);
    const c_usize: usize = @intCast(c);
    const total_samples = std.math.mul(usize, n_usize, c_usize) catch return;
    
    if (!initialized) { 
        for (0..total_samples) |i| buf[i] = 0; 
        return; 
    }
    
    const sources = store.registry.players.items(.audio);
    for (0..n_usize) |f| {
        var sample: f32 = 0;
        for (sources) |*s| {
            if (s.active and s.timer > 0) {
                const sound_duration = 0.15; // Should match physics config
                const t = 1.0 - s.timer / sound_duration;
                s.timer -= 1.0 / 44100.0;
                sample += @sin((sound_duration - s.timer) * 500.0 * std.math.pi) * @exp(-t * 8.0) * 0.3;
                if (s.timer <= 0) s.active = false;
            }
        }
        for (0..c_usize) |ch| buf[f * c_usize + ch] = sample;
    }
}

pub fn main() void {
    sapp.run(.{ .init_cb = init, .frame_cb = frame, .cleanup_cb = cleanup, .event_cb = event, .width = 1024, .height = 768, .window_title = "FPS" });
}