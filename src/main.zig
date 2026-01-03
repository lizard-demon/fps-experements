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
const collision = @import("lib/collision.zig");
const physics = @import("lib/physics.zig");
const mesh = @import("lib/mesh.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;

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
    
    // World
    world: collision.World = undefined,
    mesh_builder: mesh.MeshBuilder = undefined,
    
    fn init(self: *@This(), allocator: std.mem.Allocator) void {
        self.allocator = allocator;
        self.world = collision.World.init(allocator);
        self.mesh_builder = mesh.MeshBuilder.init(allocator);
        
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
    
    fn addBox(self: *@This(), pos: Vec3, size: Vec3, color: [4]f32) !void {
        try self.world.addBox(pos, size);
        try self.mesh_builder.addBox(pos, size, color);
    }
    
    fn addSlope(self: *@This(), center: Vec3, size: Vec3, angle_degrees: f32, color: [4]f32) !void {
        const angle_rad = angle_degrees * std.math.pi / 180.0;
        const half_size = Vec3.scale(size, 0.5);
        
        var planes = try self.allocator.alloc(collision.Plane, 5);
        defer self.allocator.free(planes);
        
        planes[0] = collision.Plane.new(Vec3.new(0, -1, 0), center.data[1] - half_size.data[1]);
        
        const slope_normal = Vec3.normalize(Vec3.new(@sin(angle_rad), @cos(angle_rad), 0));
        const slope_point = Vec3.new(center.data[0], center.data[1] + half_size.data[1], center.data[2]);
        planes[1] = collision.Plane.new(slope_normal, -Vec3.dot(slope_normal, slope_point));
        
        planes[2] = collision.Plane.new(Vec3.new(-1, 0, 0), center.data[0] - half_size.data[0]);
        planes[3] = collision.Plane.new(Vec3.new(1, 0, 0), -(center.data[0] + half_size.data[0]));
        planes[4] = collision.Plane.new(Vec3.new(0, 0, -1), center.data[2] - half_size.data[2]);
        
        const brush = try self.world.factory.createFromPlanes(planes);
        try self.world.addBrush(brush);
        
        // Convert to mesh planes
        var mesh_planes = try self.allocator.alloc(mesh.Plane, planes.len);
        defer self.allocator.free(mesh_planes);
        for (planes, 0..) |p, i| {
            mesh_planes[i] = mesh.Plane{ .normal = p.normal, .distance = p.distance };
        }
        
        const mesh_brush = mesh.Brush.new(mesh_planes);
        try self.mesh_builder.addBrush(mesh_brush, color);
    }
    
    fn build(self: *@This()) !void {
        if (self.bindings.vertex_buffers[0].id != 0) sg.destroyBuffer(self.bindings.vertex_buffers[0]);
        if (self.bindings.index_buffer.id != 0) sg.destroyBuffer(self.bindings.index_buffer);
        if (self.mesh_builder.vertices.items.len > 0) {
            self.bindings.vertex_buffers[0] = sg.makeBuffer(.{ .data = .{ .ptr = self.mesh_builder.vertices.items.ptr, .size = self.mesh_builder.vertices.items.len * @sizeOf(mesh.Vertex) } });
            self.bindings.index_buffer = sg.makeBuffer(.{ .usage = .{ .index_buffer = true }, .data = .{ .ptr = self.mesh_builder.indices.items.ptr, .size = self.mesh_builder.indices.items.len * @sizeOf(u16) } });
        }
        self.vertex_count = @intCast(self.mesh_builder.indices.items.len);
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
    
    // Build world
    store.resources.addBox(Vec3.new(0, -1, 0), Vec3.new(40, 2, 40), .{ 0.3, 0.3, 0.35, 1 }) catch {};
    store.resources.addBox(Vec3.new(20, 5, 0), Vec3.new(2, 10, 40), .{ 0.5, 0.4, 0.3, 1 }) catch {};
    store.resources.addBox(Vec3.new(-20, 5, 0), Vec3.new(2, 10, 40), .{ 0.5, 0.4, 0.3, 1 }) catch {};
    store.resources.addBox(Vec3.new(0, 5, 20), Vec3.new(40, 10, 2), .{ 0.5, 0.4, 0.3, 1 }) catch {};
    store.resources.addBox(Vec3.new(0, 5, -20), Vec3.new(40, 10, 2), .{ 0.5, 0.4, 0.3, 1 }) catch {};
    
    store.resources.addBox(Vec3.new(-10, 0.5, -10), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {};
    store.resources.addBox(Vec3.new(-10, 1.5, -6), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {};
    store.resources.addBox(Vec3.new(-10, 2.5, -2), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {};
    
    store.resources.addSlope(Vec3.new(10, 0, 0), Vec3.new(8, 6, 8), 30.0, .{ 0.2, 0.6, 0.2, 1 }) catch {};
    
    store.resources.build() catch {};
    
    // Create player
    store.resources.player_entity = store.create(Player, allocator, .{
        .transform = .{ .pos = Vec3.new(0, 2, 0) },
        .physics = .{}, .input = .{}, .audio = .{},
    }) catch unreachable;
    
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
    _ = simgui.handleEvent(e.*);
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
    if (!initialized) { for (0..@intCast(n * c)) |i| buf[i] = 0; return; }
    
    const sources = store.registry.players.items(.audio);
    for (0..@intCast(n)) |f| {
        var sample: f32 = 0;
        for (sources) |*s| {
            if (s.active and s.timer > 0) {
                const t = 1.0 - s.timer / 0.15;
                s.timer -= 1.0 / 44100.0;
                sample += @sin((0.15 - s.timer) * 500.0 * std.math.pi) * @exp(-t * 8.0) * 0.3;
                if (s.timer <= 0) s.active = false;
            }
        }
        for (0..@as(usize, @intCast(c))) |ch| buf[f * @as(usize, @intCast(c)) + ch] = sample;
    }
}

pub fn main() void {
    sapp.run(.{ .init_cb = init, .frame_cb = frame, .cleanup_cb = cleanup, .event_cb = event, .width = 1024, .height = 768, .window_title = "FPS" });
}