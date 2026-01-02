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
const brush_mod = @import("lib/brush.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = brush_mod.AABB;
const BrushWorld = world.BrushWorld;
const Vertex = extern struct { pos: [3]f32, col: [4]f32 };

// Components
const Transform = struct { pos: Vec3 = Vec3.zero() };
const Physics = struct { vel: Vec3 = Vec3.zero(), on_ground: bool = false };
const Input = struct { 
    yaw: f32 = 0, pitch: f32 = 0, mdx: f32 = 0, mdy: f32 = 0,
    keys: packed struct { w: bool = false, a: bool = false, s: bool = false, d: bool = false, sp: bool = false } = .{}, 
    lock: bool = false 
};
const Audio = struct { timer: f32 = 0, active: bool = false };

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
    
    // Collision
    brush_world: BrushWorld = undefined,
    
    // Geometry
    vertices: std.ArrayListUnmanaged(Vertex) = .{},
    indices: std.ArrayListUnmanaged(u16) = .{},
    
    fn init(self: *@This(), allocator: std.mem.Allocator) void {
        self.allocator = allocator;
        self.brush_world = BrushWorld.init(allocator);
        self.vertices = .{};
        self.indices = .{};
        
        // Init rendering
        var layout = sg.VertexLayoutState{};
        layout.attrs[0].format = .FLOAT3;
        layout.attrs[1].format = .FLOAT4;
        self.pipeline = sg.makePipeline(.{ 
            .shader = sg.makeShader(shader.cubeShaderDesc(sg.queryBackend())), 
            .layout = layout, .index_type = .UINT16, 
            .depth = .{ .compare = .LESS_EQUAL, .write_enabled = true }, 
            .cull_mode = .FRONT 
        });
        self.pass_action = .{ .colors = .{ .{ .load_action = .CLEAR, .clear_value = .{ .r = 0.15, .g = 0.15, .b = 0.18, .a = 1.0 } }, .{}, .{}, .{}, .{}, .{}, .{}, .{} } };
    }
    
    fn deinit(self: *@This()) void {
        self.brush_world.deinit();
        self.vertices.deinit(self.allocator);
        self.indices.deinit(self.allocator);
    }
    
    fn add_box(self: *@This(), pos: Vec3, size: Vec3, color: [4]f32) !void {
        // Add brush to collision world
        try self.brush_world.addBox(pos, size);
        
        // Add visual geometry
        const half = Vec3.scale(size, 0.5);
        const base = @as(u16, @intCast(self.vertices.items.len));
        const verts = [_][3]f32{ .{1,-1,-1}, .{-1,-1,-1}, .{-1,1,-1}, .{1,1,-1}, .{-1,-1,1}, .{1,-1,1}, .{1,1,1}, .{-1,1,1}, .{-1,-1,-1}, .{-1,-1,1}, .{-1,1,1}, .{-1,1,-1}, .{1,-1,1}, .{1,-1,-1}, .{1,1,-1}, .{1,1,1}, .{-1,-1,1}, .{-1,-1,-1}, .{1,-1,-1}, .{1,-1,1}, .{-1,1,-1}, .{-1,1,1}, .{1,1,1}, .{1,1,-1} };
        
        for (verts) |v| {
            var c = color;
            if (v[1] > 0) { c[0] *= 1.1; c[1] *= 1.1; c[2] *= 1.1; }
            else if (v[1] < 0) { c[0] *= 0.7; c[1] *= 0.7; c[2] *= 0.7; }
            try self.vertices.append(self.allocator, .{
                .pos = .{ pos.data[0] + v[0]*half.data[0], pos.data[1] + v[1]*half.data[1], pos.data[2] + v[2]*half.data[2] },
                .col = c,
            });
        }
        
        inline for (0..6) |f| for ([_]u16{0,1,2,0,2,3}) |i| try self.indices.append(self.allocator, base + @as(u16, @intCast(f*4 + i)));
    }
    
    fn build(self: *@This()) !void {
        try self.brush_world.build();
        
        if (self.bindings.vertex_buffers[0].id != 0) sg.destroyBuffer(self.bindings.vertex_buffers[0]);
        if (self.bindings.index_buffer.id != 0) sg.destroyBuffer(self.bindings.index_buffer);
        if (self.vertices.items.len > 0) {
            self.bindings.vertex_buffers[0] = sg.makeBuffer(.{ .data = .{ .ptr = self.vertices.items.ptr, .size = self.vertices.items.len * @sizeOf(Vertex) } });
            self.bindings.index_buffer = sg.makeBuffer(.{ .usage = .{ .index_buffer = true }, .data = .{ .ptr = self.indices.items.ptr, .size = self.indices.items.len * @sizeOf(u16) } });
        }
        self.vertex_count = @intCast(self.indices.items.len);
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

fn sys_physics(transforms: []Transform, physics: []Physics, inputs: []Input, audios: []Audio, resources: *GameResources) void {
    const dt = resources.delta_time;
    const size = Vec3.new(0.98, 1.8, 0.98);
    const player_aabb = AABB.fromCenterSize(Vec3.zero(), size);
    
    for (transforms, physics, inputs, audios) |*t, *p, *i, *a| {
        const fwd: f32 = if (i.keys.w) 1 else if (i.keys.s) -1 else 0;
        const side: f32 = if (i.keys.d) 1 else if (i.keys.a) -1 else 0;
        const jump = i.keys.sp;
        
        // Apply gravity
        p.vel.data[1] -= 12.0 * dt;
        
        // Jump if on ground
        if (jump and p.on_ground) { 
            p.vel.data[1] = 4.0; 
            a.timer = 0.15; 
            a.active = true; 
        }
        
        // Calculate desired movement direction
        var dir = Vec3.zero();
        if (side != 0) dir = Vec3.add(dir, Vec3.scale(Vec3.new(@cos(i.yaw), 0, @sin(i.yaw)), side));
        if (fwd != 0) dir = Vec3.add(dir, Vec3.scale(Vec3.new(@sin(i.yaw), 0, -@cos(i.yaw)), fwd));
        
        // Apply movement acceleration
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
        
        // Apply friction when on ground
        if (p.on_ground) {
            const speed = @sqrt(p.vel.data[0] * p.vel.data[0] + p.vel.data[2] * p.vel.data[2]);
            if (speed > 0.1) {
                const factor = @max(0, speed - @max(speed, 0.1) * 5.0 * dt) / speed;
                p.vel.data[0] *= factor; 
                p.vel.data[2] *= factor;
            } else { 
                p.vel.data[0] = 0; 
                p.vel.data[2] = 0; 
            }
        }
        
        // Simple movement with collision
        const delta = Vec3.scale(p.vel, dt);
        const old_pos = t.pos;
        const new_pos = resources.brush_world.movePlayer(t.pos, delta, player_aabb);
        
        // Only update velocity components that were blocked by collision
        const actual_delta = Vec3.sub(new_pos, old_pos);
        
        // If we couldn't move in a direction, zero that velocity component
        if (@abs(delta.data[0]) > 0.001 and @abs(actual_delta.data[0]) < 0.001) {
            p.vel.data[0] = 0;
        }
        if (@abs(delta.data[1]) > 0.001 and @abs(actual_delta.data[1]) < 0.001) {
            p.vel.data[1] = 0;
        }
        if (@abs(delta.data[2]) > 0.001 and @abs(actual_delta.data[2]) < 0.001) {
            p.vel.data[2] = 0;
        }
        
        // Check if we're on ground by testing slightly below
        const ground_test_pos = Vec3.new(new_pos.data[0], new_pos.data[1] - 0.1, new_pos.data[2]);
        const ground_aabb = AABB{
            .min = Vec3.add(ground_test_pos, player_aabb.min),
            .max = Vec3.add(ground_test_pos, player_aabb.max)
        };
        
        p.on_ground = resources.brush_world.testCollision(ground_aabb) and p.vel.data[1] <= 0.1;
        
        t.pos = new_pos;
    }
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
    
    // World
    store.resources.add_box(Vec3.new(0, -1, 0), Vec3.new(60, 2, 60), .{ 0.3, 0.3, 0.35, 1 }) catch {};
    const walls = [_][6]f32{ .{30,5,0,2,10,60}, .{-30,5,0,2,10,60}, .{0,5,30,60,10,2}, .{0,5,-30,60,10,2}, .{0,5,0,4,10,4} };
    for (walls) |w| store.resources.add_box(Vec3.new(w[0], w[1], w[2]), Vec3.new(w[3], w[4], w[5]), .{ 0.5, 0.4, 0.3, 1 }) catch {};
    for (0..4) |i| {
        const f = @as(f32, @floatFromInt(i));
        store.resources.add_box(Vec3.new(10 + f * 2, 0.5 + f, 10), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {};
    }
    store.resources.add_box(Vec3.new(18, 3.5, 10), Vec3.new(8, 1, 8), .{ 0.6, 0.2, 0.2, 1 }) catch {};
    store.resources.build() catch {};
    
    // Player
    store.resources.player_entity = store.create(Player, allocator, .{
        .transform = .{ .pos = Vec3.new(0, 10, -10) },
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