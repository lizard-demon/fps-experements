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
            .cull_mode = .BACK 
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
        const brush = try self.brush_world.addBoxAndReturn(pos, size);
        
        // Add visual geometry using the brush bounds to ensure perfect alignment
        try self.add_brush_visual(brush, color);
    }
    
    fn add_slope(self: *@This(), center: Vec3, size: Vec3, angle_degrees: f32, color: [4]f32) !void {
        const angle_rad = angle_degrees * std.math.pi / 180.0;
        const half_size = Vec3.scale(size, 0.5);
        
        // Create planes for a sloped brush (wedge shape)
        var planes = try self.allocator.alloc(brush_mod.Plane, 5);
        defer self.allocator.free(planes);
        
        // Bottom plane (y = min.y): normal (0,-1,0) pointing outward (downward)
        planes[0] = brush_mod.Plane.new(Vec3.new(0, -1, 0), center.data[1] - half_size.data[1]);
        
        // Sloped surface - normal points outward from the slope
        const slope_normal = Vec3.normalize(Vec3.new(@sin(angle_rad), @cos(angle_rad), 0));
        const slope_point = Vec3.new(center.data[0], center.data[1] + half_size.data[1], center.data[2]);
        planes[1] = brush_mod.Plane.new(slope_normal, -Vec3.dot(slope_normal, slope_point));
        
        // Side walls (normals point outward)
        planes[2] = brush_mod.Plane.new(Vec3.new(-1, 0, 0), center.data[0] - half_size.data[0]);
        planes[3] = brush_mod.Plane.new(Vec3.new(1, 0, 0), -(center.data[0] + half_size.data[0]));
        
        // Back wall (normal points outward)
        planes[4] = brush_mod.Plane.new(Vec3.new(0, 0, -1), center.data[2] - half_size.data[2]);
        
        const brush = try brush_mod.Brush.init(self.allocator, planes);
        try self.brush_world.addBrush(brush);
        
        // Add visual geometry that matches the slope exactly
        try self.add_slope_visual(center, size, angle_degrees, color);
    }
    
    // Generate visual mesh from brush planes using a simple approach
    fn add_brush_visual(self: *@This(), brush: brush_mod.Brush, color: [4]f32) !void {
        const min = brush.bounds.min;
        const max = brush.bounds.max;
        const base = @as(u16, @intCast(self.vertices.items.len));
        
        const vertices = [_]Vec3{
            Vec3.new(min.data[0], min.data[1], min.data[2]), Vec3.new(max.data[0], min.data[1], min.data[2]),
            Vec3.new(max.data[0], min.data[1], max.data[2]), Vec3.new(min.data[0], min.data[1], max.data[2]),
            Vec3.new(min.data[0], max.data[1], min.data[2]), Vec3.new(max.data[0], max.data[1], min.data[2]),
            Vec3.new(max.data[0], max.data[1], max.data[2]), Vec3.new(min.data[0], max.data[1], max.data[2]),
        };
        
        for (vertices) |v| {
            var c = color;
            if (v.data[1] <= (min.data[1] + max.data[1]) * 0.5) { c[0] *= 0.7; c[1] *= 0.7; c[2] *= 0.7; }
            else { c[0] *= 1.1; c[1] *= 1.1; c[2] *= 1.1; }
            try self.vertices.append(self.allocator, .{ .pos = .{ v.data[0], v.data[1], v.data[2] }, .col = c });
        }
        
        // Clockwise winding for back-face culling
        const faces = [_][3]u16{
            .{ 0, 2, 1 }, .{ 0, 3, 2 }, .{ 4, 5, 6 }, .{ 4, 6, 7 },
            .{ 0, 1, 5 }, .{ 0, 5, 4 }, .{ 2, 7, 6 }, .{ 2, 6, 3 },
            .{ 0, 4, 7 }, .{ 0, 7, 3 }, .{ 1, 2, 6 }, .{ 1, 6, 5 },
        };
        
        for (faces) |face| {
            for (face) |idx| try self.indices.append(self.allocator, base + idx);
        }
    }
    
    fn add_slope_visual(self: *@This(), center: Vec3, size: Vec3, angle_degrees: f32, color: [4]f32) !void {
        const angle_rad = angle_degrees * std.math.pi / 180.0;
        const half = Vec3.scale(size, 0.5);
        const base = @as(u16, @intCast(self.vertices.items.len));
        
        const sin_a = @sin(angle_rad);
        const cos_a = @cos(angle_rad);
        
        // Calculate geometry based on collision planes
        const bottom_y = center.data[1] - half.data[1];
        const slope_d = sin_a * center.data[0] + cos_a * (center.data[1] + half.data[1]);
        
        const left_x = center.data[0] - half.data[0];
        const right_x = center.data[0] + half.data[0];
        const front_z = center.data[2] - half.data[2];
        const back_z = center.data[2] + half.data[2];
        
        const left_slope_y = (slope_d - sin_a * left_x) / cos_a;
        const right_slope_y = (slope_d - sin_a * right_x) / cos_a;
        
        const vertices = [_]Vec3{
            Vec3.new(left_x, bottom_y, front_z),   Vec3.new(right_x, bottom_y, front_z),
            Vec3.new(right_x, bottom_y, back_z),   Vec3.new(left_x, bottom_y, back_z),
            Vec3.new(left_x, left_slope_y, front_z),   Vec3.new(right_x, right_slope_y, front_z),
            Vec3.new(right_x, right_slope_y, back_z),  Vec3.new(left_x, left_slope_y, back_z),
        };
        
        for (vertices) |v| {
            var c = color;
            if (v.data[1] <= center.data[1]) { c[0] *= 0.7; c[1] *= 0.7; c[2] *= 0.7; }
            else { c[0] *= 1.1; c[1] *= 1.1; c[2] *= 1.1; }
            try self.vertices.append(self.allocator, .{ .pos = .{ v.data[0], v.data[1], v.data[2] }, .col = c });
        }
        
        // Clockwise winding for back-face culling
        const faces = [_][3]u16{
            .{ 0, 2, 1 }, .{ 0, 3, 2 }, .{ 4, 5, 6 }, .{ 4, 6, 7 },
            .{ 0, 1, 5 }, .{ 0, 5, 4 }, .{ 2, 7, 6 }, .{ 2, 6, 3 },
            .{ 0, 4, 7 }, .{ 0, 7, 3 }, .{ 1, 2, 6 }, .{ 1, 6, 5 },
        };
        
        for (faces) |face| {
            for (face) |idx| try self.indices.append(self.allocator, base + idx);
        }
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
    const tolerance = 0.01;
    
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
        const old_pos = t.pos;
        const new_pos = resources.brush_world.movePlayer(t.pos, delta, player_aabb);
        
        // Update velocity based on actual movement (for sliding and bouncing)
        const actual_delta = Vec3.sub(new_pos, old_pos);
        
        // Only zero velocity if we completely failed to move in that direction
        // This allows for sliding along walls and surfaces
        if (@abs(delta.data[0]) > tolerance and @abs(actual_delta.data[0]) < tolerance * 0.1) {
            p.vel.data[0] *= 0.1; // Reduce but don't completely zero for sliding
        }
        if (@abs(delta.data[1]) > tolerance and @abs(actual_delta.data[1]) < tolerance * 0.1) {
            p.vel.data[1] = 0; // Always zero Y velocity when hitting ceiling/floor
        }
        if (@abs(delta.data[2]) > tolerance and @abs(actual_delta.data[2]) < tolerance * 0.1) {
            p.vel.data[2] *= 0.1; // Reduce but don't completely zero for sliding
        }
        
        // Improved ground detection - check slightly below current position
        const ground_check_distance = 0.05;
        const ground_test_pos = Vec3.new(new_pos.data[0], new_pos.data[1] - ground_check_distance, new_pos.data[2]);
        const ground_aabb = AABB{
            .min = Vec3.add(ground_test_pos, player_aabb.min),
            .max = Vec3.add(ground_test_pos, player_aabb.max)
        };
        
        // We're on ground if there's collision below us and we're not moving up fast
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
    
    // Clean test world with just essentials
    
    // Ground plane
    store.resources.add_box(Vec3.new(0, -1, 0), Vec3.new(40, 2, 40), .{ 0.3, 0.3, 0.35, 1 }) catch {};
    
    // Boundary walls
    store.resources.add_box(Vec3.new(20, 5, 0), Vec3.new(2, 10, 40), .{ 0.5, 0.4, 0.3, 1 }) catch {}; // Right wall
    store.resources.add_box(Vec3.new(-20, 5, 0), Vec3.new(2, 10, 40), .{ 0.5, 0.4, 0.3, 1 }) catch {}; // Left wall
    store.resources.add_box(Vec3.new(0, 5, 20), Vec3.new(40, 10, 2), .{ 0.5, 0.4, 0.3, 1 }) catch {}; // Front wall
    store.resources.add_box(Vec3.new(0, 5, -20), Vec3.new(40, 10, 2), .{ 0.5, 0.4, 0.3, 1 }) catch {}; // Back wall
    
    // Simple steps for testing step-up mechanics
    store.resources.add_box(Vec3.new(-10, 0.5, -10), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {}; // Step 1
    store.resources.add_box(Vec3.new(-10, 1.5, -6), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {}; // Step 2
    store.resources.add_box(Vec3.new(-10, 2.5, -2), Vec3.new(4, 1, 4), .{ 0.6, 0.2, 0.2, 1 }) catch {}; // Step 3
    
    // One test slope for slope mechanics
    store.resources.add_slope(Vec3.new(10, 0, 0), Vec3.new(8, 6, 8), 30.0, .{ 0.2, 0.6, 0.2, 1 }) catch {};
    
    store.resources.build() catch {};
    
    // Player - spawn in the center of the simplified world
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