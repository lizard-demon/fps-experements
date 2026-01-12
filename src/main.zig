const std = @import("std");
const sokol = @import("sokol");
const sapp = sokol.app;
const sg = sokol.gfx;
const saudio = sokol.audio;
const simgui = sokol.imgui;
const ig = @import("cimgui");

const math = @import("lib/math.zig");
const ecs = @import("lib/ecs.zig");
const collision = @import("resources/collision.zig");
const mesh = @import("resources/mesh.zig");
const physics = @import("resources/physics.zig");
const renderer = @import("resources/render.zig");
const config = @import("lib/config.zig");

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
const Resources = struct {
    delta_time: f32 = 0,
    allocator: std.mem.Allocator,
    player_entity: ecs.Entity(Player) = undefined,
    
    // Rendering
    renderer: renderer.Renderer = undefined,
    
    // World - now just holds brush data
    world: collision.World = undefined,
    
    // Static brush storage for complex demo geometry
    brush_planes: [80]collision.Plane = undefined, // Increased for complex shapes
    brushes: [8]collision.Brush = undefined,
    
    fn init(self: *@This(), allocator: std.mem.Allocator) void {
        self.allocator = allocator;
        self.renderer = renderer.Renderer.init(allocator);
        
        var plane_idx: usize = 0;
        var brush_idx: usize = 0;
        
        // 1. Massive ground platform
        brush_idx += self.addBoxBrush(&plane_idx, Vec3.new(0, config.World.Geometry.ground_center_y, 0), Vec3.new(config.World.Geometry.ground_size_x, config.World.Geometry.ground_size_y, config.World.Geometry.ground_size_z), brush_idx);
        
        // 2. Massive slope/mountain
        brush_idx += self.addSlopeBrush(&plane_idx, brush_idx);
        
        // 3. Scattered structures
        const structures = [_]struct { pos: Vec3, size: Vec3 }{
            .{ .pos = Vec3.new(25, 3, 0), .size = Vec3.new(4, 6, 4) },
            .{ .pos = Vec3.new(-25, 2, 15), .size = Vec3.new(6, 4, 6) },
            .{ .pos = Vec3.new(0, 1, -30), .size = Vec3.new(8, 2, 3) },
            .{ .pos = Vec3.new(-15, 4, -15), .size = Vec3.new(3, 8, 3) },
            .{ .pos = Vec3.new(30, 1.5, 25), .size = Vec3.new(5, 3, 5) },
        };
        
        for (structures) |structure| {
            if (brush_idx >= self.brushes.len) break;
            brush_idx += self.addBoxBrush(&plane_idx, structure.pos, structure.size, brush_idx);
        }
        
        self.initializeWorld(brush_idx);
    }
    
    fn addBoxBrush(self: *@This(), plane_idx: *usize, center: Vec3, size: Vec3, brush_idx: usize) u32 {
        if (brush_idx >= self.brushes.len or plane_idx.* + 6 > self.brush_planes.len) return 0;
        
        const half_size = Vec3.scale(size, 0.5);
        const planes = [6]collision.Plane{
            .{ .normal = Vec3.new( 1,  0,  0), .distance = -(center.data[0] + half_size.data[0]) },
            .{ .normal = Vec3.new(-1,  0,  0), .distance = center.data[0] - half_size.data[0] },
            .{ .normal = Vec3.new( 0,  1,  0), .distance = -(center.data[1] + half_size.data[1]) },
            .{ .normal = Vec3.new( 0, -1,  0), .distance = center.data[1] - half_size.data[1] },
            .{ .normal = Vec3.new( 0,  0,  1), .distance = -(center.data[2] + half_size.data[2]) },
            .{ .normal = Vec3.new( 0,  0, -1), .distance = center.data[2] - half_size.data[2] },
        };
        
        @memcpy(self.brush_planes[plane_idx.*..plane_idx.* + 6], &planes);
        self.brushes[brush_idx] = .{ 
            .planes = self.brush_planes[plane_idx.*..plane_idx.* + 6], 
            .bounds = AABB.new(Vec3.sub(center, half_size), Vec3.add(center, half_size))
        };
        plane_idx.* += 6;
        return 1;
    }
    
    fn addSlopeBrush(self: *@This(), plane_idx: *usize, brush_idx: usize) u32 {
        if (brush_idx >= self.brushes.len or plane_idx.* + 6 > self.brush_planes.len) return 0;
        
        const slope_width = config.World.Geometry.slope_width;
        const slope_height = config.World.Geometry.slope_height;
        const slope_center = Vec3.new(0, 0, config.World.Geometry.slope_center_z);
        const slope_angle = config.World.Geometry.slope_angle_degrees * config.Math.degrees_to_radians;
        
        const slope_normal = Vec3.normalize(Vec3.new(0, @cos(slope_angle), -@sin(slope_angle)));
        const slope_point = Vec3.new(0, slope_height, slope_center.data[2] + slope_width/2);
        
        const planes = [6]collision.Plane{
            .{ .normal = Vec3.new( 0, -1,  0), .distance = config.World.Geometry.slope_ground_level },
            .{ .normal = Vec3.new(-1,  0,  0), .distance = -slope_width/2 },
            .{ .normal = Vec3.new( 1,  0,  0), .distance = -slope_width/2 },
            .{ .normal = Vec3.new( 0,  0, -1), .distance = slope_center.data[2] - slope_width/2 },
            .{ .normal = Vec3.new( 0,  0,  1), .distance = -(slope_center.data[2] + slope_width/2) },
            .{ .normal = slope_normal, .distance = -Vec3.dot(slope_normal, slope_point) },
        };
        
        @memcpy(self.brush_planes[plane_idx.*..plane_idx.* + 6], &planes);
        self.brushes[brush_idx] = .{ 
            .planes = self.brush_planes[plane_idx.*..plane_idx.* + 6], 
            .bounds = AABB.new(
                Vec3.new(-slope_width/2, config.World.Geometry.slope_ground_level, slope_center.data[2] - slope_width/2), 
                Vec3.new(slope_width/2, slope_height, slope_center.data[2] + slope_width/2)
            )
        };
        plane_idx.* += 6;
        return 1;
    }
    
    fn initializeWorld(self: *@This(), brush_count: usize) void {
        self.world = collision.World.init(self.brushes[0..brush_count], self.allocator) catch |err| {
            std.log.err("Failed to initialize world: {}", .{err});
            return;
        };
        
        std.log.info("Created {} brushes", .{brush_count});
        for (self.brushes[0..brush_count], 0..) |brush, i| {
            std.log.info("  Brush {}: {} planes, bounds: ({d:.1},{d:.1},{d:.1}) to ({d:.1},{d:.1},{d:.1})", .{
                i, brush.planes.len,
                brush.bounds.min.data[0], brush.bounds.min.data[1], brush.bounds.min.data[2],
                brush.bounds.max.data[0], brush.bounds.max.data[1], brush.bounds.max.data[2]
            });
        }
    }
    
    fn buildWorldMesh(self: *@This()) !void {
        try self.renderer.buildWorldMesh(self.world.original_brushes);
    }
    
    fn render(self: *const @This(), view: Mat4) void {
        self.renderer.render(view);
    }
    
    fn deinit(self: *@This()) void {
        self.world.deinit();
        self.renderer.deinit();
    }
};

// Configuration constants
const MOUSE_SENSITIVITY: f32 = config.Input.mouse_sensitivity;
const PITCH_LIMIT: f32 = config.Input.pitch_limit;
const EYE_HEIGHT: f32 = config.Rendering.eye_height;

// Systems
fn sys_input(inputs: []Input, resources: *Resources) void {
    _ = resources;
    for (inputs) |*i| {
        i.yaw += i.mdx * MOUSE_SENSITIVITY;
        i.pitch = std.math.clamp(i.pitch + i.mdy * MOUSE_SENSITIVITY, -PITCH_LIMIT, PITCH_LIMIT);
        i.mdx = 0; i.mdy = 0;
    }
}

fn sys_physics(transforms: []Transform, physics_comps: []Physics, inputs: []Input, audios: []Audio, resources: *Resources) void {
    physics.update(transforms, physics_comps, inputs, audios, &resources.world, resources.delta_time);
}

fn sys_render(transforms: []Transform, inputs: []Input, resources: *Resources) void {
    for (transforms, inputs) |t, i| {
        const eye = Vec3.add(t.pos, Vec3.new(0, EYE_HEIGHT, 0));
        const cy, const sy = .{ @cos(i.yaw), @sin(i.yaw) };
        const cp, const sp = .{ @cos(i.pitch), @sin(i.pitch) };
        const view = Mat4{ .data = .{ 
            cy, sy*sp, -sy*cp, 0, 
            0, cp, sp, 0, 
            sy, -cy*sp, cy*cp, 0, 
            -eye.data[0]*cy - eye.data[2]*sy, 
            -eye.data[0]*sy*sp - eye.data[1]*cp + eye.data[2]*cy*sp, 
            eye.data[0]*sy*cp - eye.data[1]*sp - eye.data[2]*cy*cp, 
            1 
        }};
        resources.render(view);
        break;
    }
}

// Global state
var store: ecs.Store(Registry, Resources) = undefined;
var initialized: bool = false;

export fn init() void {
    const allocator = std.heap.c_allocator;
    sg.setup(.{ .environment = sokol.glue.environment() });
    saudio.setup(.{ .stream_cb = audio });
    simgui.setup(.{});
    
    store = ecs.Store(Registry, Resources){ .registry = .{ .players = .{} }, .resources = undefined };
    store.resources.init(allocator);
    
    // Build visual representation using original brushes for rendering
    store.resources.buildWorldMesh() catch |err| {
        std.log.err("Failed to build world mesh: {}", .{err});
    };
    
    // Create player - spawn at origin on the large ground plane
    store.resources.player_entity = store.create(Player, allocator, .{
        .transform = .{ .pos = Vec3.new(config.World.Player.spawn_x, config.World.Player.spawn_y, config.World.Player.spawn_z) },
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
    
    renderer.Renderer.renderCrosshair();
    
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
                const sound_duration = config.Audio.jump_sound_duration;
                const t = 1.0 - s.timer / sound_duration;
                s.timer -= 1.0 / config.Audio.sample_rate;
                sample += @sin((sound_duration - s.timer) * config.Audio.jump_frequency * config.Math.pi) * @exp(-t * config.Audio.jump_decay) * config.Audio.jump_volume;
                if (s.timer <= 0) s.active = false;
            }
        }
        for (0..c_usize) |ch| buf[f * c_usize + ch] = sample;
    }
}

pub fn main() void {
    sapp.run(.{ .init_cb = init, .frame_cb = frame, .cleanup_cb = cleanup, .event_cb = event, .width = config.Window.width, .height = config.Window.height, .window_title = config.Window.title });
}