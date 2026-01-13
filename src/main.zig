const std = @import("std");
const sokol = @import("sokol");
const sapp = sokol.app;
const sg = sokol.gfx;
const saudio = sokol.audio;
const simgui = sokol.imgui;
const ig = @import("cimgui");

const math = @import("lib/math.zig");
const bvh = @import("lib/bvh.zig");

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = math.AABB;

// Module types
const Physics = @import("resources/physics.zig").Physics(.{});
const Renderer = @import("resources/render.zig").Renderer(.{});

// Game state
const Game = struct {
    allocator: std.mem.Allocator,
    delta_time: f32 = 0,
    
    // Systems
    physics: Physics = undefined,
    renderer: Renderer = undefined,
    
    // World data
    world: bvh.CollisionWorld = undefined,
    brush_planes: [80]bvh.Plane = undefined,
    brushes: [8]bvh.Brush = undefined,
    
    fn init(self: *@This(), allocator: std.mem.Allocator) void {
        self.allocator = allocator;
        self.physics = Physics.init(allocator);
        self.renderer = Renderer.init(allocator);
        
        var plane_idx: usize = 0;
        var brush_idx: usize = 0;
        
        // 1. Massive ground platform
        brush_idx += self.addBoxBrush(&plane_idx, Vec3.new(0, -2.25, 0), Vec3.new(100.0, 4.5, 100.0), brush_idx);
        
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
        const planes = [6]bvh.Plane{
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
        
        const slope_width = 30.0;
        const slope_height = 20.0;
        const slope_center = Vec3.new(0, 0, 20.0);
        const slope_angle = 46.0 * (3.14159265359 / 180.0);
        
        const slope_normal = Vec3.normalize(Vec3.new(0, @cos(slope_angle), -@sin(slope_angle)));
        const slope_point = Vec3.new(0, slope_height, slope_center.data[2] + slope_width/2);
        
        const planes = [6]bvh.Plane{
            .{ .normal = Vec3.new( 0, -1,  0), .distance = 0.0 },
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
                Vec3.new(-slope_width/2, 0.0, slope_center.data[2] - slope_width/2), 
                Vec3.new(slope_width/2, slope_height, slope_center.data[2] + slope_width/2)
            )
        };
        plane_idx.* += 6;
        return 1;
    }
    
    fn initializeWorld(self: *@This(), brush_count: usize) void {
        self.world = bvh.CollisionWorld.init(self.brushes[0..brush_count], self.allocator) catch |err| {
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
    
    fn deinit(self: *@This()) void {
        self.world.deinit();
        self.renderer.deinit();
        self.physics.deinit();
    }
};

// Configuration constants
const MOUSE_SENSITIVITY: f32 = 0.002;
const PITCH_LIMIT: f32 = 1.5;
const EYE_HEIGHT: f32 = 0.6;

// Global state
var game: Game = undefined;
var initialized: bool = false;

export fn init() void {
    const allocator = std.heap.c_allocator;
    sg.setup(.{ .environment = sokol.glue.environment() });
    saudio.setup(.{ .stream_cb = audio });
    simgui.setup(.{});
    
    game = .{ .allocator = allocator };
    game.init(allocator);
    
    game.buildWorldMesh() catch |err| {
        std.log.err("Failed to build world mesh: {}", .{err});
    };
    
    // Set initial player position
    game.physics.setPosition(Vec3.new(0.0, 3.0, -20.0));
    
    initialized = true;
}

export fn frame() void {
    game.delta_time = @as(f32, @floatCast(sapp.frameDuration()));
    
    // Update physics
    game.physics.handleInput(MOUSE_SENSITIVITY, PITCH_LIMIT);
    game.physics.update(&game.world, game.delta_time);
    
    // Render
    simgui.newFrame(.{ .width = sapp.width(), .height = sapp.height(), .delta_time = sapp.frameDuration() });
    
    const view = game.physics.getViewMatrix(EYE_HEIGHT);
    game.renderer.render(view);
    game.renderer.renderCrosshair();
    
    simgui.render(); 
    sg.endPass(); 
    sg.commit();
}

export fn cleanup() void {
    initialized = false;
    saudio.shutdown();
    game.deinit();
    simgui.shutdown(); 
    sg.shutdown();
}

export fn event(e: [*c]const sapp.Event) void {
    if (e == null) return;
    _ = simgui.handleEvent(e.*);
    
    if (!initialized) return;
    
    const d = e.*.type == .KEY_DOWN;
    
    switch (e.*.type) {
        .KEY_DOWN, .KEY_UP => switch (e.*.key_code) {
            .W => game.physics.onKeyEvent(.W, d),
            .A => game.physics.onKeyEvent(.A, d),
            .S => game.physics.onKeyEvent(.S, d),
            .D => game.physics.onKeyEvent(.D, d),
            .SPACE => game.physics.onKeyEvent(.SPACE, d),
            .ESCAPE => {
                game.physics.onKeyEvent(.ESCAPE, d);
                if (d and !game.physics.isMouseLocked()) {
                    sapp.showMouse(true);
                    sapp.lockMouse(false);
                }
            },
            else => {},
        },
        .MOUSE_DOWN => if (e.*.mouse_button == .LEFT) {
            game.physics.onMouseEvent(.LEFT, true);
            if (game.physics.isMouseLocked()) {
                sapp.showMouse(false);
                sapp.lockMouse(true);
            }
        },
        .MOUSE_MOVE => {
            game.physics.onMouseMove(e.*.mouse_dx, e.*.mouse_dy);
        },
        else => {},
    }
}

fn audio(buf: [*c]f32, n: i32, c: i32) callconv(.c) void {
    if (n <= 0 or c <= 0) return;
    
    const n_usize: usize = @intCast(n);
    const c_usize: usize = @intCast(c);
    const total_samples = std.math.mul(usize, n_usize, c_usize) catch return;
    
    if (!initialized) { 
        for (0..total_samples) |i| buf[i] = 0; 
        return; 
    }
    
    for (0..n_usize) |f| {
        const sample = game.physics.getAudioSample(44100.0);
        for (0..c_usize) |ch| buf[f * c_usize + ch] = sample;
    }
}

pub fn main() void {
    sapp.run(.{ 
        .init_cb = init, 
        .frame_cb = frame, 
        .cleanup_cb = cleanup, 
        .event_cb = event, 
        .width = 1024, 
        .height = 768, 
        .window_title = "FPS" 
    });
}