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

const Vec3 = math.Vec3;
const Mat4 = math.Mat4;
const AABB = collision.AABB;
const BrushWorld = collision.BrushWorld;
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
            .cull_mode = .NONE 
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
        try self.add_brush_visual_generic(brush, color);
    }
    
    fn add_slope(self: *@This(), center: Vec3, size: Vec3, angle_degrees: f32, color: [4]f32) !void {
        const angle_rad = angle_degrees * std.math.pi / 180.0;
        const half_size = Vec3.scale(size, 0.5);
        
        // Create planes for a sloped brush (wedge shape)
        var planes = try self.allocator.alloc(collision.Plane, 5);
        defer self.allocator.free(planes);
        
        // Bottom plane (y = min.y): normal (0,-1,0) pointing outward (downward)
        planes[0] = collision.Plane.new(Vec3.new(0, -1, 0), center.data[1] - half_size.data[1]);
        
        // Sloped surface - normal points outward from the slope
        const slope_normal = Vec3.normalize(Vec3.new(@sin(angle_rad), @cos(angle_rad), 0));
        const slope_point = Vec3.new(center.data[0], center.data[1] + half_size.data[1], center.data[2]);
        planes[1] = collision.Plane.new(slope_normal, -Vec3.dot(slope_normal, slope_point));
        
        // Side walls (normals point outward)
        planes[2] = collision.Plane.new(Vec3.new(-1, 0, 0), center.data[0] - half_size.data[0]);
        planes[3] = collision.Plane.new(Vec3.new(1, 0, 0), -(center.data[0] + half_size.data[0]));
        
        // Back wall (normal points outward)
        planes[4] = collision.Plane.new(Vec3.new(0, 0, -1), center.data[2] - half_size.data[2]);
        
        const brush = try collision.Brush.init(self.allocator, planes);
        try self.brush_world.addBrush(brush);
        
        // Add visual geometry using the generic convex hull generator
        try self.add_brush_visual_generic(brush, color);
    }
    
    // Generic convex hull mesh generator for any brush
    fn add_brush_visual_generic(self: *@This(), brush: collision.Brush, color: [4]f32) !void {
        // For now, fall back to a simpler approach that works reliably
        // Generate faces directly from brush planes using bounds intersection
        
        for (brush.planes) |plane| {
            try self.addBrushPlaneFace(brush, plane, color);
        }
    }
    
    // Generate a face for a specific plane of the brush
    fn addBrushPlaneFace(self: *@This(), brush: collision.Brush, plane: collision.Plane, color: [4]f32) !void {
        // Find the intersection of this plane with the brush bounds to create a face
        var face_vertices = std.ArrayListUnmanaged(Vec3){};
        defer face_vertices.deinit(self.allocator);
        
        // For all planes, use edge intersection method to ensure proper clipping
        try self.generateClippedFace(brush, plane, &face_vertices);
        
        if (face_vertices.items.len < 3) return;
        
        // Ensure correct winding order based on plane normal
        self.ensureCorrectWinding(face_vertices.items, plane.normal);
        
        // Add vertices to mesh
        const base = @as(u16, @intCast(self.vertices.items.len));
        const center = self.calculateCenter(face_vertices.items);
        
        for (face_vertices.items) |vertex| {
            var c = color;
            // Simple lighting based on Y coordinate
            if (vertex.data[1] <= center.data[1]) { 
                c[0] *= 0.7; c[1] *= 0.7; c[2] *= 0.7; 
            } else { 
                c[0] *= 1.1; c[1] *= 1.1; c[2] *= 1.1; 
            }
            try self.vertices.append(self.allocator, .{ 
                .pos = .{ vertex.data[0], vertex.data[1], vertex.data[2] }, 
                .col = c 
            });
        }
        
        // Triangulate the face (fan triangulation from first vertex)
        for (1..face_vertices.items.len - 1) |i| {
            try self.indices.append(self.allocator, base);
            try self.indices.append(self.allocator, base + @as(u16, @intCast(i)));
            try self.indices.append(self.allocator, base + @as(u16, @intCast(i + 1)));
        }
    }
    
    // Generate a properly clipped face for any plane
    fn generateClippedFace(self: *@This(), brush: collision.Brush, target_plane: collision.Plane, face_vertices: *std.ArrayListUnmanaged(Vec3)) !void {
        // Start with a large polygon on the target plane, then clip it against all other planes
        
        // Create initial large quad on the target plane
        var polygon = std.ArrayListUnmanaged(Vec3){};
        defer polygon.deinit(self.allocator);
        
        // Find a reasonable size for the initial polygon based on brush bounds
        const bounds_size = Vec3.sub(brush.bounds.max, brush.bounds.min);
        const max_size = @max(@max(bounds_size.data[0], bounds_size.data[1]), bounds_size.data[2]) * 2;
        
        // Create a local coordinate system for the plane
        var tangent = Vec3.new(1, 0, 0);
        if (@abs(Vec3.dot(target_plane.normal, tangent)) > 0.9) {
            tangent = Vec3.new(0, 1, 0);
        }
        tangent = Vec3.normalize(Vec3.sub(tangent, Vec3.scale(target_plane.normal, Vec3.dot(tangent, target_plane.normal))));
        const bitangent = Vec3.cross(target_plane.normal, tangent);
        
        // Find a point on the plane (use brush center projected onto plane)
        const brush_center = Vec3.scale(Vec3.add(brush.bounds.min, brush.bounds.max), 0.5);
        const plane_point = Vec3.sub(brush_center, Vec3.scale(target_plane.normal, target_plane.distanceToPoint(brush_center)));
        
        // Create initial large quad
        const half_size = max_size * 0.5;
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, -half_size)), Vec3.scale(bitangent, -half_size)));
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, half_size)), Vec3.scale(bitangent, -half_size)));
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, half_size)), Vec3.scale(bitangent, half_size)));
        try polygon.append(self.allocator, Vec3.add(Vec3.add(plane_point, Vec3.scale(tangent, -half_size)), Vec3.scale(bitangent, half_size)));
        
        // Clip against all other planes
        for (brush.planes) |clip_plane| {
            // Skip the target plane itself
            if (Vec3.dot(clip_plane.normal, target_plane.normal) > 0.999 and 
                @abs(clip_plane.distance - target_plane.distance) < 0.001) continue;
            
            try self.clipPolygonByPlane(&polygon, clip_plane);
            if (polygon.items.len == 0) break; // Polygon completely clipped away
        }
        
        // Copy result to face_vertices
        for (polygon.items) |vertex| {
            try face_vertices.append(self.allocator, vertex);
        }
    }
    
    // Clip a polygon by a plane using Sutherland-Hodgman algorithm
    fn clipPolygonByPlane(self: *@This(), polygon: *std.ArrayListUnmanaged(Vec3), plane: collision.Plane) !void {
        if (polygon.items.len == 0) return;
        
        var output = std.ArrayListUnmanaged(Vec3){};
        defer {
            // Replace polygon contents with output
            polygon.deinit(self.allocator);
            polygon.* = output;
        }
        
        if (polygon.items.len == 0) return;
        
        var prev_vertex = polygon.items[polygon.items.len - 1];
        var prev_inside = plane.distanceToPoint(prev_vertex) <= collision.COLLISION_EPSILON;
        
        for (polygon.items) |curr_vertex| {
            const curr_inside = plane.distanceToPoint(curr_vertex) <= collision.COLLISION_EPSILON;
            
            if (curr_inside) {
                if (!prev_inside) {
                    // Entering: add intersection point
                    if (self.intersectLineWithPlaneUnbounded(prev_vertex, curr_vertex, plane)) |intersection| {
                        try output.append(self.allocator, intersection);
                    }
                }
                // Add current vertex
                try output.append(self.allocator, curr_vertex);
            } else if (prev_inside) {
                // Exiting: add intersection point
                if (self.intersectLineWithPlaneUnbounded(prev_vertex, curr_vertex, plane)) |intersection| {
                    try output.append(self.allocator, intersection);
                }
            }
            
            prev_vertex = curr_vertex;
            prev_inside = curr_inside;
        }
    }
    
    // Intersect an unbounded line with a plane
    fn intersectLineWithPlaneUnbounded(self: *@This(), start: Vec3, end: Vec3, plane: collision.Plane) ?Vec3 {
        _ = self;
        const dir = Vec3.sub(end, start);
        const denom = Vec3.dot(plane.normal, dir);
        
        if (@abs(denom) < collision.COLLISION_EPSILON) return null; // Line is parallel to plane
        
        const t = -(plane.distanceToPoint(start)) / denom;
        return Vec3.add(start, Vec3.scale(dir, t));
    }
    
    // Ensure vertices are wound correctly for the given normal
    fn ensureCorrectWinding(self: *@This(), vertices: []Vec3, normal: Vec3) void {
        if (vertices.len < 3) return;
        
        // Calculate the center
        const center = self.calculateCenter(vertices);
        
        // Sort vertices counter-clockwise around the normal
        self.sortVerticesCounterClockwise(vertices, normal, center);
    }
    
    // Calculate center of vertices
    fn calculateCenter(self: *@This(), vertices: []Vec3) Vec3 {
        _ = self;
        var center = Vec3.zero();
        for (vertices) |vertex| {
            center = Vec3.add(center, vertex);
        }
        return Vec3.scale(center, 1.0 / @as(f32, @floatFromInt(vertices.len)));
    }
    
    // Sort vertices counter-clockwise around a plane normal
    fn sortVerticesCounterClockwise(self: *@This(), vertices: []Vec3, normal: Vec3, center: Vec3) void {
        _ = self;
        // Create a local coordinate system for the plane
        var tangent = Vec3.new(1, 0, 0);
        if (@abs(Vec3.dot(normal, tangent)) > 0.9) {
            tangent = Vec3.new(0, 1, 0);
        }
        tangent = Vec3.normalize(Vec3.sub(tangent, Vec3.scale(normal, Vec3.dot(tangent, normal))));
        const bitangent = Vec3.cross(normal, tangent);
        
        // Sort by angle around the center
        const Context = struct {
            center: Vec3,
            tangent: Vec3,
            bitangent: Vec3,
            
            fn lessThan(ctx: @This(), a: Vec3, b: Vec3) bool {
                const va = Vec3.sub(a, ctx.center);
                const vb = Vec3.sub(b, ctx.center);
                
                const angle_a = std.math.atan2(Vec3.dot(va, ctx.bitangent), Vec3.dot(va, ctx.tangent));
                const angle_b = std.math.atan2(Vec3.dot(vb, ctx.bitangent), Vec3.dot(vb, ctx.tangent));
                
                return angle_a < angle_b;
            }
        };
        
        const context = Context{ .center = center, .tangent = tangent, .bitangent = bitangent };
        std.mem.sort(Vec3, vertices, context, Context.lessThan);
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
                const friction_factor: f32 = 5.0;
                const factor = @max(0, speed - @max(speed, 0.1) * friction_factor * dt) / speed;
                p.vel.data[0] *= factor; 
                p.vel.data[2] *= factor;
            } else { 
                p.vel.data[0] = 0; 
                p.vel.data[2] = 0; 
            }
        }
        
        // Movement with improved collision response
        const delta = Vec3.scale(p.vel, dt);
        const move_result = resources.brush_world.movePlayer(t.pos, delta, player_aabb);
        
        // Apply velocity adjustments from collision system
        p.vel = Vec3.add(p.vel, Vec3.scale(move_result.velocity_adjustment, 1.0 / dt));
        
        // Update ground state and handle ceiling hits
        p.on_ground = move_result.on_ground;
        if (move_result.hit_ceiling) {
            p.vel.data[1] = @min(p.vel.data[1], 0); // Stop upward movement when hitting ceiling
        }
        
        // Apply some sliding friction when hitting walls
        if (move_result.hit_wall and p.on_ground) {
            const horizontal_speed = @sqrt(p.vel.data[0] * p.vel.data[0] + p.vel.data[2] * p.vel.data[2]);
            if (horizontal_speed > 0.1) {
                const wall_friction: f32 = 2.0;
                const factor = @max(0, horizontal_speed - horizontal_speed * wall_friction * dt) / horizontal_speed;
                p.vel.data[0] *= factor;
                p.vel.data[2] *= factor;
            }
        }
        
        t.pos = move_result.position;
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