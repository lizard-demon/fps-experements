const std = @import("std");

pub const EntityId = u32;

pub fn Entity(comptime T: type) type {
    _ = T;
    return struct { id: EntityId };
}

pub fn List(comptime T: type) type {
    return struct {
        pub const Archetype = T;
        list: std.MultiArrayList(T) = .{},
        entities: std.ArrayList(EntityId) = .{},

        pub fn create(self: *@This(), allocator: std.mem.Allocator, item: T, entity_id: EntityId) !Entity(T) {
            try self.list.append(allocator, item);
            try self.entities.append(allocator, entity_id);
            return Entity(T){ .id = entity_id };
        }
        
        pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
            self.list.deinit(allocator);
            self.entities.deinit(allocator);
        }

        pub fn items(self: @This(), comptime field: std.meta.FieldEnum(T)) []@FieldType(T, @tagName(field)) {
            return self.list.items(field);
        }
    };
}

pub fn Store(comptime Registry: type, comptime Resources: type) type {
    return struct {
        registry: Registry,
        resources: Resources,
        next_entity_id: EntityId = 0,

        pub fn create(self: *@This(), comptime ArchetypeType: type, allocator: std.mem.Allocator, item: ArchetypeType) !Entity(ArchetypeType) {
            const entity_id = self.next_entity_id;
            self.next_entity_id +%= 1;
            
            inline for (@typeInfo(Registry).@"struct".fields) |field| {
                if (field.type.Archetype == ArchetypeType) {
                    return @field(self.registry, field.name).create(allocator, item, entity_id);
                }
            }
            @compileError("Archetype not found in registry");
        }

        pub fn get(self: *@This(), entity_id: EntityId, comptime T: type) ?T {
            const ComponentType = @typeInfo(T).pointer.child;
            
            inline for (@typeInfo(Registry).@"struct".fields) |field| {
                if (!@hasDecl(field.type, "Archetype")) continue;
                
                const archetype_list = &@field(self.registry, field.name);
                const Archetype = field.type.Archetype;
                
                const has_component = comptime blk: {
                    for (std.meta.fields(Archetype)) |comp_field| {
                        if (comp_field.type == ComponentType) break :blk true;
                    }
                    break :blk false;
                };
                
                if (has_component) {
                    const index = blk: {
                        for (archetype_list.entities.items, 0..) |eid, i| {
                            if (eid == entity_id) break :blk i;
                        }
                        break :blk null;
                    };
                    if (index) |idx| {
                        inline for (std.meta.fields(Archetype)) |comp_field| {
                            if (comp_field.type == ComponentType) {
                                const field_enum = @field(@TypeOf(archetype_list.list).Field, comp_field.name);
                                return &archetype_list.list.items(field_enum)[idx];
                            }
                        }
                    }
                }
            }
            return null;
        }

        pub fn run(self: *@This(), system: anytype) void {
            const Args = std.meta.ArgsTuple(@TypeOf(system));

            inline for (@typeInfo(Registry).@"struct".fields) |field| {
                if (!@hasDecl(field.type, "Archetype")) continue;

                const Archetype = field.type.Archetype;
                const storage = @field(self.registry, field.name).list;
                const slices = storage.slice();

                var args: Args = undefined;
                var match = true;

                inline for (@typeInfo(Args).@"struct".fields, 0..) |arg_info, i| {
                    if (arg_info.type == *Resources) {
                        args[i] = &self.resources;
                        continue;
                    }
                    
                    const ComponentType = @typeInfo(arg_info.type).pointer.child;
                    var found = false;
                    
                    inline for (std.meta.fields(Archetype)) |comp_field| {
                        if (comp_field.type == ComponentType) {
                            const FieldEnum = @TypeOf(storage).Field;
                            args[i] = slices.items(@field(FieldEnum, comp_field.name));
                            found = true;
                            break;
                        }
                    }
                    if (!found) match = false;
                }

                if (match) @call(.auto, system, args);
            }
        }
    };
}