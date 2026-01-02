const std = @import("std");

pub const EntityId = u32;

pub fn Entity(comptime T: type) type {
    _ = T; // We don't actually need to store the type at runtime
    return struct {
        id: EntityId,
    };
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
            const entity_id = blk: {
                const id = self.next_entity_id;
                self.next_entity_id +%= 1;
                break :blk id;
            };
            
            inline for (@typeInfo(Registry).@"struct".fields) |field| {
                if (field.type.Archetype == ArchetypeType) {
                    return @field(self.registry, field.name).create(allocator, item, entity_id);
                }
            }
            @compileError("Archetype not found in registry");
        }

        // Unified get() method - automatically detects what you want based on type
        // Like Rob Pike's regex: one simple function that handles all cases internally
        pub fn get(self: *@This(), entity_id: EntityId, comptime T: type) ?T {
            // Getting component - T should be *ComponentType
            const ComponentType = @typeInfo(T).pointer.child;
            
            inline for (@typeInfo(Registry).@"struct".fields) |field| {
                if (!@hasDecl(field.type, "Archetype")) continue;
                
                const archetype_list = &@field(self.registry, field.name);
                const Archetype = field.type.Archetype;
                
                // Check if this archetype has the component
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
                        // Find the field name for this component
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

        // Single, unified system runner - automatically detects what the system needs
        // Like Rob Pike's regex: one simple function that handles all cases internally
        // Includes automatic relationship detection via special system signatures
        pub fn run(self: *@This(), system: anytype) void {
            const Args = std.meta.ArgsTuple(@TypeOf(system));

            inline for (@typeInfo(Registry).@"struct".fields) |field| {
                if (!@hasDecl(field.type, "Archetype")) continue;

                const Archetype = field.type.Archetype;
                const storage = @field(self.registry, field.name).list;
                const slices = storage.slice();
                const archetype_list = &@field(self.registry, field.name);

                var args: Args = undefined;
                var match = true;

                inline for (@typeInfo(Args).@"struct".fields, 0..) |arg_info, i| {
                    // Check if this argument is the Resources type
                    if (arg_info.type == *Resources) {
                        args[i] = &self.resources;
                        continue;
                    }
                    
                    // Check if this argument is EntityId slice
                    if (arg_info.type == []EntityId) {
                        args[i] = archetype_list.entities.items;
                        continue;
                    }
                    
                    // Check if this argument is the Store itself (for complex systems)
                    if (arg_info.type == *@This()) {
                        args[i] = self;
                        continue;
                    }
                    
                    // Regular component matching
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

        // Unified relationship system - replaces both followRelation and forEachRelation
        // Systems that need relationships just take the Store as a parameter and use this method
        pub fn relation(self: *@This(), entity_id: EntityId, comptime RelationComponent: type, comptime TargetComponent: type) ?*TargetComponent {
            // First, find the relation component on the entity
            if (self.get(entity_id, *RelationComponent)) |rel_comp| {
                // Extract the target entity ID from the relation
                const target_id = switch (@typeInfo(RelationComponent)) {
                    .@"struct" => |struct_info| blk: {
                        // Look for a field that contains an EntityId
                        inline for (struct_info.fields) |field| {
                            if (field.type == EntityId) {
                                break :blk @field(rel_comp.*, field.name);
                            }
                        }
                        @compileError("Relation component must have an EntityId field");
                    },
                    else => @compileError("Relation component must be a struct"),
                };
                
                // Now get the target component from the related entity
                return self.get(target_id, *TargetComponent);
            }
            return null;
        }

        // Query across multiple archetypes - also auto-detects system needs
        pub fn query(self: *@This(), comptime archetypes: []const type, system: anytype) void {
            inline for (archetypes) |ArchetypeType| {
                inline for (@typeInfo(Registry).@"struct".fields) |field| {
                    if (field.type.Archetype == ArchetypeType) {
                        const storage = @field(self.registry, field.name).list;
                        const slices = storage.slice();
                        const archetype_list = &@field(self.registry, field.name);
                        
                        // Check if this archetype matches the system signature
                        const Args = std.meta.ArgsTuple(@TypeOf(system));
                        var args: Args = undefined;
                        var match = true;

                        inline for (@typeInfo(Args).@"struct".fields, 0..) |arg_info, i| {
                            // Check if this argument is the Resources type
                            if (arg_info.type == *Resources) {
                                args[i] = &self.resources;
                                continue;
                            }
                            
                            // Check if this argument is EntityId slice
                            if (arg_info.type == []EntityId) {
                                args[i] = archetype_list.entities.items;
                                continue;
                            }
                            
                            // Check if this argument is the Store itself
                            if (arg_info.type == *@This()) {
                                args[i] = self;
                                continue;
                            }
                            
                            const ComponentType = @typeInfo(arg_info.type).pointer.child;
                            var found = false;
                            
                            inline for (std.meta.fields(ArchetypeType)) |comp_field| {
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
                        break;
                    }
                }
            }
        }
    };
}