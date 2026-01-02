const std = @import("std");

pub fn List(comptime T: type) type {
    return struct {
        pub const Archetype = T;
        list: std.MultiArrayList(T) = .{},

        pub fn append(self: *@This(), allocator: std.mem.Allocator, item: T) !void {
            try self.list.append(allocator, item);
        }
        
        pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
            self.list.deinit(allocator);
        }

        pub fn items(self: @This(), comptime field: std.meta.FieldEnum(T)) []@FieldType(T, @tagName(field)) {
            return self.list.items(field);
        }
    };
}

pub fn Store(comptime Registry: type) type {
    return struct {
        registry: Registry,

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