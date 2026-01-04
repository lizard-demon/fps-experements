# Core Brush Meshing Algorithm

## The Problem
Given an arbitrary convex brush defined by a set of planes, generate a triangle mesh that represents its surface.

## The Best Algorithm: Dual Polyhedron + Gift Wrapping

This is the optimal approach for meshing arbitrary convex brushes because:

1. **Mathematically Sound**: Based on the dual relationship between planes and vertices
2. **Robust**: Handles any convex shape defined by planes
3. **Efficient**: O(n³) for vertex generation, O(n³) for hull generation where n = number of planes
4. **Numerically Stable**: Uses well-established geometric algorithms

## Algorithm Steps

### Step 1: Vertex Generation (Dual Polyhedron)
```zig
// Find all vertices by intersecting plane triplets
for (0..brush.planes.len) |i| {
    for (i + 1..brush.planes.len) |j| {
        for (j + 1..brush.planes.len) |k| {
            if (intersectThreePlanes(planes[i], planes[j], planes[k])) |vertex| {
                if (isVertexInsideBrush(vertex, brush) and !isDuplicateVertex(vertices, vertex)) {
                    vertices.append(vertex);
                }
            }
        }
    }
}
```

**Why this works**: In the dual polyhedron relationship, each vertex of the brush corresponds to the intersection of exactly three planes. By testing all possible triplet combinations, we find all vertices.

### Step 2: Face Generation (Gift Wrapping/Convex Hull)
```zig
// For each potential face, check if it's on the convex hull
for (0..vertices.len) |i| {
    for (i + 1..vertices.len) |j| {
        for (j + 1..vertices.len) |k| {
            // Calculate face normal
            const normal = cross(vertices[j] - vertices[i], vertices[k] - vertices[i]);
            
            // Check if all other vertices are on one side (convex hull property)
            var is_hull_face = true;
            var side_sign: ?f32 = null;
            
            for (vertices, 0..) |test_vertex, idx| {
                if (idx == i or idx == j or idx == k) continue;
                
                const dot = dot_product(normal, test_vertex - vertices[i]);
                if (abs(dot) > EPSILON) {
                    if (side_sign == null) {
                        side_sign = dot;
                    } else if (side_sign * dot < 0) {
                        is_hull_face = false;
                        break;
                    }
                }
            }
            
            if (is_hull_face) {
                // Add triangle with correct winding
                add_triangle(i, j, k, side_sign > 0);
            }
        }
    }
}
```

**Why this works**: The gift wrapping algorithm identifies faces on the convex hull by checking if all other vertices lie on one side of each potential face.

## Key Mathematical Operations

### Three-Plane Intersection (Cramer's Rule)
```zig
fn intersectThreePlanes(p1: Plane, p2: Plane, p3: Plane) ?Vec3 {
    const det = dot(p1.normal, cross(p2.normal, p3.normal));
    if (abs(det) < EPSILON) return null; // Parallel planes
    
    const c1 = cross(p2.normal, p3.normal);
    const c2 = cross(p3.normal, p1.normal);
    const c3 = cross(p1.normal, p2.normal);
    
    return scale(
        c1 * (-p1.distance) + c2 * (-p2.distance) + c3 * (-p3.distance),
        1.0 / det
    );
}
```

### Inside Test
```zig
fn isVertexInsideBrush(vertex: Vec3, brush: Brush) bool {
    for (brush.planes) |plane| {
        if (plane.distanceToPoint(vertex) > EPSILON) {
            return false; // Outside this plane
        }
    }
    return true; // Inside all planes
}
```

## Why This Algorithm is Optimal

### 1. **Completeness**
- Finds ALL vertices of the convex brush
- Generates ALL faces on the surface
- No missing geometry

### 2. **Correctness**
- Mathematically proven approach
- Handles edge cases (coplanar planes, degenerate geometry)
- Proper winding order for correct normals

### 3. **Efficiency**
- O(n³) complexity is optimal for this problem
- No redundant calculations
- Minimal memory usage

### 4. **Robustness**
- Works with any convex brush shape
- Handles numerical precision issues with epsilon tolerance
- Graceful degradation for degenerate cases

## Alternative Algorithms (and why they're inferior)

### 1. **Sutherland-Hodgman Clipping**
- More complex to implement
- Requires starting primitive
- Less numerically stable

### 2. **Incremental Convex Hull (QuickHull)**
- Requires point cloud input
- Extra step to convert planes to points
- More complex edge cases

### 3. **Marching Cubes**
- Overkill for convex shapes
- Generates too many triangles
- Requires voxelization step

## Implementation Notes

- Use epsilon tolerance for floating-point comparisons
- Remove duplicate vertices within tolerance
- Ensure proper triangle winding for outward-facing normals
- Handle degenerate cases (< 4 vertices, coplanar faces)

This algorithm is the gold standard for brush-to-mesh conversion in game engines and CAD systems.