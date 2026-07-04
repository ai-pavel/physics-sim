import Foundation

/// Represents the geometric shape of a rigid body.
public enum Shape {
    case circle(radius: Double)
    case polygon(vertices: [Vec2])  // vertices in local space, CCW winding

    /// Creates a rectangle polygon centered at origin.
    public static func rectangle(width: Double, height: Double) -> Shape {
        let hw = width / 2
        let hh = height / 2
        return .polygon(vertices: [
            Vec2(-hw, -hh),
            Vec2( hw, -hh),
            Vec2( hw,  hh),
            Vec2(-hw,  hh),
        ])
    }

    /// Computes the moment of inertia for a given mass.
    public func momentOfInertia(mass: Double) -> Double {
        switch self {
        case .circle(let radius):
            // I = 0.5 * m * r^2
            return 0.5 * mass * radius * radius
        case .polygon(let vertices):
            // Use the polygon moment of inertia formula
            let n = vertices.count
            guard n >= 3 else { return 1.0 }
            var numerator = 0.0
            var denominator = 0.0
            for i in 0..<n {
                let a = vertices[i]
                let b = vertices[(i + 1) % n]
                let crossVal = abs(a.cross(b))
                numerator += crossVal * (a.dot(a) + a.dot(b) + b.dot(b))
                denominator += crossVal
            }
            guard denominator > 1e-12 else { return 1.0 }
            return (mass / 6.0) * (numerator / denominator)
        }
    }

    /// Returns the vertices transformed to world space.
    public func worldVertices(position: Vec2, angle: Double) -> [Vec2] {
        switch self {
        case .circle:
            return []
        case .polygon(let vertices):
            let cosA = cos(angle)
            let sinA = sin(angle)
            return vertices.map { v in
                Vec2(
                    cosA * v.x - sinA * v.y + position.x,
                    sinA * v.x + cosA * v.y + position.y
                )
            }
        }
    }

    /// Returns the edge normals for SAT, in world orientation.
    public func worldNormals(angle: Double) -> [Vec2] {
        switch self {
        case .circle:
            return []
        case .polygon(let vertices):
            let n = vertices.count
            let cosA = cos(angle)
            let sinA = sin(angle)
            var normals: [Vec2] = []
            for i in 0..<n {
                let a = vertices[i]
                let b = vertices[(i + 1) % n]
                let edge = b - a
                // Outward normal (right-hand perpendicular for CCW winding)
                let normal = Vec2(edge.y, -edge.x).normalized
                // Rotate by body angle
                let rotated = Vec2(
                    cosA * normal.x - sinA * normal.y,
                    sinA * normal.x + cosA * normal.y
                )
                normals.append(rotated)
            }
            return normals
        }
    }
}
