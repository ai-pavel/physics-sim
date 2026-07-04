import Foundation

/// A 2D vector with standard math operations.
public struct Vec2: Equatable, CustomStringConvertible {
    public var x: Double
    public var y: Double

    public init(_ x: Double, _ y: Double) {
        self.x = x
        self.y = y
    }

    public static let zero = Vec2(0, 0)

    // MARK: - Arithmetic

    public static func + (lhs: Vec2, rhs: Vec2) -> Vec2 {
        Vec2(lhs.x + rhs.x, lhs.y + rhs.y)
    }

    public static func - (lhs: Vec2, rhs: Vec2) -> Vec2 {
        Vec2(lhs.x - rhs.x, lhs.y - rhs.y)
    }

    public static func * (lhs: Vec2, rhs: Double) -> Vec2 {
        Vec2(lhs.x * rhs, lhs.y * rhs)
    }

    public static func * (lhs: Double, rhs: Vec2) -> Vec2 {
        Vec2(lhs * rhs.x, lhs * rhs.y)
    }

    public static func / (lhs: Vec2, rhs: Double) -> Vec2 {
        Vec2(lhs.x / rhs, lhs.y / rhs)
    }

    public static prefix func - (v: Vec2) -> Vec2 {
        Vec2(-v.x, -v.y)
    }

    public static func += (lhs: inout Vec2, rhs: Vec2) {
        lhs.x += rhs.x
        lhs.y += rhs.y
    }

    public static func -= (lhs: inout Vec2, rhs: Vec2) {
        lhs.x -= rhs.x
        lhs.y -= rhs.y
    }

    public static func *= (lhs: inout Vec2, rhs: Double) {
        lhs.x *= rhs
        lhs.y *= rhs
    }

    // MARK: - Vector operations

    /// Dot product.
    public func dot(_ other: Vec2) -> Double {
        x * other.x + y * other.y
    }

    /// 2D cross product (scalar result).
    public func cross(_ other: Vec2) -> Double {
        x * other.y - y * other.x
    }

    /// Cross product with a scalar (returns perpendicular vector, ω × r convention).
    public func cross(_ s: Double) -> Vec2 {
        Vec2(-s * y, s * x)
    }

    /// Squared magnitude.
    public var lengthSquared: Double {
        x * x + y * y
    }

    /// Magnitude.
    public var length: Double {
        sqrt(lengthSquared)
    }

    /// Returns a unit vector, or zero if length is near zero.
    public var normalized: Vec2 {
        let len = length
        guard len > 1e-12 else { return .zero }
        return self / len
    }

    /// Perpendicular vector (rotated 90 degrees counter-clockwise).
    public var perpendicular: Vec2 {
        Vec2(-y, x)
    }

    /// Distance to another point.
    public func distance(to other: Vec2) -> Double {
        (self - other).length
    }

    public var description: String {
        String(format: "(%.3f, %.3f)", x, y)
    }
}
