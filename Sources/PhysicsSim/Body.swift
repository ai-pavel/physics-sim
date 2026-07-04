import Foundation

/// A 2D rigid body with physical properties.
public class Body {
    /// Unique identifier.
    public let id: Int

    /// Shape of the body.
    public let shape: Shape

    // MARK: - Physical properties

    /// Mass (0 or .infinity means static/immovable).
    public var mass: Double
    /// Inverse mass (precomputed).
    public var inverseMass: Double
    /// Moment of inertia.
    public var inertia: Double
    /// Inverse moment of inertia.
    public var inverseInertia: Double
    /// Coefficient of restitution (bounciness, 0..1).
    public var restitution: Double
    /// Friction coefficient.
    public var friction: Double

    // MARK: - State

    /// Position of the center of mass in world space.
    public var position: Vec2
    /// Previous position (used by Verlet integrator).
    public var previousPosition: Vec2
    /// Linear velocity.
    public var velocity: Vec2
    /// Rotation angle in radians.
    public var angle: Double
    /// Previous angle (used by Verlet integrator).
    public var previousAngle: Double
    /// Angular velocity in radians per second.
    public var angularVelocity: Double
    /// Accumulated force for the current step.
    public var force: Vec2
    /// Accumulated torque for the current step.
    public var torque: Double
    /// Whether this body is static (immovable).
    public var isStatic: Bool

    private static var nextId = 0

    public init(
        shape: Shape,
        position: Vec2,
        mass: Double = 1.0,
        restitution: Double = 0.5,
        friction: Double = 0.3,
        isStatic: Bool = false
    ) {
        self.id = Body.nextId
        Body.nextId += 1

        self.shape = shape
        self.position = position
        self.previousPosition = position
        self.velocity = .zero
        self.angle = 0
        self.previousAngle = 0
        self.angularVelocity = 0
        self.force = .zero
        self.torque = 0
        self.restitution = restitution
        self.friction = friction
        self.isStatic = isStatic

        if isStatic {
            self.mass = 0
            self.inverseMass = 0
            self.inertia = 0
            self.inverseInertia = 0
        } else {
            self.mass = mass
            self.inverseMass = mass > 0 ? 1.0 / mass : 0
            self.inertia = shape.momentOfInertia(mass: mass)
            self.inverseInertia = self.inertia > 0 ? 1.0 / self.inertia : 0
        }
    }

    /// Apply a force at the center of mass.
    public func applyForce(_ f: Vec2) {
        force += f
    }

    /// Apply a force at a world-space point.
    public func applyForce(_ f: Vec2, at point: Vec2) {
        force += f
        let r = point - position
        torque += r.cross(f)
    }

    /// Apply an impulse at the center of mass.
    public func applyImpulse(_ impulse: Vec2) {
        velocity += impulse * inverseMass
    }

    /// Apply an impulse at a world-space contact point.
    public func applyImpulse(_ impulse: Vec2, at point: Vec2) {
        velocity += impulse * inverseMass
        let r = point - position
        angularVelocity += inverseInertia * r.cross(impulse)
    }

    /// Clear accumulated forces and torques.
    public func clearForces() {
        force = .zero
        torque = 0
    }
}

extension Body: Equatable {
    public static func == (lhs: Body, rhs: Body) -> Bool {
        lhs.id == rhs.id
    }
}

extension Body: Hashable {
    public func hash(into hasher: inout Hasher) {
        hasher.combine(id)
    }
}
