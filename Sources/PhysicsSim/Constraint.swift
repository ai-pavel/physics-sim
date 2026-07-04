import Foundation

/// A constraint connecting two bodies.
public protocol Constraint {
    /// Apply the constraint forces/impulses for the given time step.
    func solve(dt: Double)
}

/// Maintains a fixed distance between two anchor points on two bodies.
public class DistanceJoint: Constraint {
    public let bodyA: Body
    public let bodyB: Body
    /// Local-space anchor on body A.
    public let anchorA: Vec2
    /// Local-space anchor on body B.
    public let anchorB: Vec2
    /// Target distance.
    public let distance: Double
    /// Stiffness (0..1), 1 = rigid.
    public let stiffness: Double

    public init(bodyA: Body, bodyB: Body, anchorA: Vec2 = .zero, anchorB: Vec2 = .zero, distance: Double? = nil, stiffness: Double = 1.0) {
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.anchorA = anchorA
        self.anchorB = anchorB
        self.stiffness = stiffness

        if let d = distance {
            self.distance = d
        } else {
            let wA = bodyA.position + anchorA
            let wB = bodyB.position + anchorB
            self.distance = wA.distance(to: wB)
        }
    }

    public func solve(dt: Double) {
        let worldA = worldAnchor(body: bodyA, local: anchorA)
        let worldB = worldAnchor(body: bodyB, local: anchorB)

        let delta = worldB - worldA
        let currentDist = delta.length
        guard currentDist > 1e-12 else { return }

        let normal = delta / currentDist
        let error = currentDist - distance

        let totalInvMass = bodyA.inverseMass + bodyB.inverseMass
        guard totalInvMass > 0 else { return }

        let correction = normal * (error / totalInvMass) * stiffness

        bodyA.position += correction * bodyA.inverseMass
        bodyB.position -= correction * bodyB.inverseMass

        // Velocity correction
        let relVel = bodyB.velocity - bodyA.velocity
        let normalVel = relVel.dot(normal)
        let impulse = normal * (normalVel / totalInvMass) * stiffness

        bodyA.velocity += impulse * bodyA.inverseMass
        bodyB.velocity -= impulse * bodyB.inverseMass
    }

    private func worldAnchor(body: Body, local: Vec2) -> Vec2 {
        let cosA = cos(body.angle)
        let sinA = sin(body.angle)
        return Vec2(
            cosA * local.x - sinA * local.y + body.position.x,
            sinA * local.x + cosA * local.y + body.position.y
        )
    }
}

/// Pins a body to a fixed world-space point.
public class PinJoint: Constraint {
    public let body: Body
    /// The world-space pin position.
    public let pinPosition: Vec2
    /// Local-space anchor on the body.
    public let anchor: Vec2
    /// Stiffness (0..1), 1 = rigid.
    public let stiffness: Double

    public init(body: Body, pinPosition: Vec2, anchor: Vec2 = .zero, stiffness: Double = 1.0) {
        self.body = body
        self.pinPosition = pinPosition
        self.anchor = anchor
        self.stiffness = stiffness
    }

    public func solve(dt: Double) {
        guard body.inverseMass > 0 else { return }

        let worldAnchor = self.worldAnchor()
        let delta = pinPosition - worldAnchor

        // Position correction
        body.position += delta * stiffness

        // Velocity damping toward pin
        let anchorVel = body.velocity + (worldAnchor - body.position).cross(body.angularVelocity)
        body.velocity -= anchorVel * stiffness
    }

    private func worldAnchor() -> Vec2 {
        let cosA = cos(body.angle)
        let sinA = sin(body.angle)
        return Vec2(
            cosA * anchor.x - sinA * anchor.y + body.position.x,
            sinA * anchor.x + cosA * anchor.y + body.position.y
        )
    }
}

/// A spring connecting two bodies with damping.
public class Spring: Constraint {
    public let bodyA: Body
    public let bodyB: Body
    /// Local-space anchor on body A.
    public let anchorA: Vec2
    /// Local-space anchor on body B.
    public let anchorB: Vec2
    /// Rest length of the spring.
    public let restLength: Double
    /// Spring constant (Hooke's law).
    public let k: Double
    /// Damping coefficient.
    public let damping: Double

    public init(bodyA: Body, bodyB: Body, anchorA: Vec2 = .zero, anchorB: Vec2 = .zero,
                restLength: Double? = nil, k: Double = 50.0, damping: Double = 1.0) {
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.anchorA = anchorA
        self.anchorB = anchorB
        self.k = k
        self.damping = damping

        if let rl = restLength {
            self.restLength = rl
        } else {
            let wA = bodyA.position + anchorA
            let wB = bodyB.position + anchorB
            self.restLength = wA.distance(to: wB)
        }
    }

    public func solve(dt: Double) {
        let worldA = worldAnchor(body: bodyA, local: anchorA)
        let worldB = worldAnchor(body: bodyB, local: anchorB)

        let delta = worldB - worldA
        let currentLength = delta.length
        guard currentLength > 1e-12 else { return }

        let normal = delta / currentLength
        let extension_ = currentLength - restLength

        // Spring force (Hooke's law)
        let springForce = normal * (k * extension_)

        // Damping force
        let relVel = bodyB.velocity - bodyA.velocity
        let dampingForce = normal * (damping * relVel.dot(normal))

        let totalForce = springForce + dampingForce

        bodyA.applyForce(totalForce)
        bodyB.applyForce(-totalForce)
    }

    private func worldAnchor(body: Body, local: Vec2) -> Vec2 {
        let cosA = cos(body.angle)
        let sinA = sin(body.angle)
        return Vec2(
            cosA * local.x - sinA * local.y + body.position.x,
            sinA * local.x + cosA * local.y + body.position.y
        )
    }
}
