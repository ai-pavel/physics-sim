import Foundation

/// Physics integrator protocol.
public protocol Integrator {
    func integrate(body: Body, dt: Double, gravity: Vec2)
}

/// Velocity Verlet integrator with gravity support.
public struct VerletIntegrator: Integrator {

    public init() {}

    public func integrate(body: Body, dt: Double, gravity: Vec2) {
        guard !body.isStatic, body.inverseMass > 0 else { return }

        // Apply gravity
        let acceleration = gravity + body.force * body.inverseMass
        let angularAcceleration = body.torque * body.inverseInertia

        // Velocity Verlet integration
        // v(t + dt) = v(t) + a(t) * dt
        body.velocity += acceleration * dt
        body.angularVelocity += angularAcceleration * dt

        // Store previous position for Verlet
        body.previousPosition = body.position
        body.previousAngle = body.angle

        // x(t + dt) = x(t) + v(t + dt) * dt
        body.position += body.velocity * dt
        body.angle += body.angularVelocity * dt

        // Apply damping
        body.velocity *= 0.999
        body.angularVelocity *= 0.999

        body.clearForces()
    }
}
