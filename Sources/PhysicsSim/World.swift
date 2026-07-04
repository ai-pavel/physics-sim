import Foundation

/// The physics world containing all bodies, constraints, and simulation logic.
public class World {
    /// All rigid bodies in the world.
    public private(set) var bodies: [Body] = []
    /// All constraints in the world.
    public private(set) var constraints: [Constraint] = []
    /// Gravity vector (default: Earth-like downward gravity).
    public var gravity: Vec2
    /// The integrator used for time-stepping.
    public var integrator: Integrator
    /// Number of constraint solver iterations per step.
    public var constraintIterations: Int = 4

    private let detector = CollisionDetector()
    private let resolver = CollisionResolver()

    public init(gravity: Vec2 = Vec2(0, -9.81), integrator: Integrator = VerletIntegrator()) {
        self.gravity = gravity
        self.integrator = integrator
    }

    /// Add a body to the world.
    @discardableResult
    public func addBody(_ body: Body) -> Body {
        bodies.append(body)
        return body
    }

    /// Remove a body from the world.
    public func removeBody(_ body: Body) {
        bodies.removeAll { $0 === body }
    }

    /// Add a constraint to the world.
    public func addConstraint(_ constraint: Constraint) {
        constraints.append(constraint)
    }

    /// Step the simulation forward by dt seconds.
    public func step(dt: Double) {
        // 1. Integrate positions and velocities
        for body in bodies {
            integrator.integrate(body: body, dt: dt, gravity: gravity)
        }

        // 2. Solve constraints
        for _ in 0..<constraintIterations {
            for constraint in constraints {
                constraint.solve(dt: dt)
            }
        }

        // 3. Detect and resolve collisions
        for i in 0..<bodies.count {
            for j in (i + 1)..<bodies.count {
                let a = bodies[i]
                let b = bodies[j]

                // Skip if both are static
                if a.isStatic && b.isStatic { continue }

                if let contact = detector.detect(a: a, b: b) {
                    resolver.resolve(contact)
                }
            }
        }
    }
}
