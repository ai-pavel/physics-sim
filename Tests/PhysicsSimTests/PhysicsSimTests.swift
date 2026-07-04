import XCTest
@testable import PhysicsSim

final class Vec2Tests: XCTestCase {
    func testAddition() {
        let a = Vec2(1, 2)
        let b = Vec2(3, 4)
        let result = a + b
        XCTAssertEqual(result.x, 4)
        XCTAssertEqual(result.y, 6)
    }

    func testSubtraction() {
        let a = Vec2(5, 3)
        let b = Vec2(2, 1)
        let result = a - b
        XCTAssertEqual(result.x, 3)
        XCTAssertEqual(result.y, 2)
    }

    func testScalarMultiplication() {
        let v = Vec2(2, 3)
        let result = v * 2
        XCTAssertEqual(result.x, 4)
        XCTAssertEqual(result.y, 6)
    }

    func testDotProduct() {
        let a = Vec2(1, 0)
        let b = Vec2(0, 1)
        XCTAssertEqual(a.dot(b), 0)

        let c = Vec2(2, 3)
        let d = Vec2(4, 5)
        XCTAssertEqual(c.dot(d), 23) // 2*4 + 3*5
    }

    func testCrossProduct() {
        let a = Vec2(1, 0)
        let b = Vec2(0, 1)
        XCTAssertEqual(a.cross(b), 1)
    }

    func testLength() {
        let v = Vec2(3, 4)
        XCTAssertEqual(v.length, 5, accuracy: 1e-10)
    }

    func testNormalized() {
        let v = Vec2(3, 4)
        let n = v.normalized
        XCTAssertEqual(n.length, 1, accuracy: 1e-10)
        XCTAssertEqual(n.x, 0.6, accuracy: 1e-10)
        XCTAssertEqual(n.y, 0.8, accuracy: 1e-10)
    }

    func testPerpendicular() {
        let v = Vec2(1, 0)
        let p = v.perpendicular
        XCTAssertEqual(p.x, 0, accuracy: 1e-10)
        XCTAssertEqual(p.y, 1, accuracy: 1e-10)
    }

    func testZeroNormalized() {
        let v = Vec2.zero
        let n = v.normalized
        XCTAssertEqual(n.x, 0)
        XCTAssertEqual(n.y, 0)
    }
}

final class BodyTests: XCTestCase {
    func testBodyCreation() {
        let body = Body(shape: .circle(radius: 1), position: Vec2(0, 0), mass: 2.0)
        XCTAssertEqual(body.mass, 2.0)
        XCTAssertEqual(body.inverseMass, 0.5, accuracy: 1e-10)
        XCTAssertEqual(body.position.x, 0)
        XCTAssertEqual(body.position.y, 0)
    }

    func testStaticBody() {
        let body = Body(shape: .circle(radius: 1), position: Vec2(0, 0), isStatic: true)
        XCTAssertEqual(body.inverseMass, 0)
        XCTAssertEqual(body.inverseInertia, 0)
        XCTAssertTrue(body.isStatic)
    }

    func testApplyImpulse() {
        let body = Body(shape: .circle(radius: 1), position: Vec2(0, 0), mass: 2.0)
        body.applyImpulse(Vec2(4, 0))
        XCTAssertEqual(body.velocity.x, 2.0, accuracy: 1e-10) // impulse * inverseMass = 4 * 0.5
        XCTAssertEqual(body.velocity.y, 0)
    }

    func testApplyForce() {
        let body = Body(shape: .circle(radius: 1), position: Vec2(0, 0), mass: 1.0)
        body.applyForce(Vec2(10, -5))
        XCTAssertEqual(body.force.x, 10)
        XCTAssertEqual(body.force.y, -5)
        body.clearForces()
        XCTAssertEqual(body.force.x, 0)
        XCTAssertEqual(body.force.y, 0)
    }
}

final class ShapeTests: XCTestCase {
    func testCircleMomentOfInertia() {
        let shape = Shape.circle(radius: 2)
        let I = shape.momentOfInertia(mass: 3)
        XCTAssertEqual(I, 0.5 * 3 * 4, accuracy: 1e-10) // 0.5 * m * r^2
    }

    func testRectangleVertices() {
        let shape = Shape.rectangle(width: 2, height: 4)
        if case .polygon(let verts) = shape {
            XCTAssertEqual(verts.count, 4)
        } else {
            XCTFail("Expected polygon shape")
        }
    }

    func testWorldVertices() {
        let shape = Shape.rectangle(width: 2, height: 2)
        let verts = shape.worldVertices(position: Vec2(10, 10), angle: 0)
        XCTAssertEqual(verts.count, 4)
        // Unrotated: corners at (9,9), (11,9), (11,11), (9,11)
        XCTAssertEqual(verts[0].x, 9, accuracy: 1e-10)
        XCTAssertEqual(verts[0].y, 9, accuracy: 1e-10)
        XCTAssertEqual(verts[2].x, 11, accuracy: 1e-10)
        XCTAssertEqual(verts[2].y, 11, accuracy: 1e-10)
    }
}

final class CollisionTests: XCTestCase {
    func testCircleCircleCollision() {
        let detector = CollisionDetector()
        let a = Body(shape: .circle(radius: 1), position: Vec2(0, 0))
        let b = Body(shape: .circle(radius: 1), position: Vec2(1.5, 0))
        let contact = detector.detect(a: a, b: b)
        XCTAssertNotNil(contact)
        XCTAssertEqual(contact!.depth, 0.5, accuracy: 1e-10)
    }

    func testCircleCircleNoCollision() {
        let detector = CollisionDetector()
        let a = Body(shape: .circle(radius: 1), position: Vec2(0, 0))
        let b = Body(shape: .circle(radius: 1), position: Vec2(3, 0))
        let contact = detector.detect(a: a, b: b)
        XCTAssertNil(contact)
    }

    func testPolygonPolygonCollision() {
        let detector = CollisionDetector()
        let a = Body(shape: .rectangle(width: 2, height: 2), position: Vec2(0, 0))
        let b = Body(shape: .rectangle(width: 2, height: 2), position: Vec2(1.5, 0))
        let contact = detector.detect(a: a, b: b)
        XCTAssertNotNil(contact)
        XCTAssertEqual(contact!.depth, 0.5, accuracy: 1e-10)
    }

    func testPolygonPolygonNoCollision() {
        let detector = CollisionDetector()
        let a = Body(shape: .rectangle(width: 2, height: 2), position: Vec2(0, 0))
        let b = Body(shape: .rectangle(width: 2, height: 2), position: Vec2(5, 0))
        let contact = detector.detect(a: a, b: b)
        XCTAssertNil(contact)
    }
}

final class CollisionResolutionTests: XCTestCase {
    func testCircleBounce() {
        let detector = CollisionDetector()
        let resolver = CollisionResolver()

        let a = Body(shape: .circle(radius: 1), position: Vec2(0, 0), mass: 1.0, restitution: 1.0, isStatic: true)
        let b = Body(shape: .circle(radius: 1), position: Vec2(0, 1.5), mass: 1.0, restitution: 1.0)
        b.velocity = Vec2(0, -5)

        if let contact = detector.detect(a: a, b: b) {
            resolver.resolve(contact)
            // With restitution 1.0, the ball should bounce back
            XCTAssertGreaterThan(b.velocity.y, 0)
        } else {
            XCTFail("Expected collision")
        }
    }
}

final class WorldTests: XCTestCase {
    func testGravity() {
        let world = World(gravity: Vec2(0, -10))
        let body = Body(shape: .circle(radius: 1), position: Vec2(0, 10), mass: 1.0)
        world.addBody(body)

        // Step for 1 second (60 steps)
        for _ in 0..<60 {
            world.step(dt: 1.0 / 60.0)
        }

        // Body should have fallen
        XCTAssertLessThan(body.position.y, 10)
    }

    func testFloorCollision() {
        let world = World(gravity: Vec2(0, -10))

        let floor = Body(shape: .rectangle(width: 100, height: 2), position: Vec2(0, -1), isStatic: true)
        floor.restitution = 0.5
        world.addBody(floor)

        let ball = Body(shape: .circle(radius: 0.5), position: Vec2(0, 5), mass: 1.0, restitution: 0.5)
        world.addBody(ball)

        // Simulate 3 seconds
        for _ in 0..<180 {
            world.step(dt: 1.0 / 60.0)
        }

        // Ball should not have fallen through the floor (floor top is at y=0)
        XCTAssertGreaterThan(ball.position.y, -0.5, "Ball should not fall through floor")
    }

    func testAddRemoveBody() {
        let world = World()
        let body = Body(shape: .circle(radius: 1), position: Vec2(0, 0))
        world.addBody(body)
        XCTAssertEqual(world.bodies.count, 1)
        world.removeBody(body)
        XCTAssertEqual(world.bodies.count, 0)
    }
}

final class ConstraintTests: XCTestCase {
    func testDistanceJoint() {
        let world = World(gravity: Vec2(0, 0))
        let a = Body(shape: .circle(radius: 0.5), position: Vec2(0, 0), mass: 1.0)
        let b = Body(shape: .circle(radius: 0.5), position: Vec2(3, 0), mass: 1.0)
        world.addBody(a)
        world.addBody(b)

        let joint = DistanceJoint(bodyA: a, bodyB: b, distance: 3.0)
        world.addConstraint(joint)

        // Push them apart
        b.velocity = Vec2(10, 0)

        // Step simulation
        for _ in 0..<60 {
            world.step(dt: 1.0 / 60.0)
        }

        // Distance should stay close to 3.0
        let dist = a.position.distance(to: b.position)
        XCTAssertEqual(dist, 3.0, accuracy: 1.0) // Allow some tolerance
    }

    func testSpringOscillation() {
        let world = World(gravity: Vec2(0, 0))
        let a = Body(shape: .circle(radius: 0.5), position: Vec2(0, 0), mass: 1.0, isStatic: true)
        let b = Body(shape: .circle(radius: 0.5), position: Vec2(5, 0), mass: 1.0)
        world.addBody(a)
        world.addBody(b)

        let spring = Spring(bodyA: a, bodyB: b, restLength: 2.0, k: 20.0, damping: 0.5)
        world.addConstraint(spring)

        // The spring should pull body b toward rest length
        let initialDist = a.position.distance(to: b.position)
        XCTAssertEqual(initialDist, 5.0, accuracy: 1e-10)

        for _ in 0..<120 {
            world.step(dt: 1.0 / 60.0)
        }

        // Body b should have moved closer due to spring force
        let finalDist = a.position.distance(to: b.position)
        XCTAssertLessThan(finalDist, initialDist)
    }
}

final class IntegratorTests: XCTestCase {
    func testVerletFreefall() {
        let integrator = VerletIntegrator()
        let body = Body(shape: .circle(radius: 1), position: Vec2(0, 100), mass: 1.0)
        let gravity = Vec2(0, -10)

        // Integrate for 1 second
        for _ in 0..<60 {
            integrator.integrate(body: body, dt: 1.0 / 60.0, gravity: gravity)
        }

        // After 1s of freefall: y = 100 - 0.5*10*1^2 = 95
        XCTAssertEqual(body.position.y, 95, accuracy: 1.0)
        // Velocity should be approximately -10 m/s
        XCTAssertEqual(body.velocity.y, -10, accuracy: 1.0)
    }

    func testStaticBodyNotAffected() {
        let integrator = VerletIntegrator()
        let body = Body(shape: .circle(radius: 1), position: Vec2(5, 5), isStatic: true)
        let gravity = Vec2(0, -10)

        integrator.integrate(body: body, dt: 1.0 / 60.0, gravity: gravity)

        XCTAssertEqual(body.position.x, 5)
        XCTAssertEqual(body.position.y, 5)
    }
}
