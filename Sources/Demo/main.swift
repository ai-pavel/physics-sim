import PhysicsSim
import Foundation

// Create a physics world with downward gravity
let world = World(gravity: Vec2(0, -9.81))

// Create a static floor (large rectangle at y = 0)
let floor = Body(
    shape: .rectangle(width: 100, height: 2),
    position: Vec2(0, -1),
    isStatic: true
)
floor.restitution = 0.6
world.addBody(floor)

// Create a bouncing circle
let circle = Body(
    shape: .circle(radius: 0.5),
    position: Vec2(0, 10),
    mass: 1.0,
    restitution: 0.7
)
world.addBody(circle)

// Create a bouncing rectangle
let box = Body(
    shape: .rectangle(width: 1.0, height: 1.0),
    position: Vec2(2, 8),
    mass: 2.0,
    restitution: 0.5
)
box.angularVelocity = 1.5
world.addBody(box)

// Create a triangle
let triangle = Body(
    shape: .polygon(vertices: [
        Vec2(0, 0.6),
        Vec2(-0.5, -0.3),
        Vec2(0.5, -0.3),
    ]),
    position: Vec2(-2, 12),
    mass: 1.5,
    restitution: 0.6
)
world.addBody(triangle)

// Create two bodies connected by a spring
let springBodyA = Body(
    shape: .circle(radius: 0.3),
    position: Vec2(5, 6),
    mass: 0.5,
    restitution: 0.4
)
world.addBody(springBodyA)

let springBodyB = Body(
    shape: .circle(radius: 0.3),
    position: Vec2(5, 3),
    mass: 0.5,
    restitution: 0.4
)
world.addBody(springBodyB)

let spring = Spring(bodyA: springBodyA, bodyB: springBodyB, restLength: 2.0, k: 30.0, damping: 0.5)
world.addConstraint(spring)

// Simulation parameters
let dt = 1.0 / 60.0
let totalFrames = 120  // 2 seconds at 60fps
let printInterval = 10  // Print every 10 frames

print("=== 2D Rigid Body Physics Simulation ===")
print("Bodies: circle, box (rotating), triangle, 2 spring-connected circles")
print("Gravity: \(world.gravity)")
print("Time step: \(dt)s, Frames: \(totalFrames)")
print(String(repeating: "-", count: 70))

for frame in 0..<totalFrames {
    world.step(dt: dt)

    if frame % printInterval == 0 {
        let time = String(format: "%.2f", Double(frame) * dt)
        print("Frame \(String(format: "%3d", frame)) (t=\(time)s):")
        print("  Circle    pos=\(circle.position)  vel=\(circle.velocity)")
        print("  Box       pos=\(box.position)  angle=\(String(format: "%.2f", box.angle)) rad")
        print("  Triangle  pos=\(triangle.position)")
        print("  Spring A  pos=\(springBodyA.position)")
        print("  Spring B  pos=\(springBodyB.position)")
    }
}

print(String(repeating: "-", count: 70))
print("Simulation complete.")
