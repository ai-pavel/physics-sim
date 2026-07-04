# PhysicsSim

[![CI](https://github.com/ai-pavel/physics-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/ai-pavel/physics-sim/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/ai-pavel/physics-sim/branch/main/graph/badge.svg)](https://codecov.io/gh/ai-pavel/physics-sim)

A 2D rigid body physics engine implemented in Swift.

## Features

- **Vec2 math**: Full 2D vector operations (add, subtract, multiply, dot, cross, normalize, perpendicular)
- **Rigid bodies**: Mass, velocity, angular velocity, restitution, friction
- **Shapes**: Circles and convex polygons (with rectangle convenience constructor)
- **Collision detection**: SAT (Separating Axis Theorem) for polygon-polygon, circle-circle, and circle-polygon
- **Collision resolution**: Impulse-based physics with friction and positional correction
- **Constraints**: Distance joint, pin joint, and spring (Hooke's law with damping)
- **Integration**: Velocity Verlet integrator with gravity and damping

## Project Structure

```
Sources/
  PhysicsSim/
    Vec2.swift         - 2D vector math
    Body.swift         - Rigid body with physical properties
    Shape.swift        - Circle and convex polygon shapes
    Collision.swift    - SAT collision detection and impulse resolution
    Constraint.swift   - Distance joint, pin joint, spring
    Integrator.swift   - Velocity Verlet integrator
    World.swift        - Physics world simulation loop
  Demo/
    main.swift         - CLI demo simulating bouncing shapes
Tests/
  PhysicsSimTests/
    PhysicsSimTests.swift - XCTest unit tests
```

## Build and Run

```bash
# Build
swift build

# Run the demo
swift run Demo

# Run tests
swift test
```

## Usage

```swift
import PhysicsSim

let world = World(gravity: Vec2(0, -9.81))

// Create a static floor
let floor = Body(shape: .rectangle(width: 100, height: 2), position: Vec2(0, -1), isStatic: true)
world.addBody(floor)

// Create a bouncing ball
let ball = Body(shape: .circle(radius: 0.5), position: Vec2(0, 10), mass: 1.0, restitution: 0.7)
world.addBody(ball)

// Simulate
for _ in 0..<600 {
    world.step(dt: 1.0 / 60.0)
    print(ball.position)
}
```
