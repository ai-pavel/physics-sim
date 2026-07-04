// swift-tools-version: 5.9

import PackageDescription

let package = Package(
    name: "PhysicsSim",
    targets: [
        .target(
            name: "PhysicsSim",
            path: "Sources/PhysicsSim"
        ),
        .executableTarget(
            name: "Demo",
            dependencies: ["PhysicsSim"],
            path: "Sources/Demo"
        ),
        .testTarget(
            name: "PhysicsSimTests",
            dependencies: ["PhysicsSim"],
            path: "Tests/PhysicsSimTests"
        ),
    ]
)
