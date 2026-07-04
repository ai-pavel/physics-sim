import Foundation

/// Information about a collision between two bodies.
public struct Contact {
    /// The two colliding bodies.
    public let bodyA: Body
    public let bodyB: Body
    /// Collision normal (points from A to B).
    public let normal: Vec2
    /// Penetration depth.
    public let depth: Double
    /// Contact point in world space.
    public let point: Vec2
}

/// Collision detection using SAT for polygons and geometric tests for circles.
public struct CollisionDetector {

    public init() {}

    /// Detect collision between two bodies. Returns a Contact if colliding, nil otherwise.
    public func detect(a: Body, b: Body) -> Contact? {
        switch (a.shape, b.shape) {
        case (.circle(let rA), .circle(let rB)):
            return circleVsCircle(a: a, radiusA: rA, b: b, radiusB: rB)
        case (.circle(let rA), .polygon):
            if let c = circleVsPolygon(circle: a, radius: rA, polygon: b) {
                return c
            }
            return nil
        case (.polygon, .circle(let rB)):
            // circleVsPolygon already returns bodyA=polygon, bodyB=circle with correct normal
            return circleVsPolygon(circle: b, radius: rB, polygon: a)
        case (.polygon, .polygon):
            return polygonVsPolygon(a: a, b: b)
        }
    }

    // MARK: - Circle vs Circle

    private func circleVsCircle(a: Body, radiusA: Double, b: Body, radiusB: Double) -> Contact? {
        let diff = b.position - a.position
        let distSq = diff.lengthSquared
        let sumR = radiusA + radiusB
        guard distSq < sumR * sumR else { return nil }

        let dist = sqrt(distSq)
        let normal: Vec2
        if dist < 1e-12 {
            normal = Vec2(1, 0)
        } else {
            normal = diff / dist
        }
        let depth = sumR - dist
        let point = a.position + normal * (radiusA - depth / 2)
        return Contact(bodyA: a, bodyB: b, normal: normal, depth: depth, point: point)
    }

    // MARK: - Circle vs Polygon

    private func circleVsPolygon(circle: Body, radius: Double, polygon: Body) -> Contact? {
        let vertices = polygon.shape.worldVertices(position: polygon.position, angle: polygon.angle)
        let n = vertices.count
        guard n >= 3 else { return nil }

        var minDepth = Double.infinity
        var bestNormal = Vec2.zero
        var bestPoint = Vec2.zero

        // Check polygon edge normals
        for i in 0..<n {
            let a = vertices[i]
            let b = vertices[(i + 1) % n]
            let edge = b - a
            let axis = Vec2(edge.y, -edge.x).normalized

            let (minP, maxP) = projectPolygon(vertices: vertices, axis: axis)
            let circleProj = circle.position.dot(axis)
            let minC = circleProj - radius
            let maxC = circleProj + radius

            if minP >= maxC || minC >= maxP { return nil }

            let overlap = min(maxP - minC, maxC - minP)
            if overlap < minDepth {
                minDepth = overlap
                bestNormal = axis
            }
        }

        // Check axis from circle center to closest vertex
        var closestDist = Double.infinity
        var closestVertex = vertices[0]
        for v in vertices {
            let d = (v - circle.position).lengthSquared
            if d < closestDist {
                closestDist = d
                closestVertex = v
            }
        }

        let vertexAxis = (closestVertex - circle.position).normalized
        if vertexAxis.lengthSquared > 0.5 {
            let (minP, maxP) = projectPolygon(vertices: vertices, axis: vertexAxis)
            let circleProj = circle.position.dot(vertexAxis)
            let minC = circleProj - radius
            let maxC = circleProj + radius

            if minP >= maxC || minC >= maxP { return nil }

            let overlap = min(maxP - minC, maxC - minP)
            if overlap < minDepth {
                minDepth = overlap
                bestNormal = vertexAxis
            }
        }

        // Ensure normal points from polygon to circle
        let dir = circle.position - polygon.position
        if bestNormal.dot(dir) < 0 {
            bestNormal = -bestNormal
        }

        bestPoint = circle.position - bestNormal * radius

        return Contact(bodyA: polygon, bodyB: circle, normal: bestNormal, depth: minDepth, point: bestPoint)
    }

    // MARK: - Polygon vs Polygon (SAT)

    private func polygonVsPolygon(a: Body, b: Body) -> Contact? {
        let vertsA = a.shape.worldVertices(position: a.position, angle: a.angle)
        let vertsB = b.shape.worldVertices(position: b.position, angle: b.angle)
        guard vertsA.count >= 3, vertsB.count >= 3 else { return nil }

        var minDepth = Double.infinity
        var bestNormal = Vec2.zero

        // Check axes from A's edges
        for i in 0..<vertsA.count {
            let edge = vertsA[(i + 1) % vertsA.count] - vertsA[i]
            let axis = Vec2(edge.y, -edge.x).normalized
            guard axis.lengthSquared > 0.5 else { continue }

            let (minA, maxA) = projectPolygon(vertices: vertsA, axis: axis)
            let (minB, maxB) = projectPolygon(vertices: vertsB, axis: axis)

            if minA >= maxB || minB >= maxA { return nil }

            let overlap = min(maxA - minB, maxB - minA)
            if overlap < minDepth {
                minDepth = overlap
                bestNormal = axis
            }
        }

        // Check axes from B's edges
        for i in 0..<vertsB.count {
            let edge = vertsB[(i + 1) % vertsB.count] - vertsB[i]
            let axis = Vec2(edge.y, -edge.x).normalized
            guard axis.lengthSquared > 0.5 else { continue }

            let (minA, maxA) = projectPolygon(vertices: vertsA, axis: axis)
            let (minB, maxB) = projectPolygon(vertices: vertsB, axis: axis)

            if minA >= maxB || minB >= maxA { return nil }

            let overlap = min(maxA - minB, maxB - minA)
            if overlap < minDepth {
                minDepth = overlap
                bestNormal = axis
            }
        }

        // Ensure normal points from A to B
        let dir = b.position - a.position
        if bestNormal.dot(dir) < 0 {
            bestNormal = -bestNormal
        }

        // Find contact point (deepest penetrating vertex)
        let contactPoint = findContactPoint(vertsA: vertsA, vertsB: vertsB, normal: bestNormal)

        return Contact(bodyA: a, bodyB: b, normal: bestNormal, depth: minDepth, point: contactPoint)
    }

    // MARK: - Helpers

    private func projectPolygon(vertices: [Vec2], axis: Vec2) -> (min: Double, max: Double) {
        var minProj = vertices[0].dot(axis)
        var maxProj = minProj
        for i in 1..<vertices.count {
            let proj = vertices[i].dot(axis)
            minProj = min(minProj, proj)
            maxProj = max(maxProj, proj)
        }
        return (minProj, maxProj)
    }

    private func findContactPoint(vertsA: [Vec2], vertsB: [Vec2], normal: Vec2) -> Vec2 {
        // Find the vertex of B most penetrating into A (most negative projection along normal from A's perspective)
        var bestProj = Double.infinity
        var bestPoint = vertsB[0]

        for v in vertsB {
            let proj = v.dot(normal)
            if proj < bestProj {
                bestProj = proj
                bestPoint = v
            }
        }

        // Also check vertices of A
        for v in vertsA {
            let proj = -(v.dot(normal))
            if proj < bestProj {
                bestProj = proj
                bestPoint = v
            }
        }

        return bestPoint
    }
}

/// Resolves collisions using impulse-based physics.
public struct CollisionResolver {

    public init() {}

    /// Resolve a contact by applying impulses to the involved bodies.
    public func resolve(_ contact: Contact) {
        let a = contact.bodyA
        let b = contact.bodyB

        // Skip if both are static
        guard a.inverseMass + b.inverseMass > 0 else { return }

        let rA = contact.point - a.position
        let rB = contact.point - b.position

        // Relative velocity at contact point
        let velA = a.velocity + rA.cross(a.angularVelocity)
        let velB = b.velocity + rB.cross(b.angularVelocity)
        let relativeVelocity = velB - velA

        let contactVel = relativeVelocity.dot(contact.normal)

        // Don't resolve if bodies are separating
        guard contactVel < 0 else { return }

        let rACrossN = rA.cross(contact.normal)
        let rBCrossN = rB.cross(contact.normal)

        let invMassSum = a.inverseMass + b.inverseMass
            + rACrossN * rACrossN * a.inverseInertia
            + rBCrossN * rBCrossN * b.inverseInertia

        let restitution = min(a.restitution, b.restitution)
        let j = -(1 + restitution) * contactVel / invMassSum

        let impulse = contact.normal * j

        a.applyImpulse(-impulse, at: contact.point)
        b.applyImpulse(impulse, at: contact.point)

        // Friction impulse
        let tangent = (relativeVelocity - contact.normal * contactVel).normalized
        if tangent.lengthSquared > 0.1 {
            let tangentVel = relativeVelocity.dot(tangent)
            let rACrossT = rA.cross(tangent)
            let rBCrossT = rB.cross(tangent)
            let invMassSumT = a.inverseMass + b.inverseMass
                + rACrossT * rACrossT * a.inverseInertia
                + rBCrossT * rBCrossT * b.inverseInertia

            var jt = -tangentVel / invMassSumT
            let frictionCoeff = (a.friction + b.friction) / 2

            // Coulomb's law: clamp friction impulse
            if abs(jt) > j * frictionCoeff {
                jt = j * frictionCoeff * (jt > 0 ? 1 : -1)
            }

            let frictionImpulse = tangent * jt
            a.applyImpulse(-frictionImpulse, at: contact.point)
            b.applyImpulse(frictionImpulse, at: contact.point)
        }

        // Positional correction to prevent sinking
        let correction = max(contact.depth - 0.01, 0) / invMassSum * 0.4 * contact.normal
        a.position -= correction * a.inverseMass
        b.position += correction * b.inverseMass
    }
}
