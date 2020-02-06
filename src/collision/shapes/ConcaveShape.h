/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef REACTPHYSICS3D_CONCAVE_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_SHAPE_H

// Libraries
#include "CollisionShape.h"
#include "TriangleShape.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class TriangleCallback
/**
 * This class is used to encapsulate a callback method for
 * a single triangle of a ConcaveMesh.
 */
class TriangleCallback {

    public:

        /// Destructor
        virtual ~TriangleCallback() = default;

        /// Report a triangle
        virtual void testTriangle(const Vector3* trianglePoints, const Vector3* verticesNormals, uint shapeId)=0;

};


// Class ConcaveShape
/**
 * This abstract class represents a concave collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConcaveShape : public CollisionShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Raycast test type for the triangle (front, back, front-back)
        TriangleRaycastSide mRaycastTestType;

        /// Scale of the shape
        Vector3 mScale;

        // -------------------- Methods -------------------- //

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, Collider* collider) const override;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveShape(CollisionShapeName name, const Vector3& scaling);

        /// Destructor
        virtual ~ConcaveShape() override = default;

        /// Deleted copy-constructor
        ConcaveShape(const ConcaveShape& shape) = delete;

        /// Deleted assignment operator
        ConcaveShape& operator=(const ConcaveShape& shape) = delete;

        /// Return the raycast test type (front, back, front-back)
        TriangleRaycastSide getRaycastTestType() const;

        // Set the raycast test type (front, back, front-back)
        void setRaycastTestType(TriangleRaycastSide testType);

        /// Return the scale of the shape
        const Vector3& getScale() const;

        /// Set the scale of the shape
        void setScale(const Vector3& scale);

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const override;

        /// Return true if the collision shape is a polyhedron
        virtual bool isPolyhedron() const override;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void computeOverlappingTriangles(const AABB& localAABB, List<Vector3>& triangleVertices,
                                                 List<Vector3>& triangleVerticesNormals, List<uint>& shapeIds,
                                                 MemoryAllocator& allocator) const=0;
};

// Return true if the collision shape is convex, false if it is concave
inline bool ConcaveShape::isConvex() const {
    return false;
}

// Return true if the collision shape is a polyhedron
inline bool ConcaveShape::isPolyhedron() const {
    return true;
}

// Return true if a point is inside the collision shape
inline bool ConcaveShape::testPointInside(const Vector3& localPoint, Collider* collider) const {
    return false;
}

// Return the raycast test type (front, back, front-back)
inline TriangleRaycastSide ConcaveShape::getRaycastTestType() const {
    return mRaycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
inline void ConcaveShape::setRaycastTestType(TriangleRaycastSide testType) {
    mRaycastTestType = testType;
}

// Return the scale of the shape
inline const Vector3& ConcaveShape::getScale() const {
    return mScale;
}

// Set the scale of the shape
/// Note that you might want to recompute the inertia tensor and center of mass of the body
/// after changing the scale of a collision shape
inline void ConcaveShape::setScale(const Vector3& scale) {
    mScale = scale;
}

}

#endif

