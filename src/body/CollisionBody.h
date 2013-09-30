/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_COLLISION_BODY_H
#define REACTPHYSICS3D_COLLISION_BODY_H

// Libraries
#include <stdexcept>
#include <cassert>
#include "Body.h"
#include "../mathematics/Transform.h"
#include "../collision/shapes/AABB.h"
#include "../collision/shapes/CollisionShape.h"
#include "../memory/MemoryAllocator.h"
#include "../configuration.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Class declarations
struct ContactManifoldListElement;

// Class CollisionBody
/**
 * This class represents a body that is able to collide with others
 * bodies. This class inherits from the Body class.
 */
class CollisionBody : public Body {

    protected :

        // -------------------- Attributes -------------------- //

        /// Collision shape of the body
        CollisionShape* mCollisionShape;

        /// Position and orientation of the body
        Transform mTransform;

        /// Last position and orientation of the body
        Transform mOldTransform;

        /// Interpolation factor used for the state interpolation
        decimal mInterpolationFactor;

        /// True if the body is able to move
        bool mIsMotionEnabled;

        /// True if the body can collide with others bodies
        bool mIsCollisionEnabled;

        /// AABB for Broad-Phase collision detection
        AABB mAabb;

        /// True if the body has moved during the last frame
        bool mHasMoved;

        /// First element of the linked list of contact manifolds involving this body
        ContactManifoldListElement* mContactManifoldsList;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionBody(const CollisionBody& body);

        /// Private assignment operator
        CollisionBody& operator=(const CollisionBody& body);

        /// Reset the contact manifold lists
        void resetContactManifoldsList(MemoryAllocator& memoryAllocator);

        /// Update the old transform with the current one.
        void updateOldTransform();

        /// Update the Axis-Aligned Bounding Box coordinates
        void updateAABB();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionBody(const Transform& transform, CollisionShape* collisionShape, bodyindex id);

        /// Destructor
        virtual ~CollisionBody();

        /// Return the collision shape
        CollisionShape* getCollisionShape() const;

        /// Set the collision shape
        void setCollisionShape(CollisionShape* collisionShape);

        /// Return the current position and orientation
        const Transform& getTransform() const;

        /// Set the current position and orientation
        void setTransform(const Transform& transform);

        /// Return the AAABB of the body
        const AABB& getAABB() const;

        /// Return the interpolated transform for rendering
        Transform getInterpolatedTransform() const;

        /// Set the interpolation factor of the body
        void setInterpolationFactor(decimal factor);

        /// Return true if the rigid body is allowed to move
        bool isMotionEnabled() const;

        /// Enable/disable the motion of the body
        void enableMotion(bool isMotionEnabled);

        /// Return true if the body can collide with others bodies
        bool isCollisionEnabled() const;

        /// Enable/disable the collision with this body
        void enableCollision(bool isCollisionEnabled);

        /// Return the first element of the linked list of contact manifolds involving this body
        const ContactManifoldListElement* getContactManifoldsLists() const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
        friend class CollisionDetection;
};

// Return the collision shape
inline CollisionShape* CollisionBody::getCollisionShape() const {
    assert(mCollisionShape);
    return mCollisionShape;
}

// Set the collision shape
inline void CollisionBody::setCollisionShape(CollisionShape* collisionShape) {
    assert(collisionShape);
    mCollisionShape = collisionShape;
}

// Return the interpolated transform for rendering
inline Transform CollisionBody::getInterpolatedTransform() const {
    return Transform::interpolateTransforms(mOldTransform, mTransform, mInterpolationFactor);
}

// Set the interpolation factor of the body
inline void CollisionBody::setInterpolationFactor(decimal factor) {
    // Set the factor
    mInterpolationFactor = factor;
}

// Return true if the rigid body is allowed to move
inline bool CollisionBody::isMotionEnabled() const {
    return mIsMotionEnabled;
}

// Enable/disable the motion of the body
inline void CollisionBody::enableMotion(bool isMotionEnabled) {
    mIsMotionEnabled = isMotionEnabled;
}

// Return the current position and orientation
inline const Transform& CollisionBody::getTransform() const {
    return mTransform;
}

// Set the current position and orientation
inline void CollisionBody::setTransform(const Transform& transform) {

    // Check if the body has moved
    if (mTransform != transform) {
        mHasMoved = true;
    }

    mTransform = transform;
}

// Return the AAABB of the body
inline const AABB& CollisionBody::getAABB() const {
    return mAabb;
}

 // Return true if the body can collide with others bodies
inline bool CollisionBody::isCollisionEnabled() const {
    return mIsCollisionEnabled;
}

// Enable/disable the collision with this body
inline void CollisionBody::enableCollision(bool isCollisionEnabled) {
    mIsCollisionEnabled = isCollisionEnabled;
}

// Update the old transform with the current one.
/// This is used to compute the interpolated position and orientation of the body
inline void CollisionBody::updateOldTransform() {
    mOldTransform = mTransform;
}

// Update the rigid body in order to reflect a change in the body state
inline void CollisionBody::updateAABB() {

    // Update the AABB
    mCollisionShape->updateAABB(mAabb, mTransform);
}

// Return the first element of the linked list of contact manifolds involving this body
inline const ContactManifoldListElement* CollisionBody::getContactManifoldsLists() const {
    return mContactManifoldsList;
}

}

 #endif