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

// Libraries
#include "DynamicsWorld.h"
#include "constraint/BallAndSocketJoint.h"
#include "constraint/SliderJoint.h"
#include "constraint/HingeJoint.h"
#include "constraint/FixedJoint.h"

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
DynamicsWorld::DynamicsWorld(const Vector3 &gravity, decimal timeStep = DEFAULT_TIMESTEP)
              : CollisionWorld(), mTimer(timeStep), mGravity(gravity), mIsGravityEnabled(true),
                mConstrainedLinearVelocities(NULL), mConstrainedAngularVelocities(NULL),
                mContactSolver(mMapBodyToConstrainedVelocityIndex),
                mConstraintSolver(mConstrainedPositions, mConstrainedOrientations,
                                  mMapBodyToConstrainedVelocityIndex),
                mNbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
                mNbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS),
                mIsSleepingEnabled(SPLEEPING_ENABLED), mSplitLinearVelocities(NULL),
                mSplitAngularVelocities(NULL), mNbIslands(0), mNbIslandsCapacity(0),
                mIslands(NULL), mNbBodiesCapacity(0),
                mSleepLinearVelocity(DEFAULT_SLEEP_LINEAR_VELOCITY),
                mSleepAngularVelocity(DEFAULT_SLEEP_ANGULAR_VELOCITY),
                mTimeBeforeSleep(DEFAULT_TIME_BEFORE_SLEEP), mEventListener(NULL) {

}

// Destructor
DynamicsWorld::~DynamicsWorld() {

    // Delete the remaining overlapping pairs
    map<std::pair<bodyindex, bodyindex>, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); it++) {
        // Delete the overlapping pair
        (*it).second->OverlappingPair::~OverlappingPair();
        mMemoryAllocator.release((*it).second, sizeof(OverlappingPair));
    }

    // Release the memory allocated for the islands
    for (uint i=0; i<mNbIslands; i++) {

        // Call the island destructor
        mIslands[i]->Island::~Island();

        // Release the allocated memory for the island
        mMemoryAllocator.release(mIslands[i], sizeof(Island));
    }
    if (mNbIslandsCapacity > 0) {
        mMemoryAllocator.release(mIslands, sizeof(Island*) * mNbIslandsCapacity);
    }

    // Release the memory allocated for the bodies velocity arrays
    if (mNbBodiesCapacity > 0) {
        delete[] mSplitLinearVelocities;
        delete[] mSplitAngularVelocities;
        delete[] mConstrainedLinearVelocities;
        delete[] mConstrainedAngularVelocities;
    }

#ifdef IS_PROFILING_ACTIVE

    // Print the profiling report
    Profiler::printReport(std::cout);

    // Destroy the profiler (release the allocated memory)
    Profiler::destroy();
#endif

}

// Update the physics simulation
void DynamicsWorld::update() {

#ifdef IS_PROFILING_ACTIVE
    // Increment the frame counter of the profiler
    Profiler::incrementFrameCounter();
#endif

    PROFILE("DynamicsWorld::update()");

    assert(mTimer.getIsRunning());
    
    // Compute the time since the last update() call and update the timer
    mTimer.update();

    // While the time accumulator is not empty
    while(mTimer.isPossibleToTakeStep()) {

        // Remove all contact manifolds
        mContactManifolds.clear();

        // Reset all the contact manifolds lists of each body
        resetContactManifoldListsOfBodies();
		
        // Compute the collision detection
        mCollisionDetection.computeCollisionDetection();

        // Compute the islands (separate groups of bodies with constraints between each others)
        computeIslands();

        // Integrate the velocities
        integrateRigidBodiesVelocities();

        // Reset the movement boolean variable of each body to false
        resetBodiesMovementVariable();

        // Update the timer
        mTimer.nextStep();

        // Solve the contacts and constraints
        solveContactsAndConstraints();

        // Integrate the position and orientation of each body
        integrateRigidBodiesPositions();

        // Solve the position correction for constraints
        solvePositionCorrection();

        if (mIsSleepingEnabled) updateSleepingBodies();

        // Update the AABBs of the bodies
        updateRigidBodiesAABB();
    }

    // Reset the external force and torque applied to the bodies
    resetBodiesForceAndTorque();

    // Compute and set the interpolation factor to all the bodies
    setInterpolationFactorToAllBodies();
}

// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are integrated using
/// the sympletic Euler time stepping scheme.
void DynamicsWorld::integrateRigidBodiesPositions() {

    PROFILE("DynamicsWorld::integrateRigidBodiesPositions()");

    decimal dt = static_cast<decimal>(mTimer.getTimeStep());
    
    // For each island of the world
    for (uint i=0; i < mNbIslands; i++) {

        RigidBody** bodies = mIslands[i]->getBodies();

        // For each body of the island
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // If the body is allowed to move
            if (bodies[b]->isMotionEnabled()) {

                // Get the constrained velocity
                uint indexArray = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;
                Vector3 newLinVelocity = mConstrainedLinearVelocities[indexArray];
                Vector3 newAngVelocity = mConstrainedAngularVelocities[indexArray];

                // Update the linear and angular velocity of the body
                bodies[b]->setLinearVelocity(newLinVelocity);
                bodies[b]->setAngularVelocity(newAngVelocity);

                // Add the split impulse velocity from Contact Solver (only used
                // to update the position)
                if (mContactSolver.isSplitImpulseActive()) {

                    newLinVelocity += mSplitLinearVelocities[indexArray];
                    newAngVelocity += mSplitAngularVelocities[indexArray];
                }

                // Get current position and orientation of the body
                const Vector3& currentPosition = bodies[b]->getTransform().getPosition();
                const Quaternion& currentOrientation = bodies[b]->getTransform().getOrientation();

                // Compute the new position of the body
                Vector3 newPosition = currentPosition + newLinVelocity * dt;
                Quaternion newOrientation = currentOrientation + Quaternion(0, newAngVelocity) *
                        currentOrientation * decimal(0.5) * dt;

                // Update the Transform of the body
                Transform newTransform(newPosition, newOrientation.getUnit());
                bodies[b]->setTransform(newTransform);
            }
        }
    }
}

// Update the AABBs of the bodies
void DynamicsWorld::updateRigidBodiesAABB() {

    PROFILE("DynamicsWorld::updateRigidBodiesAABB()");

    // For each rigid body of the world
    set<RigidBody*>::iterator it;
    for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        // If the body has moved
        if ((*it)->mHasMoved) {

            // Update the AABB of the rigid body
            (*it)->updateAABB();
        }
    }
}

// Compute and set the interpolation factor to all bodies
void DynamicsWorld::setInterpolationFactorToAllBodies() {

    PROFILE("DynamicsWorld::setInterpolationFactorToAllBodies()");
    
    // Compute the interpolation factor
    decimal factor = mTimer.computeInterpolationFactor();
    assert(factor >= 0.0 && factor <= 1.0);

    // Set the factor to all bodies
    set<RigidBody*>::iterator it;
    for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        (*it)->setInterpolationFactor(factor);
    }
}

// Initialize the bodies velocities arrays for the next simulation step.
void DynamicsWorld::initVelocityArrays() {

    // Allocate memory for the bodies velocity arrays
    uint nbBodies = mRigidBodies.size();
    if (mNbBodiesCapacity != nbBodies && nbBodies > 0) {
        if (mNbBodiesCapacity > 0) {
            delete[] mSplitLinearVelocities;
            delete[] mSplitAngularVelocities;
        }
        mNbBodiesCapacity = nbBodies;
        mSplitLinearVelocities = new Vector3[mNbBodiesCapacity];
        mSplitAngularVelocities = new Vector3[mNbBodiesCapacity];
        mConstrainedLinearVelocities = new Vector3[mNbBodiesCapacity];
        mConstrainedAngularVelocities = new Vector3[mNbBodiesCapacity];
        assert(mSplitLinearVelocities != NULL);
        assert(mSplitAngularVelocities != NULL);
        assert(mConstrainedLinearVelocities != NULL);
        assert(mConstrainedAngularVelocities != NULL);
    }

    // Reset the velocities arrays
    for (uint i=0; i<mNbBodiesCapacity; i++) {
        mSplitLinearVelocities[i].setToZero();
        mSplitAngularVelocities[i].setToZero();
    }

    // Initialize the map of body indexes in the velocity arrays
    mMapBodyToConstrainedVelocityIndex.clear();
    std::set<RigidBody*>::const_iterator it;
    uint indexBody = 0;
    for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        // Add the body into the map
        mMapBodyToConstrainedVelocityIndex.insert(std::make_pair<RigidBody*,
                                                  uint>(*it, indexBody));
        indexBody++;
    }
}

// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void DynamicsWorld::integrateRigidBodiesVelocities() {

    PROFILE("DynamicsWorld::integrateRigidBodiesVelocities()");

    // Initialize the bodies velocity arrays
    initVelocityArrays();

    decimal dt = static_cast<decimal>(mTimer.getTimeStep());

    // For each island of the world
    for (uint i=0; i < mNbIslands; i++) {

        RigidBody** bodies = mIslands[i]->getBodies();

        // For each body of the island
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Insert the body into the map of constrained velocities
            uint indexBody = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

            assert(mSplitLinearVelocities[indexBody] == Vector3(0, 0, 0));
            assert(mSplitAngularVelocities[indexBody] == Vector3(0, 0, 0));

            // If the body is allowed to move
            if (bodies[b]->isMotionEnabled()) {

                // Integrate the external force to get the new velocity of the body
                mConstrainedLinearVelocities[indexBody] = bodies[b]->getLinearVelocity() +
                        dt * bodies[b]->getMassInverse() * bodies[b]->mExternalForce;
                mConstrainedAngularVelocities[indexBody] = bodies[b]->getAngularVelocity() +
                        dt * bodies[b]->getInertiaTensorInverseWorld() *
                        bodies[b]->mExternalTorque;

                // If the gravity has to be applied to this rigid body
                if (bodies[b]->isGravityEnabled() && mIsGravityEnabled) {

                    // Integrate the gravity force
                    mConstrainedLinearVelocities[indexBody] += dt * bodies[b]->getMassInverse() *
                            bodies[b]->getMass() * mGravity;
                }

                // Apply the velocity damping
                // Damping force : F_c = -c' * v (c=damping factor)
                // Equation      : m * dv/dt = -c' * v
                //                 => dv/dt = -c * v (with c=c'/m)
                //                 => dv/dt + c * v = 0
                // Solution      : v(t) = v0 * e^(-c * t)
                //                 => v(t + dt) = v0 * e^(-c(t + dt))
                //                              = v0 * e^(-ct) * e^(-c * dt)
                //                              = v(t) * e^(-c * dt)
                //                 => v2 = v1 * e^(-c * dt)
                // Using Taylor Serie for e^(-x) : e^x ~ 1 + x + x^2/2! + ...
                //                              => e^(-x) ~ 1 - x
                //                 => v2 = v1 * (1 - c * dt)
                decimal linDampingFactor = bodies[b]->getLinearDamping();
                decimal angDampingFactor = bodies[b]->getAngularDamping();
                decimal linearDamping = clamp(decimal(1.0) - dt * linDampingFactor,
                                              decimal(0.0), decimal(1.0));
                decimal angularDamping = clamp(decimal(1.0) - dt * angDampingFactor,
                                               decimal(0.0), decimal(1.0));
                mConstrainedLinearVelocities[indexBody] *= clamp(linearDamping, decimal(0.0),
                                                                 decimal(1.0));
                mConstrainedAngularVelocities[indexBody] *= clamp(angularDamping, decimal(0.0),
                                                                  decimal(1.0));

                // Update the old Transform of the body
                bodies[b]->updateOldTransform();
            }

            indexBody++;
        }
    }
}

// Solve the contacts and constraints
void DynamicsWorld::solveContactsAndConstraints() {

    PROFILE("DynamicsWorld::solveContactsAndConstraints()");

    // Get the current time step
    decimal dt = static_cast<decimal>(mTimer.getTimeStep());

    // Set the velocities arrays
    mContactSolver.setSplitVelocitiesArrays(mSplitLinearVelocities, mSplitAngularVelocities);
    mContactSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                                                  mConstrainedAngularVelocities);
    mConstraintSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                                                     mConstrainedAngularVelocities);

    // ---------- Solve velocity constraints for joints and contacts ---------- //

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // Check if there are contacts and constraints to solve
        bool isConstraintsToSolve = mIslands[islandIndex]->getNbJoints() > 0;
        bool isContactsToSolve = mIslands[islandIndex]->getNbContactManifolds() > 0;
        if (!isConstraintsToSolve && !isContactsToSolve) continue;

        // If there are contacts in the current island
        if (isContactsToSolve) {

            // Initialize the solver
            mContactSolver.initializeForIsland(dt, mIslands[islandIndex]);

            // Warm start the contact solver
            mContactSolver.warmStart();
        }

        // If there are constraints
        if (isConstraintsToSolve) {

            // Initialize the constraint solver
            mConstraintSolver.initializeForIsland(dt, mIslands[islandIndex]);
        }

        // For each iteration of the velocity solver
        for (uint i=0; i<mNbVelocitySolverIterations; i++) {

            // Solve the constraints
            if (isConstraintsToSolve) {
                mConstraintSolver.solveVelocityConstraints(mIslands[islandIndex]);
            }

            // Solve the contacts
            if (isContactsToSolve) mContactSolver.solve();
        }        

        // Cache the lambda values in order to use them in the next
        // step and cleanup the contact solver
        if (isContactsToSolve) {
            mContactSolver.storeImpulses();
            mContactSolver.cleanup();
        }
    }
}

// Solve the position error correction of the constraints
void DynamicsWorld::solvePositionCorrection() {

    PROFILE("DynamicsWorld::solvePositionCorrection()");

    // Do not continue if there is no constraints
    if (mJoints.empty()) return;

    // ---------- Get the position/orientation of the rigid bodies ---------- //

    // TODO : Use better memory allocation here
    mConstrainedPositions = std::vector<Vector3>(mRigidBodies.size());
    mConstrainedOrientations = std::vector<Quaternion>(mRigidBodies.size());

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // For each body of the island
        RigidBody** bodies = mIslands[islandIndex]->getBodies();
        for (uint b=0; b < mIslands[islandIndex]->getNbBodies(); b++) {

            uint index = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

            // Get the position/orientation of the rigid body
            const Transform& transform = bodies[b]->getTransform();
            mConstrainedPositions[index] = transform.getPosition();
            mConstrainedOrientations[index]= transform.getOrientation();
        }

        // ---------- Solve the position error correction for the constraints ---------- //

        // For each iteration of the position (error correction) solver
        for (uint i=0; i<mNbPositionSolverIterations; i++) {

            // Solve the position constraints
            mConstraintSolver.solvePositionConstraints(mIslands[islandIndex]);
        }

        // ---------- Update the position/orientation of the rigid bodies ---------- //

        for (uint b=0; b < mIslands[islandIndex]->getNbBodies(); b++) {

            uint index = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

            // Get the new position/orientation of the body
            const Vector3& newPosition = mConstrainedPositions[index];
            const Quaternion& newOrientation = mConstrainedOrientations[index];

            // Update the Transform of the body
            Transform newTransform(newPosition, newOrientation.getUnit());
            bodies[b]->setTransform(newTransform);
        }
    }
}

// Create a rigid body into the physics world
RigidBody* DynamicsWorld::createRigidBody(const Transform& transform, decimal mass,
                                          const Matrix3x3& inertiaTensorLocal,
                                          const CollisionShape& collisionShape) {

    // Compute the body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create a collision shape for the rigid body into the world
    CollisionShape* newCollisionShape = createCollisionShape(collisionShape);

    // Create the rigid body
    RigidBody* rigidBody = new (mMemoryAllocator.allocate(sizeof(RigidBody))) RigidBody(transform,
                                                                                mass,
                                                                                inertiaTensorLocal,
                                                                                newCollisionShape,
                                                                                bodyID);
    assert(rigidBody != NULL);

    // Add the rigid body to the physics world
    mBodies.insert(rigidBody);
    mRigidBodies.insert(rigidBody);

    // Add the rigid body to the collision detection
    mCollisionDetection.addBody(rigidBody);

    // Return the pointer to the rigid body
    return rigidBody;
}

// Destroy a rigid body and all the joints which it belongs
void DynamicsWorld::destroyRigidBody(RigidBody* rigidBody) {

    // Remove the body from the collision detection
    mCollisionDetection.removeBody(rigidBody);

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(rigidBody->getID());

    // Remove the collision shape from the world
    removeCollisionShape(rigidBody->getCollisionShape());

    // Destroy all the joints in which the rigid body to be destroyed is involved
    JointListElement* element;
    for (element = rigidBody->mJointsList; element != NULL; element = element->next) {
        destroyJoint(element->joint);
    }

    // Reset the contact manifold list of the body
    rigidBody->resetContactManifoldsList(mMemoryAllocator);

    // Call the destructor of the rigid body
    rigidBody->RigidBody::~RigidBody();

    // Remove the rigid body from the list of rigid bodies
    mBodies.erase(rigidBody);
    mRigidBodies.erase(rigidBody);

    // Free the object from the memory allocator
    mMemoryAllocator.release(rigidBody, sizeof(RigidBody));
}

// Create a joint between two bodies in the world and return a pointer to the new joint
Joint* DynamicsWorld::createJoint(const JointInfo& jointInfo) {

    Joint* newJoint = NULL;

    // Allocate memory to create the new joint
    switch(jointInfo.type) {

        // Ball-and-Socket joint
        case BALLSOCKETJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(BallAndSocketJoint));
            const BallAndSocketJointInfo& info = dynamic_cast<const BallAndSocketJointInfo&>(
                                                                                        jointInfo);
            newJoint = new (allocatedMemory) BallAndSocketJoint(info);
            break;
        }

        // Slider joint
        case SLIDERJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(SliderJoint));
            const SliderJointInfo& info = dynamic_cast<const SliderJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) SliderJoint(info);
            break;
        }

        // Hinge joint
        case HINGEJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(HingeJoint));
            const HingeJointInfo& info = dynamic_cast<const HingeJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) HingeJoint(info);
            break;
        }

        // Fixed joint
        case FIXEDJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(FixedJoint));
            const FixedJointInfo& info = dynamic_cast<const FixedJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) FixedJoint(info);
            break;
        }

        default:
        {
            assert(false);
            return NULL;
        }
    }

    // If the collision between the two bodies of the constraint is disabled
    if (!jointInfo.isCollisionEnabled) {

        // Add the pair of bodies in the set of body pairs that cannot collide with each other
        mCollisionDetection.addNoCollisionPair(jointInfo.body1, jointInfo.body2);
    }

    // Add the joint into the world
    mJoints.insert(newJoint);

    // Add the joint into the joint list of the bodies involved in the joint
    addJointToBody(newJoint);

    // Return the pointer to the created joint
    return newJoint;
}

// Destroy a joint
void DynamicsWorld::destroyJoint(Joint* joint) {

    assert(joint != NULL);

    // If the collision between the two bodies of the constraint was disabled
    if (!joint->isCollisionEnabled()) {

        // Remove the pair of bodies from the set of body pairs that cannot collide with each other
        mCollisionDetection.removeNoCollisionPair(joint->getBody1(), joint->getBody2());
    }

    // Wake up the two bodies of the joint
    joint->getBody1()->setIsSleeping(false);
    joint->getBody2()->setIsSleeping(false);

    // Remove the joint from the world
    mJoints.erase(joint);

    // Remove the joint from the joint list of the bodies involved in the joint
    joint->mBody1->removeJointFromJointsList(mMemoryAllocator, joint);
    joint->mBody2->removeJointFromJointsList(mMemoryAllocator, joint);

    size_t nbBytes = joint->getSizeInBytes();

    // Call the destructor of the joint
    joint->Joint::~Joint();

    // Release the allocated memory
    mMemoryAllocator.release(joint, nbBytes);
}

// Add the joint to the list of joints of the two bodies involved in the joint
void DynamicsWorld::addJointToBody(Joint* joint) {

    assert(joint != NULL);

    // Add the joint at the beginning of the linked list of joints of the first body
    void* allocatedMemory1 = mMemoryAllocator.allocate(sizeof(JointListElement));
    JointListElement* jointListElement1 = new (allocatedMemory1) JointListElement(joint,
                                                                     joint->mBody1->mJointsList);
    joint->mBody1->mJointsList = jointListElement1;

    // Add the joint at the beginning of the linked list of joints of the second body
    void* allocatedMemory2 = mMemoryAllocator.allocate(sizeof(JointListElement));
    JointListElement* jointListElement2 = new (allocatedMemory2) JointListElement(joint,
                                                                     joint->mBody2->mJointsList);
    joint->mBody2->mJointsList = jointListElement2;
}

// Add a contact manifold to the linked list of contact manifolds of the two bodies involed
// in the corresponding contact
void DynamicsWorld::addContactManifoldToBody(ContactManifold* contactManifold,
                                             CollisionBody* body1, CollisionBody* body2) {

    assert(contactManifold != NULL);

    // Add the contact manifold at the beginning of the linked
    // list of contact manifolds of the first body
    void* allocatedMemory1 = mMemoryAllocator.allocate(sizeof(ContactManifoldListElement));
    ContactManifoldListElement* listElement1 = new (allocatedMemory1)
                                                  ContactManifoldListElement(contactManifold,
                                                                     body1->mContactManifoldsList);
    body1->mContactManifoldsList = listElement1;

    // Add the joint at the beginning of the linked list of joints of the second body
    void* allocatedMemory2 = mMemoryAllocator.allocate(sizeof(ContactManifoldListElement));
    ContactManifoldListElement* listElement2 = new (allocatedMemory2)
                                                  ContactManifoldListElement(contactManifold,
                                                                     body2->mContactManifoldsList);
    body2->mContactManifoldsList = listElement2;
}

// Reset all the contact manifolds linked list of each body
void DynamicsWorld::resetContactManifoldListsOfBodies() {

    // For each rigid body of the world
    for (std::set<RigidBody*>::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        // Reset the contact manifold list of the body
        (*it)->resetContactManifoldsList(mMemoryAllocator);
    }
}

// Compute the islands of awake bodies.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void DynamicsWorld::computeIslands() {

    PROFILE("DynamicsWorld::computeIslands()");

    uint nbBodies = mRigidBodies.size();

    // Clear all the islands
    for (uint i=0; i<mNbIslands; i++) {

        // Call the island destructor
        mIslands[i]->Island::~Island();

        // Release the allocated memory for the island
        mMemoryAllocator.release(mIslands[i], sizeof(Island));
    }

    // Allocate and create the array of islands
    if (mNbIslandsCapacity != nbBodies && nbBodies > 0) {
        if (mNbIslandsCapacity > 0) {
            mMemoryAllocator.release(mIslands, sizeof(Island*) * mNbIslandsCapacity);
        }
        mNbIslandsCapacity = nbBodies;
        mIslands = (Island**)mMemoryAllocator.allocate(sizeof(Island*) * mNbIslandsCapacity);
    }
    mNbIslands = 0;

    // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
    for (std::set<RigidBody*>::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
        (*it)->mIsAlreadyInIsland = false;
    }
    for (std::vector<ContactManifold*>::iterator it = mContactManifolds.begin();
         it != mContactManifolds.end(); ++it) {
        (*it)->mIsAlreadyInIsland = false;
    }
    for (std::set<Joint*>::iterator it = mJoints.begin(); it != mJoints.end(); ++it) {
        (*it)->mIsAlreadyInIsland = false;
    }

    // Create a stack (using an array) for the rigid bodies to visit during the Depth First Search
    size_t nbBytesStack = sizeof(RigidBody*) * nbBodies;
    RigidBody** stackBodiesToVisit = (RigidBody**)mMemoryAllocator.allocate(nbBytesStack);

    // For each rigid body of the world
    for (std::set<RigidBody*>::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        RigidBody* body = *it;

        // If the body has already been added to an island, we go to the next body
        if (body->mIsAlreadyInIsland) continue;

        // If the body is not moving, we go to the next body
        // TODO : When we will use STATIC bodies, we will need to take care of this case here
        if (!body->isMotionEnabled()) continue;

        // If the body is sleeping or inactive, we go to the next body
        if (body->isSleeping() || !body->isActive()) continue;

        // Reset the stack of bodies to visit
        uint stackIndex = 0;
        stackBodiesToVisit[stackIndex] = body;
        stackIndex++;
        body->mIsAlreadyInIsland = true;

        // Create the new island
        void* allocatedMemoryIsland = mMemoryAllocator.allocate(sizeof(Island));
        mIslands[mNbIslands] = new (allocatedMemoryIsland) Island(nbBodies,mContactManifolds.size(),
                                                                  mJoints.size(), mMemoryAllocator);

        // While there are still some bodies to visit in the stack
        while (stackIndex > 0) {

            // Get the next body to visit from the stack
            stackIndex--;
            RigidBody* bodyToVisit = stackBodiesToVisit[stackIndex];
            assert(bodyToVisit->isActive());

            // Awake the body if it is slepping
            bodyToVisit->setIsSleeping(false);

            // Add the body into the island
            mIslands[mNbIslands]->addBody(bodyToVisit);

            // If the current body is not moving, we do not want to perform the DFS
            // search across that body
            if (!bodyToVisit->isMotionEnabled()) continue;

            // For each contact manifold in which the current body is involded
            ContactManifoldListElement* contactElement;
            for (contactElement = bodyToVisit->mContactManifoldsList; contactElement != NULL;
                 contactElement = contactElement->next) {

                ContactManifold* contactManifold = contactElement->contactManifold;

                // Check if the current contact manifold has already been added into an island
                if (contactManifold->isAlreadyInIsland()) continue;

                // Add the contact manifold into the island
                mIslands[mNbIslands]->addContactManifold(contactManifold);
                contactManifold->mIsAlreadyInIsland = true;

                // Get the other body of the contact manifold
                RigidBody* body1 = dynamic_cast<RigidBody*>(contactManifold->getBody1());
                RigidBody* body2 = dynamic_cast<RigidBody*>(contactManifold->getBody2());
                RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }

            // For each joint in which the current body is involved
            JointListElement* jointElement;
            for (jointElement = bodyToVisit->mJointsList; jointElement != NULL;
                 jointElement = jointElement->next) {

                Joint* joint = jointElement->joint;

                // Check if the current joint has already been added into an island
                if (joint->isAlreadyInIsland()) continue;

                // Add the joint into the island
                mIslands[mNbIslands]->addJoint(joint);
                joint->mIsAlreadyInIsland = true;

                // Get the other body of the contact manifold
                RigidBody* body1 = dynamic_cast<RigidBody*>(joint->getBody1());
                RigidBody* body2 = dynamic_cast<RigidBody*>(joint->getBody2());
                RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }
        }

        // Reset the isAlreadyIsland variable of the static bodies so that they
        // can also be included in the other islands
        for (uint i=0; i < mIslands[mNbIslands]->mNbBodies; i++) {

            if (!mIslands[mNbIslands]->mBodies[i]->isMotionEnabled()) {
                mIslands[mNbIslands]->mBodies[i]->mIsAlreadyInIsland = false;
            }
        }

        mNbIslands++;
     }

    // Release the allocated memory for the stack of bodies to visit
    mMemoryAllocator.release(stackBodiesToVisit, nbBytesStack);
}

// Put bodies to sleep if needed.
/// For each island, if all the bodies have been almost still for a long enough period of
/// time, we put all the bodies of the island to sleep.
void DynamicsWorld::updateSleepingBodies() {

    PROFILE("DynamicsWorld::updateSleepingBodies()");

    const decimal dt = static_cast<decimal>(mTimer.getTimeStep());
    const decimal sleepLinearVelocitySquare = mSleepLinearVelocity * mSleepLinearVelocity;
    const decimal sleepAngularVelocitySquare = mSleepAngularVelocity * mSleepAngularVelocity;

    // For each island of the world
    for (uint i=0; i<mNbIslands; i++) {

        decimal minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        RigidBody** bodies = mIslands[i]->getBodies();
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Skip static bodies
            if (!bodies[b]->isMotionEnabled()) continue;

            // If the body is velocity is large enough to stay awake
            if (bodies[b]->getLinearVelocity().lengthSquare() > sleepLinearVelocitySquare ||
                bodies[b]->getAngularVelocity().lengthSquare() > sleepAngularVelocitySquare ||
                !bodies[b]->isAllowedToSleep()) {

                // Reset the sleep time of the body
                bodies[b]->mSleepTime = decimal(0.0);
                minSleepTime = decimal(0.0);
            }
            else {  // If the body velocity is bellow the sleeping velocity threshold

                // Increase the sleep time
                bodies[b]->mSleepTime += dt;
                if (bodies[b]->mSleepTime < minSleepTime) {
                    minSleepTime = bodies[b]->mSleepTime;
                }
            }
        }

        // If the velocity of all the bodies of the island is under the
        // sleeping velocity threshold for a period of time larger than
        // the time required to become a sleeping body
        if (minSleepTime >= mTimeBeforeSleep) {

            // Put all the bodies of the island to sleep
            for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {
                bodies[b]->setIsSleeping(true);
            }
        }
    }
}

// Notify the world about a new broad-phase overlapping pair
void DynamicsWorld::notifyAddedOverlappingPair(const BroadPhasePair* addedPair) {

    // Get the pair of body index
    bodyindexpair indexPair = addedPair->getBodiesIndexPair();

    // Add the pair into the set of overlapping pairs (if not there yet)
    OverlappingPair* newPair = new (mMemoryAllocator.allocate(sizeof(OverlappingPair)))
                              OverlappingPair(addedPair->body1, addedPair->body2, mMemoryAllocator);
    assert(newPair != NULL);
    std::pair<map<bodyindexpair, OverlappingPair*>::iterator, bool> check =
            mOverlappingPairs.insert(make_pair(indexPair, newPair));
    assert(check.second);
}

// Notify the world about a removed broad-phase overlapping pair
void DynamicsWorld::notifyRemovedOverlappingPair(const BroadPhasePair* removedPair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = removedPair->getBodiesIndexPair();

    // Remove the overlapping pair from the memory allocator
    mOverlappingPairs.find(indexPair)->second->OverlappingPair::~OverlappingPair();
    mMemoryAllocator.release(mOverlappingPairs[indexPair], sizeof(OverlappingPair));
    mOverlappingPairs.erase(indexPair);
}

// Notify the world about a new narrow-phase contact
void DynamicsWorld::notifyNewContact(const BroadPhasePair* broadPhasePair,
                                     const ContactPointInfo* contactInfo) {

    // Create a new contact
    ContactPoint* contact = new (mMemoryAllocator.allocate(sizeof(ContactPoint))) ContactPoint(
                                                                                    *contactInfo);
    assert(contact != NULL);

    // Get the corresponding overlapping pair
    pair<bodyindex, bodyindex> indexPair = broadPhasePair->getBodiesIndexPair();
    OverlappingPair* overlappingPair = mOverlappingPairs.find(indexPair)->second;
    assert(overlappingPair != NULL);

    // If it is the first contact since the pair are overlapping
    if (overlappingPair->getNbContactPoints() == 0) {

        // Trigger a callback event
        if (mEventListener != NULL) mEventListener->beginContact(*contactInfo);
    }

    // Add the contact to the contact cache of the corresponding overlapping pair
    overlappingPair->addContact(contact);

    // Add the contact manifold to the world
    mContactManifolds.push_back(overlappingPair->getContactManifold());

    // Add the contact manifold into the list of contact manifolds
    // of the two bodies involved in the contact
    addContactManifoldToBody(overlappingPair->getContactManifold(), overlappingPair->mBody1,
                             overlappingPair->mBody2);

    // Trigger a callback event for the new contact
    if (mEventListener != NULL) mEventListener->newContact(*contactInfo);
}

// Enable/Disable the sleeping technique
void DynamicsWorld::enableSleeping(bool isSleepingEnabled) {
    mIsSleepingEnabled = isSleepingEnabled;

    if (!mIsSleepingEnabled) {

        // For each body of the world
        std::set<RigidBody*>::iterator it;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            // Wake up the rigid body
            (*it)->setIsSleeping(false);
        }
    }
}