/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H
#define REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H

// Libraries
#include <reactphysics3d/collision/shapes/ConcaveShape.h>
#include <reactphysics3d/collision/shapes/AABB.h>

namespace reactphysics3d {

class HeightFieldShape;
class Profiler;
class TriangleShape;

// Class HeightFieldShape
/**
 * This class represents a static height field that can be used to represent
 * a terrain. The height field is made of a grid with rows and columns with a
 * height value at each grid point. Note that the height values are not copied into the shape
 * but are shared instead. The height values can be of type integer, float or double.
 * When creating a HeightFieldShape, you need to specify the minimum and maximum height value of
 * your height field. Note that the HeightFieldShape will be re-centered based on its AABB. It means
 * that for instance, if the minimum height value is -200 and the maximum value is 400, the final
 * minimum height of the field in the simulation will be -300 and the maximum height will be 300.
 */
class HeightFieldShape : public ConcaveShape {

    public:

        /// Data type for the height data of the height field
        enum class HeightDataType {HEIGHT_FLOAT_TYPE, HEIGHT_DOUBLE_TYPE, HEIGHT_INT_TYPE};

    protected:

        // -------------------- Attributes -------------------- //

        /// Number of columns in the grid of the height field
        int mNbColumns;

        /// Number of rows in the grid of the height field
        int mNbRows;

        /// Height field width
        decimal mWidth;

        /// Height field length
        decimal mLength;

        /// Minimum height of the height field
        decimal mMinHeight;

        /// Maximum height of the height field
        decimal mMaxHeight;

        /// Up axis direction (0 => x, 1 => y, 2 => z)
        int mUpAxis;

        /// Height values scale for height field with integer height values
        decimal mIntegerHeightScale;

        /// Data type of the height values
        HeightDataType mHeightDataType;

        /// Array of data with all the height values of the height field
        void* mHeightFieldData;

        /// Local AABB of the height field (without scaling)
        AABB mAABB;

        /// Reference to the half-edge structure
        HalfEdgeStructure& mTriangleHalfEdgeStructure;

        // -------------------- Methods -------------------- //

        /// Constructor
        HeightFieldShape(int nbGridColumns, int nbGridRows, decimal minHeight, decimal maxHeight,
                         void* heightFieldData, HeightDataType dataType, MemoryAllocator& allocator,
                         HalfEdgeStructure& triangleHalfEdgeStructure, int upAxis = 1, decimal integerHeightScale = 1.0f,
                         const Vector3& scaling = Vector3(1,1,1));

        /// Raycast a single triangle of the height-field
        bool raycastTriangle(const Ray& ray, const Vector3& p1, const Vector3& p2, const Vector3& p3, uint32 shapeId,
                             Collider *collider, RaycastInfo& raycastInfo, decimal &smallestHitFraction, MemoryAllocator& allocator) const;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
        /// given the start vertex index pointer of the triangle.
        void getTriangleVerticesWithIndexPointer(int32 subPart, int32 triangleIndex,
                                                 Vector3* outTriangleVertices) const;

        /// Compute the min/max grid coords corresponding to the intersection of the AABB of the height field and the AABB to collide
        void computeMinMaxGridCoordinates(int* minCoords, int* maxCoords, const AABB& aabbToCollide) const;

        /// Compute the shape Id for a given triangle
        uint32 computeTriangleShapeId(uint32 iIndex, uint32 jIndex, uint32 secondTriangleIncrement) const;

        /// Compute the first grid cell of the heightfield intersected by a ray
        bool computeEnteringRayGridCoordinates(const Ray& ray, int& i, int& j, Vector3& outHitPoint) const;
        
        /// Destructor
        virtual ~HeightFieldShape() override = default;

    public:

        /// Deleted copy-constructor
        HeightFieldShape(const HeightFieldShape& shape) = delete;

        /// Deleted assignment operator
        HeightFieldShape& operator=(const HeightFieldShape& shape) = delete;

        /// Return the number of rows in the height field
        int getNbRows() const;

        /// Set the number of rows in the height field
        void setNbRows(const int nbGridRows);

        /// Return the number of columns in the height field
        int getNbColumns() const;

        /// Set the number of columns in the height field
        void setNbColumns(const int nbGridRows);

        /// Return the minimum height value of the height field
        decimal getMinHeight() const;

        /// Set the minimum height value of the height field
        void setMinHeight(const decimal minHeight);

        /// Return the maximum height value of the height field
        decimal getMaxHeight() const;

        /// Set the maximum height value of the height field
        void setMaxHeight(const decimal maxHeight);

        /// Set the height range of the height field
        void setMinMaxHeight(const decimal minHeight, const decimal maxHeight);

        /// Set the height field data
        void setHeightFieldData(void* heightFieldData);

        /// Return the vertex (local-coordinates) of the height field at a given (x,y) position
        Vector3 getVertexAt(int x, int y) const;

        /// Return the height of a given (x,y) point in the height field
        decimal getHeightAt(int x, int y) const;

        /// Return the type of height value in the height field
        HeightDataType getHeightDataType() const;

        /// Compute the local AABB of the height field
        void computeLocalAABB();

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const override;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void computeOverlappingTriangles(const AABB& localAABB, Array<Vector3>& triangleVertices,
                                                   Array<Vector3>& triangleVerticesNormals, Array<uint32>& shapeIds,
                                                   MemoryAllocator& allocator) const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
        friend class PhysicsCommon;
};

// Return the number of rows in the height field
RP3D_FORCE_INLINE int HeightFieldShape::getNbRows() const {
    return mNbRows;
}

// Set the number of rows in the height field
RP3D_FORCE_INLINE void HeightFieldShape::setNbRows(const int nbRows) {
    assert(nbRows >= 2);

    mNbRows = nbRows;
    mLength = static_cast<decimal>(mNbRows - 1);

    computeLocalAABB();
    notifyColliderAboutChangedSize();
}

// Return the number of columns in the height field
RP3D_FORCE_INLINE int HeightFieldShape::getNbColumns() const {
    return mNbColumns;
}

// Set the number of columns in the height field
RP3D_FORCE_INLINE void HeightFieldShape::setNbColumns(const int nbColumns) {
    assert(nbColumns >= 2);

    mNbColumns = nbColumns;
    mWidth = static_cast<decimal>(mNbColumns - 1);

    notifyColliderAboutChangedSize();
}

/// Return the minimum height value of the height field
RP3D_FORCE_INLINE decimal HeightFieldShape::getMinHeight() const {
    return mMinHeight;
}

/// Set the minimum height value of the height field
RP3D_FORCE_INLINE void HeightFieldShape::setMinHeight(const decimal minHeight) {
    assert(minHeight <= mMaxHeight);
    mMinHeight = minHeight;

    notifyColliderAboutChangedSize();
}

/// Return the maximum height value of the height field
RP3D_FORCE_INLINE decimal HeightFieldShape::getMaxHeight() const {
    return mMaxHeight;
}

/// Set the maximum height value of the height field
RP3D_FORCE_INLINE void HeightFieldShape::setMaxHeight(const decimal maxHeight) {
    assert(mMinHeight <= maxHeight);
    mMaxHeight = maxHeight;

    notifyColliderAboutChangedSize();
}

/// Set the height range of the height field
RP3D_FORCE_INLINE void HeightFieldShape::setMinMaxHeight(const decimal minHeight, const decimal maxHeight) {
    assert(minHeight <= maxHeight);
    mMinHeight = minHeight;
    mMaxHeight = maxHeight;

    notifyColliderAboutChangedSize();
}

/// Set the height field data
RP3D_FORCE_INLINE void HeightFieldShape::setHeightFieldData(void* heightFieldData) {
    assert(heightFieldData != nullptr);
    mHeightFieldData = heightFieldData;

    notifyColliderAboutChangedSize();
}

// Return the type of height value in the height field
RP3D_FORCE_INLINE HeightFieldShape::HeightDataType HeightFieldShape::getHeightDataType() const {
    return mHeightDataType;
}

// Return the number of bytes used by the collision shape
RP3D_FORCE_INLINE size_t HeightFieldShape::getSizeInBytes() const {
    return sizeof(HeightFieldShape);
}

// Return the height of a given (x,y) point in the height field
RP3D_FORCE_INLINE decimal HeightFieldShape::getHeightAt(int x, int y) const {

    assert(x >= 0 && x < mNbColumns);
    assert(y >= 0 && y < mNbRows);

    switch(mHeightDataType) {
        case HeightDataType::HEIGHT_FLOAT_TYPE : return decimal(((float*)mHeightFieldData)[y * mNbColumns + x]);
        case HeightDataType::HEIGHT_DOUBLE_TYPE : return decimal(((double*)mHeightFieldData)[y * mNbColumns + x]);
        case HeightDataType::HEIGHT_INT_TYPE : return decimal(((int*)mHeightFieldData)[y * mNbColumns + x] * mIntegerHeightScale);
        default: assert(false); return 0;
    }
}

// Compute the shape Id for a given triangle
RP3D_FORCE_INLINE uint32 HeightFieldShape::computeTriangleShapeId(uint32 iIndex, uint32 jIndex, uint32 secondTriangleIncrement) const {

    return (jIndex * (mNbColumns - 1) + iIndex) * 2 + secondTriangleIncrement;
}

}
#endif

