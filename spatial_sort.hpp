
// ================================================================================================
// -*- C++ -*-
// File: spatial_sort.hpp
// Author: Guilherme R. Lampert
// Created on: 07/08/16
// Brief: Spatial sorting for mesh vertexes. Allow quick search of neighboring vertexes.
// ================================================================================================

/*
Code in this file is largely based on SpatialSort.h found in the ASSIMP library.
<http://www.assimp.org/>

Original copyright notice:

Open Asset Import Library (ASSIMP)
----------------------------------------------------------------------

Copyright (c) 2006-2010, ASSIMP Development Team
All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the ASSIMP team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the ASSIMP Development Team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

#ifndef SPATIAL_SORT_HPP
#define SPATIAL_SORT_HPP

#include <climits>
#include <cstdint>
#include <cmath>

#include <vector>
#include <algorithm>

//
// 2 dimensional and 3 dimensional vectors or points:
//
struct Vec2
{
    float x,y;
};
struct Vec3
{
    float x,y,z;
};

// ========================================================
// class SpatialSort:
// ========================================================

//
// A little helper class to quickly find all vertexes in the epsilon environment of a given
// position. Construct an instance with an array of positions. The class stores the given positions
// by their indexes and sorts them by their distance to an arbitrary chosen plane.
// You can then query the instance for all vertexes close to a given position in an average O(log n)
// time, with O(n) worst case complexity when all vertexes lay on the plane. The plane is chosen
// so that it avoids common planes in usual data sets.
//
class SpatialSort final
{
public:

    SpatialSort()
    {
        // Define the reference plane. We choose some arbitrary vector away from all basic axes
        // in the hope that no model spreads all its vertexes along this plane.
        planeNormal.x = 0.85230f;
        planeNormal.y = 0.34321f;
        planeNormal.z = 0.57360f;

        const float len = 1.0f / std::sqrt((planeNormal.x * planeNormal.x) +
                                           (planeNormal.y * planeNormal.y) +
                                           (planeNormal.z * planeNormal.z));
        planeNormal.x *= len;
        planeNormal.y *= len;
        planeNormal.z *= len;
    }

    // Constructs a spatially sorted representation from the given position array.
    SpatialSort(const Vec3 * const positionList, const std::size_t numPositions)
        : SpatialSort()
    {
        fill(positionList, numPositions, true);
    }

    // Sets the input data for the SpatialSort. This replaces existing data, if any.
    // Finalization is required in order to use findPosition() or generateMappingTable().
    // If you haven't finalized yet, you can use append() to add data from other sources.
    void fill(const Vec3 * const positionList, const std::size_t numPositions, const bool doFinalize)
    {
        positions.clear();
        append(positionList, numPositions, doFinalize);
    }

    // Same as fill() but expects a structured vertex type with a 'position' member.
    template<typename VertexType>
    void fillStructured(const VertexType * const vertsList, const std::size_t vertCount, const bool doFinalize)
    {
        positions.clear();
        positions.reserve(doFinalize ? vertCount : (vertCount * 2));

        for (std::uint32_t i = 0; i < vertCount; ++i)
        {
            // Store position by index and distance:
            const float distance = dotProduct(vertsList[i].position, planeNormal);
            positions.emplace_back(vertsList[i].position, distance, i);
        }

        if (doFinalize)
        {
            finalize();
        }
    }

    // Same as fill(), except the method appends to existing data in the SpatialSort.
    // Requires finalization.
    void append(const Vec3 * const positionList, const std::size_t numPositions, const bool doFinalize)
    {
        const std::size_t initial = positions.size();
        positions.reserve(initial + (doFinalize ? numPositions : (numPositions * 2)));

        for (std::size_t i = 0; i < numPositions; ++i)
        {
            // Store position by index and distance:
            const float distance = dotProduct(positionList[i], planeNormal);
            positions.emplace_back(positionList[i], distance, static_cast<std::uint32_t>(i + initial));
        }

        if (doFinalize)
        {
            finalize();
        }
    }

    // Finalize the spatial sorting data structure. This can be useful after
    // multiple calls to append() with the 'finalize' parameter set to false.
    // This is required before one of findPositions() and generateMappingTable()
    // can be called to query the spatial sort.
    void finalize()
    {
        std::sort(positions.begin(), positions.end());
    }

    // Returns a list for all positions close to the given position.
    void findPositions(const Vec3 & position, const float radius, std::vector<std::uint32_t> * outResults) const
    {
        const float dist    = dotProduct(position, planeNormal);
        const float minDist = (dist - radius);
        const float maxDist = (dist + radius);

        outResults->clear();

        // Quick check for positions outside the range:
        if (positions.empty())
        {
            return;
        }
        if (maxDist < positions.front().distance)
        {
            return;
        }
        if (minDist > positions.back().distance)
        {
            return;
        }

        // Do a binary search for the minimal distance to start the iteration there:
        std::uint32_t index          = static_cast<std::uint32_t>(positions.size() / 2);
        std::uint32_t binaryStepSize = static_cast<std::uint32_t>(positions.size() / 4);

        while (binaryStepSize > 1)
        {
            if (positions[index].distance < minDist)
            {
                index += binaryStepSize;
            }
            else
            {
                index -= binaryStepSize;
            }
            binaryStepSize /= 2;
        }

        // Depending on the direction of the last step we need to single step a bit
        // back or forth to find the actual beginning element of the range.
        while ((index > 0) && (positions[index].distance > minDist))
        {
            --index;
        }
        while ((index < (positions.size() - 1)) && (positions[index].distance < minDist))
        {
            ++index;
        }

        // Now start iterating from there until the first position lays outside of the distance range.
        // Add all positions inside the distance range within the given radius to the result array.
        EntryArray::const_iterator it = (positions.begin() + index);
        const float squaredRadius = radius * radius;

        while (it->distance < maxDist)
        {
            Vec3 diff;
            diff.x = it->position.x - position.x;
            diff.y = it->position.y - position.y;
            diff.z = it->position.z - position.z;

            const float lengthSqr = dotProduct(diff, diff);
            if (lengthSqr < squaredRadius)
            {
                outResults->push_back(it->index);
            }

            ++it;
            if (it == positions.end())
            {
                break;
            }
        }
    }

    // Fills an array with indexes of all positions identical to the given position.
    // In opposite to findPositions(), it is not an epsilon that is used but a (very low)
    // tolerance of four floating-point units.
    void findIdenticalPositions(const Vec3 & position, std::vector<std::uint32_t> * outResults) const
    {
        // Epsilons have a huge disadvantage: they are of constant precision, while floating-point
        // values are of log2 precision. If you apply e=0.01 to 100, the epsilon is rather small,
        // but if you apply it to 0.001, it is enormous.
        //
        // The best way to overcome this is the unit in the last place (ULP). A precision of 2 ULPs
        // tells us that a float does not differ more than 2 bits from the "real" value. ULPs are of
        // logarithmic precision - around 1, they are 1÷(2^24) and around 10000, they are 0.00125.
        //
        // For standard C math, we can assume a precision of 0.5 ULPs according to IEEE 754.
        // The incoming vertex positions might have already been transformed, probably using rather
        // inaccurate SSE instructions, so we assume a tolerance of 4 ULPs to safely identify
        // identical vertex positions.
        static const int toleranceInULPs = 4;

        // An interesting point is that the inaccuracy grows linear with the number of operations:
        // multiplying two numbers, each inaccurate to four ULPs, results in an inaccuracy of four ULPs
        // plus 0.5 ULPs for the multiplication.
        // To compute the distance to the plane, a dot product is needed - that is a multiplication and
        // an addition on each number.
        static const int distanceToleranceInULPs = toleranceInULPs + 1;

        // The squared distance between two 3D vectors is computed the same way, but with an additional subtraction.
        static const int distance3DToleranceInULPs = distanceToleranceInULPs + 1;

        // Convert the plane distance to its signed integer representation so the ULPs tolerance can be applied.
        const BinFloat minDistBinary = floatToBinary(dotProduct(position, planeNormal)) - distanceToleranceInULPs;
        const BinFloat maxDistBinary = minDistBinary + 2 * distanceToleranceInULPs;

        outResults->clear();

        // Do a binary search for the minimal distance to start the iteration there:
        std::uint32_t index          = static_cast<std::uint32_t>(positions.size() / 2);
        std::uint32_t binaryStepSize = static_cast<std::uint32_t>(positions.size() / 4);

        while (binaryStepSize > 1)
        {
            if (minDistBinary > floatToBinary(positions[index].distance))
            {
                index += binaryStepSize;
            }
            else
            {
                index -= binaryStepSize;
            }
            binaryStepSize /= 2;
        }

        // Depending on the direction of the last step we need to single step a bit back or forth
        // to find the actual beginning element of the range.
        while ((index > 0) && (minDistBinary < floatToBinary(positions[index].distance)))
        {
            --index;
        }
        while ((index < (positions.size() - 1)) && (minDistBinary > floatToBinary(positions[index].distance)))
        {
            ++index;
        }

        // Now start iterating from there until the first position lays outside of the distance range.
        // Add all positions inside the distance range within the tolerance to the result array.
        EntryArray::const_iterator it = (positions.begin() + index);

        while (floatToBinary(it->distance) < maxDistBinary)
        {
            Vec3 diff;
            diff.x = it->position.x - position.x;
            diff.y = it->position.y - position.y;
            diff.z = it->position.z - position.z;

            const float lengthSqr = dotProduct(diff, diff);
            if (distance3DToleranceInULPs >= floatToBinary(lengthSqr))
            {
                outResults->push_back(it->index);
            }

            ++it;
            if (it == positions.end())
            {
                break;
            }
        }
    }

private:

    // Normal of the sorting plane. The center is always at (0, 0, 0).
    Vec3 planeNormal;

    static float dotProduct(const Vec3 & a, const Vec3 & b)
    {
        return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
    }

    // An entry in a spatially sorted position array. Consists of a vertex index,
    // its position and its pre-calculated distance from the reference plane.
    struct Entry
    {
        Vec3          position;
        float         distance;
        std::uint32_t index;

        Entry() = default;
        Entry(const Vec3 & pos, const float dist, const std::uint32_t idx)
            : position{ pos }, distance{ dist }, index{ idx }
        { }

        bool operator < (const Entry & e) const { return distance < e.distance; }
    };

    // All positions, sorted by distance to the sorting plane:
    using EntryArray = std::vector<Entry>;
    EntryArray positions;

    // Binary, signed-integer representation of a single-precision floating-point value.
    // IEEE 754 says: "If two floating-point numbers in the same format are ordered then they are
    // ordered the same way when their bits are reinterpreted as sign-magnitude integers."
    // This allows us to convert all floating-point numbers to signed integers of arbitrary size
    // and then use them to work with ULPs (Units in the Last Place, for high-precision
    // computations) or to compare them (integer comparisons are still faster than floating-point
    // comparisons on most platforms).
    using BinFloat = signed int;

    static BinFloat floatToBinary(const float & value)
    {
        // If this assertion fails, signed int is not big enough to store a
        // float on your platform. Correct the declaration of BinFloat as needed.
        static_assert(sizeof(BinFloat) >= sizeof(float), "Redefine 'BinFloat' to something >= sizeof(float)!");

        // A union should avoid strict-aliasing issues.
        union
        {
            float asFloat;
            BinFloat asBin;
        } conversion;

        conversion.asBin        = 0; // zero empty space in case sizeof(BinFloat) > sizeof(float)
        conversion.asFloat      = value;
        const BinFloat binValue = conversion.asBin;

        // Floating-point numbers are of sign-magnitude format, so find out what
        // signed number representation we must convert negative values to.
        // See: <http://en.wikipedia.org/wiki/Signed_number_representations>

        // Two's complement?
        if ((-42 == (~42 + 1)) && (binValue & 0x80000000))
        {
            return BinFloat(1 << (CHAR_BIT * sizeof(BinFloat) - 1)) - binValue;
        }

        // One's complement?
        if ((-42 == ~42) && (binValue & 0x80000000))
        {
            return BinFloat(-0) - binValue;
        }

        // Sign-magnitude?
        if ((-42 == (42 | (-0))) && (binValue & 0x80000000)) // -0 = 1000... Binary
        {
            return binValue;
        }

        return binValue;
    }
};

#endif // SPATIAL_SORT_HPP
