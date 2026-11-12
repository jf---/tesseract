/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008

Altered source, vendored from Bullet 3.25 (github.com/bulletphysics/bullet3, tag 3.25):
  src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp (GJK, EPA, Distance, Penetration)
  src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp (calcPenDepth)
The algorithm and its constants are unchanged. The alteration: each witness also reports, for a CastHullShape, the
weights it puts on support points of the cast's start and end poses, which btGjkEpaSolver2 does not expose. Used for
cast pairs only; every other pair still runs the configured Bullet solver.
*/
#ifndef TESSERACT_COLLISION_TESSERACT_GJK_EPA_H
#define TESSERACT_COLLISION_TESSERACT_GJK_EPA_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>

namespace tesseract::collision::bullet_internal
{
/**
 * @brief Where each witness lies between the poses of a cast, per shape of the query
 *
 * A witness is a convex combination sum_i p_i v_i of support points of its shape (sum_i p_i = 1). For a
 * CastHullShape each v_i is a CastSupportVertex, and shape[k] holds the sums over them (see CastSupportSum). It is
 * zero for a shape that is not a CastHullShape.
 */
struct CastSupportWeights
{
  CastSupportSum shape[2];  // NOLINT(modernize-avoid-c-arrays)
};

/**
 * @brief btGjkEpaPenetrationDepthSolver::calcPenDepth, also reporting the end weights of the witnesses it returns
 *
 * Runs Bullet's GJK and EPA (btGjkEpaSolver2, unchanged) over the same guess directions. When it returns a
 * penetration the weights come from the final EPA face, when it returns a distance from the final GJK simplex.
 * @return True for a penetration, as calcPenDepth
 */
bool castGjkEpaCalcPenDepth(const btConvexShape* pConvexA,
                            const btConvexShape* pConvexB,
                            const btTransform& transformA,
                            const btTransform& transformB,
                            btVector3& v,
                            btVector3& wWitnessOnA,
                            btVector3& wWitnessOnB,
                            CastSupportWeights& weights);
}  // namespace tesseract::collision::bullet_internal

#endif  // TESSERACT_COLLISION_TESSERACT_GJK_EPA_H
