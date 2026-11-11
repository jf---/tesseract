/**
 * @file narrowphase_dispatch_count.h
 * @brief Classify one Bullet child-pair narrowphase dispatch into the shared NarrowphaseStats counters
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COLLISION_BULLET_NARROWPHASE_DISPATCH_COUNT_H
#define TESSERACT_COLLISION_BULLET_NARROWPHASE_DISPATCH_COUNT_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/narrowphase_stats.h>

namespace tesseract::collision::bullet_internal
{
/**
 * @brief Count one child-pair dispatch by the shapes it is called on.
 *
 * A mesh's children are triangle shapes (TRIANGLE_SHAPE_PROXYTYPE). Exactly one triangle against a convex
 * non-triangle shape (a primitive, a convex hull, or a CastHullShape) is a leaf test. Two shapes of which neither is
 * a triangle nor a compound is a primitive test. Anything else (a compound child, two triangles, a triangle against
 * a non-convex shape) counts nothing here; a compound child is counted by its own dispatch.
 */
inline void countChildDispatch(const btCollisionShape* shape0, const btCollisionShape* shape1)
{
  const bool triangle0 = shape0->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE;
  const bool triangle1 = shape1->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE;
  if (triangle0 != triangle1)
  {
    if ((triangle0 ? shape1 : shape0)->isConvex())
      countLeafTest();
    return;
  }
  if (!triangle0 && !shape0->isCompound() && !shape1->isCompound())
    countPrimitiveTest();
}
}  // namespace tesseract::collision::bullet_internal

#endif  // TESSERACT_COLLISION_BULLET_NARROWPHASE_DISPATCH_COUNT_H
