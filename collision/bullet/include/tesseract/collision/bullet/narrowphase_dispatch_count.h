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
#include <cassert>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/narrowphase_stats.h>
#include <tesseract/collision/bullet/bullet_utils.h>

/**
 * Bullet's counted sites. leaf_tests and primitive_tests are counted at the two compound child dispatches: the
 * child-pair dispatch of the compound-compound algorithm and the one-sided dispatch of the compound algorithm. Two
 * single-shape objects (a bare primitive against a bare primitive) are dispatched by Bullet directly and are not
 * counted. box_tests counts, in the compound-compound algorithm only, every node-pair test of its tree walk and every
 * child-box test before a child-pair dispatch. The compound algorithm's own child-box test and its btDbvt walk are
 * not counted, so a single-shape object against a compound adds leaf or primitive tests but no box_tests.
 */
namespace tesseract::collision::bullet_internal
{
/**
 * @brief The shape a dispatch is counted as: the shape a CastHullShape wraps, otherwise the shape itself.
 *
 * A cast manager wraps every convex child of an active link in a CastHullShape (CUSTOM_CONVEX_SHAPE_TYPE), a mesh
 * triangle included. Counting the wrapper would make the same link pair count differently on a discrete and a cast
 * manager; counting the wrapped shape keeps the two the same.
 */
inline const btCollisionShape* countedShape(const btCollisionShape* shape)
{
  if (shape->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE)
    return shape;
  assert(dynamic_cast<const CastHullShape*>(shape) != nullptr);
  return static_cast<const CastHullShape*>(shape)->m_shape;  // NOLINT
}

/**
 * @brief Count one child-pair dispatch by the shapes it is called on, each classified by countedShape().
 *
 * A mesh's children are triangle shapes (TRIANGLE_SHAPE_PROXYTYPE). Exactly one triangle against a convex
 * non-triangle shape (a primitive or a convex hull) is a leaf test. Two shapes of which neither is a triangle nor a
 * compound is a primitive test. Anything else (a compound child, two triangles, a triangle against a non-convex
 * shape) counts nothing here; a compound child is counted by its own dispatch. A CastHullShape counts as the shape it
 * wraps, so a cast triangle is a triangle.
 */
inline void countChildDispatch(const btCollisionShape* dispatched0, const btCollisionShape* dispatched1)
{
  const btCollisionShape* shape0 = countedShape(dispatched0);
  const btCollisionShape* shape1 = countedShape(dispatched1);
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
