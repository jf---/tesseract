/**
 * @file bullet_utils.h
 * @brief Tesseract ROS Bullet environment utility function.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TESSERACT_COLLISION_BULLET_UTILS_H
#define TESSERACT_COLLISION_BULLET_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <btBulletCollisionCommon.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/logging.h>
#include <tesseract/collision/types.h>
#include <tesseract/collision/common.h>
#include <tesseract/collision/bullet/bullet_collision_shape_cache.h>

namespace tesseract::collision::bullet_internal
{
#define METERS

const btScalar BULLET_MARGIN = btScalar(0.0);
const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = btScalar(0.01) METERS;
const btScalar BULLET_LENGTH_TOLERANCE = btScalar(0.001) METERS;
const btScalar BULLET_EPSILON = btScalar(1e-3);
const btScalar BULLET_DEFAULT_CONTACT_DISTANCE = btScalar(0.05);
const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

btVector3 convertEigenToBt(const Eigen::Vector3d& v);

Eigen::Vector3d convertBtToEigen(const btVector3& v);

btQuaternion convertEigenToBt(const Eigen::Quaterniond& q);

btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r);

Eigen::Matrix3d convertBtToEigen(const btMatrix3x3& r);

btTransform convertEigenToBt(const Eigen::Isometry3d& t);

Eigen::Isometry3d convertBtToEigen(const btTransform& t);

/**
 * @brief This is a tesseract bullet collsion object.
 *
 * It is a wrapper around bullet's collision object which
 * contains specific information related to tesseract
 */
class CollisionObjectWrapper : public btCollisionObject
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<CollisionObjectWrapper>;
  using ConstPtr = std::shared_ptr<const CollisionObjectWrapper>;

  CollisionObjectWrapper() = default;
  CollisionObjectWrapper(tesseract::common::LinkId id,
                         const int& type_id,
                         CollisionShapesConst shapes,
                         tesseract::common::VectorIsometry3d shape_poses);

  short int m_collisionFilterGroup{ btBroadphaseProxy::KinematicFilter };
  short int m_collisionFilterMask{ btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter };
  bool m_enabled{ true };

  /** @brief Get the collision object id */
  const tesseract::common::LinkId& getLinkId() const;
  /** @brief Get a user defined type */
  const int& getTypeID() const;
  /** \brief Check if two CollisionObjectWrapper objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const;

  const CollisionShapesConst& getCollisionGeometries() const;

  const tesseract::common::VectorIsometry3d& getCollisionGeometriesTransforms() const;

  /**
   * @brief Get the collision object's axis aligned bounding box
   * This AABB is extended by half of the contact processing threshold, so the broadphase takes
   * the full contact processing threshold into account when testing for the overlap of two AABBs.
   * @param aabb_min The minimum point
   * @param aabb_max The maximum point
   */
  void getAABB(btVector3& aabb_min, btVector3& aabb_max) const;

  /**
   * @brief This clones the collision objects but not the collision shape wich is const.
   * @return Shared Pointer to the cloned collision object
   */
  std::shared_ptr<CollisionObjectWrapper> clone();

  void manage(const std::shared_ptr<BulletCollisionShape>& t);

  void manageReserve(std::size_t s);

protected:
  /** @brief The id of the collision object */
  tesseract::common::LinkId m_link_id;
  /** @brief A user defined type id */
  int m_type_id{ -1 };
  /* @brief The shapes that define the collision object */
  CollisionShapesConst m_shapes;
  /**< @brief The shapes poses information */
  tesseract::common::VectorIsometry3d m_shape_poses;
  /** @brief This manages the collision shape pointer so they get destroyed */
  std::vector<std::shared_ptr<BulletCollisionShape>> m_data;
};

using COW = CollisionObjectWrapper;
using Link2Cow = std::unordered_map<tesseract::common::LinkId, COW::Ptr>;
using Link2ConstCow = std::unordered_map<tesseract::common::LinkId, COW::ConstPtr>;

/**
 * @brief A support point of a CastHullShape, by the pose of the cast it is taken at
 *
 * end_weight is 0 for a point of the shape at t = 0, 1 for a point of the shape at t = 1, and 0.5 when the two poses'
 * supports are the same point, which then belongs to both (R-F7). own_pose_local is the point in the wrapped shape's
 * frame at its own pose, mixed with the same weights: (1 - end_weight) u0 + end_weight u1, u0 and u1 the wrapped
 * shape's supports at t = 0 and t = 1 in its own frame.
 */
struct CastSupportVertex
{
  btScalar end_weight{ 0 };
  btVector3 own_pose_local{ 0, 0, 0 };
};

/**
 * @brief A convex combination sum_i l_i v_i of CastSupportVertex: the weight on each pose, sum_i l_i (1 - e_i) and
 * sum_i l_i e_i, and sum_i l_i own_pose_local_i
 *
 * The two pose weights are summed apart, so a combination with all its weight on one pose has exactly zero on the
 * other (each term is l_i times 0), whatever the floating-point sum of the l_i.
 */
struct CastSupportSum
{
  btScalar start_weight{ 0 };
  btScalar end_weight{ 0 };
  btVector3 own_pose_local{ 0, 0, 0 };

  void add(btScalar weight, const CastSupportVertex& vertex)
  {
    start_weight += weight * (btScalar(1) - vertex.end_weight);
    end_weight += weight * vertex.end_weight;
    own_pose_local += vertex.own_pose_local * weight;
  }
};

/**
 * @brief Where the nearest point a narrowphase query reported on a CastHullShape lies between the poses of the cast
 *
 * The query's nearest point on the hull is a convex combination w = sum_i l_i v_i (sum_i l_i = 1) of support points
 * v_i of the hull, each the wrapped shape's support at the start or at the end pose. end_weight is sum_i l_i over the
 * v_i taken at the end pose (sum.end_weight): w = (1 - end_weight) p0 + end_weight p1, with p0 on the shape at t = 0
 * and p1 on the shape at t = 1. sum.own_pose_local is sum_i l_i u_i, u_i the point v_i in the wrapped shape's frame at
 * the pose it is taken at (see CastSupportVertex).
 */
struct CastHullContactWeight
{
  enum class Kind : std::uint8_t
  {
    NONE,             /**< No query has recorded a contact since the last read */
    NO_NEAREST_POINT, /**< The query found the shapes intersecting and penetration was not requested, so it has none */
    NEAREST_POINT     /**< end_weight holds the weight of the reported nearest point */
  };
  Kind kind{ Kind::NONE };
  CastSupportSum sum;
};

/** @brief The time and type of a cast contact */
struct CastContactTime
{
  double cc_time{ -1 };
  ContinuousCollisionType cc_type{ ContinuousCollisionType::CCType_None };
};

/**
 * @brief The time and type of a cast contact from the weights its nearest point puts on the start (@p w0) and end
 * (@p w1) poses
 *
 * All weight on the end pose is exactly t = 1, CCType_Time1; all on the start pose exactly t = 0, CCType_Time0.
 * Otherwise the time is w1 / (w0 + w1), CCType_Between, kept strictly inside (0, 1): a quotient that rounds to an end
 * is reported as the double next to it, because a contact with weight on both poses is at neither end.
 */
CastContactTime castContactTime(btScalar w0, btScalar w1);

/** @brief This is a casted collision shape used for checking if an object is collision free between two transforms */
struct CastHullShape : public btConvexShape
{
public:
  btConvexShape* m_shape;
  btTransform m_t01;

  CastHullShape(btConvexShape* shape, const btTransform& t01);

  void updateCastTransform(const btTransform& t01);
  btVector3 localGetSupportingVertex(const btVector3& vec) const override;

  /** @brief The support vertex along @p vec, and in @p vertex the pose of the cast it is taken at */
  btVector3 localGetSupportingVertex(const btVector3& vec, CastSupportVertex& vertex) const;

  /**
   * @brief Records the weight of the nearest point found by the narrowphase query that is reporting a contact on
   * this hull. The query writes it just before it reports the contact; calculateContinuousData reads it.
   */
  void setContactWeight(const CastHullContactWeight& weight) const;

  /** @brief The recorded weight, which is cleared, so a contact without a fresh record is detected */
  CastHullContactWeight takeContactWeight() const;

  /// getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0, btVector3& aabbMin, btVector3& aabbMax) const override;

  const char* getName() const override;
  btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const override;

  // LCOV_EXCL_START
  // notice that the vectors should be unit length
  void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,
                                                         btVector3* supportVerticesOut,
                                                         int numVectors) const override;

  void getAabbSlow(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const override;

  void setLocalScaling(const btVector3& scaling) override;

  const btVector3& getLocalScaling() const override;

  void setMargin(btScalar margin) override;

  btScalar getMargin() const override;

  int getNumPreferredPenetrationDirections() const override;

  void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const override;

  void calculateLocalInertia(btScalar, btVector3&) const override;
  // LCOV_EXCL_STOP

private:
  /** @brief Written by the narrowphase query of the contact being reported. Per query state, like the cast transform:
   * a CastHullShape belongs to one cast object of one manager */
  mutable CastHullContactWeight contact_weight_;
};

/** @brief @p shape as a CastHullShape, or nullptr if it is not one */
const CastHullShape* asCastHullShape(const btConvexShape* shape);

void GetAverageSupport(const btConvexShape* shape,
                       const btVector3& localNormal,
                       btScalar& outsupport,
                       btVector3& outpt);

/**
 * @brief This transversus the parent tree to find the base object and return its world transform
 * This should be the links transform and it should only need to transverse twice at max.
 * @param cow Bullet collision object wrapper.
 * @return Transform of link in world coordinates
 */
btTransform getLinkTransformFromCOW(const btCollisionObjectWrapper* cow);

/**
 * @brief This is used to check if a collision check is required between the provided two collision objects
 * @param cow1 The first collision object
 * @param cow2 The second collision object
 * @param pair The link id pair of the two collision objects
 * @param validator  The contact allowed validator
 * @param verbose Indicate if verbose information should be printed to the terminal
 * @return True if the two collision objects should be checked for collision, otherwise false
 */
bool needsCollisionCheck(const COW& cow1,
                         const COW& cow2,
                         const tesseract::common::LinkIdPair& pair,
                         const std::shared_ptr<const tesseract::common::ContactAllowedValidator>& validator,
                         bool verbose = false);

/**
 * @brief This is used to check if a collision check is required between the provided two collision objects
 * (convenience overload; resolves the link id pair only when a validator decision requires it).
 * @param cow1 The first collision object
 * @param cow2 The second collision object
 * @param validator  The contact allowed validator
 * @param verbose Indicate if verbose information should be printed to the terminal
 * @return True if the two collision objects should be checked for collision, otherwise false
 */
bool needsCollisionCheck(const COW& cow1,
                         const COW& cow2,
                         const std::shared_ptr<const tesseract::common::ContactAllowedValidator>& validator,
                         bool verbose = false);

/**
 * @brief Calculate the continuous contact data for casted collision shape
 *
 * cc_time is the weight the reported nearest point puts on the end pose of the cast, as recorded on the
 * CastHullShape by the narrowphase query reporting the contact (see CastHullContactWeight). cc_type is CCType_Time0
 * or CCType_Time1 when that weight is exactly 0 or 1, otherwise CCType_Between. nearest_points_local is the
 * recorded sum_i l_i u_i in the link frame: each support point in the link frame of its own pose.
 * @param col Contact results
 * @param cow Bullet Collision Object Wrapper
 * @param link_tf_inv The links world transform inverse
 * @param link_index The link index in teh ContactResults the shape is associated with
 * @throws std::runtime_error if no narrowphase query recorded the weight of this contact
 */
void calculateContinuousData(ContactResult* col,
                             const btCollisionObjectWrapper* cow,
                             const btTransform& link_tf_inv,
                             size_t link_index);

/** @brief This is copied directly out of BulletWorld */
struct TesseractBridgedManifoldResult : public btManifoldResult
{
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  btCollisionWorld::ContactResultCallback& m_resultCallback;

  TesseractBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                 const btCollisionObjectWrapper* obj1Wrap,
                                 btCollisionWorld::ContactResultCallback& resultCallback);

  void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override;
};

/** @brief The BroadphaseContactResultCallback is used to report contact points */
struct BroadphaseContactResultCallback
{
  ContactTestData& collisions_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  bool verbose_;

  BroadphaseContactResultCallback(ContactTestData& collisions, bool verbose = false);

  virtual ~BroadphaseContactResultCallback() = default;
  BroadphaseContactResultCallback(const BroadphaseContactResultCallback&) = default;
  BroadphaseContactResultCallback& operator=(const BroadphaseContactResultCallback&) = delete;
  BroadphaseContactResultCallback(BroadphaseContactResultCallback&&) = default;
  BroadphaseContactResultCallback& operator=(BroadphaseContactResultCallback&&) = delete;

  virtual bool needsCollision(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) const;

  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int partId0,
                                   int index0,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int partId1,
                                   int index1) = 0;
};

struct DiscreteBroadphaseContactResultCallback : public BroadphaseContactResultCallback
{
  DiscreteBroadphaseContactResultCallback(ContactTestData& collisions, bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;
};

struct CastBroadphaseContactResultCallback : public BroadphaseContactResultCallback
{
  CastBroadphaseContactResultCallback(ContactTestData& collisions, bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;
};

struct TesseractBroadphaseBridgedManifoldResult : public btManifoldResult
{
  BroadphaseContactResultCallback& result_callback_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

  TesseractBroadphaseBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                           const btCollisionObjectWrapper* obj1Wrap,
                                           BroadphaseContactResultCallback& result_callback);

  void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override;
};

/**
 * @brief A callback function that is called as part of the broadphase collision checking.
 *
 * If the AABB of two collision objects are overlapping the processOverlap method is called
 * and they are checked for collision/distance and the results are stored in collision_.
 */
class TesseractCollisionPairCallback : public btOverlapCallback
{
  const btDispatcherInfo& dispatch_info_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  btCollisionDispatcher* dispatcher_;
  BroadphaseContactResultCallback& results_callback_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

public:
  TesseractCollisionPairCallback(const btDispatcherInfo& dispatchInfo,
                                 btCollisionDispatcher* dispatcher,
                                 BroadphaseContactResultCallback& results_callback);

  ~TesseractCollisionPairCallback() override = default;
  TesseractCollisionPairCallback(const TesseractCollisionPairCallback&) = default;
  TesseractCollisionPairCallback& operator=(const TesseractCollisionPairCallback&) = delete;
  TesseractCollisionPairCallback(TesseractCollisionPairCallback&&) = default;
  TesseractCollisionPairCallback& operator=(TesseractCollisionPairCallback&&) = delete;

  bool processOverlap(btBroadphasePair& pair) override;
};

/** @brief This class is used to filter broadphase */
class TesseractOverlapFilterCallback : public btOverlapFilterCallback
{
public:
  TesseractOverlapFilterCallback(bool verbose = false);

  bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override;

private:
  bool verbose_{ false };
};

/**
 * @brief Create a bullet collision shape from tesseract collision shape
 * @param geom Tesseract collision shape
 * @return Bullet collision shape.
 */
std::shared_ptr<BulletCollisionShape> createShapePrimitive(const CollisionShapeConstPtr& geom);

/**
 * @brief Update a collision objects filters
 * @param active Set of active collision object ids
 * @param cow The collision object to update.
 */
void updateCollisionObjectFilters(const std::unordered_set<tesseract::common::LinkId>& active, const COW::Ptr& cow);

COW::Ptr createCollisionObject(const tesseract::common::LinkId& id,
                               const int& type_id,
                               const CollisionShapesConst& shapes,
                               const tesseract::common::VectorIsometry3d& shape_poses,
                               bool enabled = true);

struct DiscreteCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactTestData& collisions_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  const COW::Ptr cow_;           // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  bool verbose_;

  DiscreteCollisionCollector(ContactTestData& collisions, COW::Ptr cow, bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;

  bool needsCollision(btBroadphaseProxy* proxy0) const override;
};

struct CastCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactTestData& collisions_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  const COW::Ptr cow_;           // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  bool verbose_;

  CastCollisionCollector(ContactTestData& collisions, COW::Ptr cow, bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;

  bool needsCollision(btBroadphaseProxy* proxy0) const override;
};

COW::Ptr makeCastCollisionObject(const COW::Ptr& cow);

/**
 * @brief Update the Broadphase AABB for the input collision object
 * @param cow The collision objects
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void updateBroadphaseAABB(const COW::Ptr& cow,
                          const std::unique_ptr<btBroadphaseInterface>& broadphase,
                          const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Remove the collision object from broadphase
 * @param cow The collision objects
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void removeCollisionObjectFromBroadphase(const COW::Ptr& cow,
                                         const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                         const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Remove a batch of collision objects from broadphase, clearing their cached pairs in one pass
 *
 * Observably equivalent to calling removeCollisionObjectFromBroadphase once per entry. The pair array is swept once
 * for the whole batch; destroyProxy still runs its own sweep per proxy, but finds nothing left to remove.
 *
 * @param cows The collision objects
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void removeCollisionObjectsFromBroadphase(const std::vector<COW::Ptr>& cows,
                                          const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                          const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Add the collision object to broadphase
 * @param cow The collision objects
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void addCollisionObjectToBroadphase(const COW::Ptr& cow,
                                    const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                    const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Update a collision objects filters for broadphase
 * @param active Set of active collision object LinkIds
 * @param cow The collision object to update.
 * @param broadphase The broadphase to update.
 * @param dispatcher The dispatcher.
 */
void updateCollisionObjectFilters(const std::unordered_set<tesseract::common::LinkId>& active,
                                  const COW::Ptr& cow,
                                  const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                  const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Drop the cached algorithm from every pair naming any of the given collision objects, in one pass
 *
 * Unlike removeCollisionObjectsFromBroadphase, the pairs themselves stay in the cache: only their cached algorithm
 * is dropped, so BroadPhaseFilter is consulted again for each on the next overlap test. Use after changing a
 * property, such as enabled state, that BroadPhaseFilter depends on.
 *
 * @param cows The collision objects whose pairs should be cleaned
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void cleanCollisionObjectsFromPairs(const std::vector<COW::Ptr>& cows,
                                    const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                    const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Refresh the broadphase data structure
 * @details When change certain properties of a collision object the broadphase is not aware so this function can be
 * called to trigger an update. For example, when changing active links this changes internal flags which may require
 * moving a collision object from the static BVH to the dynamic BVH so this function must be called.
 * @param cow The collision object to update.
 * @param broadphase The broadphase to update.
 * @param dispatcher The dispatcher.
 */
void refreshBroadphaseProxy(const COW::Ptr& cow,
                            const std::unique_ptr<btBroadphaseInterface>& broadphase,
                            const std::unique_ptr<btCollisionDispatcher>& dispatcher);
}  // namespace tesseract::collision::bullet_internal
#endif  // TESSERACT_COLLISION_BULLET_UTILS_H
