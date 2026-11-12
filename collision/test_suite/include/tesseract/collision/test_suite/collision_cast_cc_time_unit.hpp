#ifndef TESSERACT_COLLISION_COLLISION_CAST_CC_TIME_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_CAST_CC_TIME_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <octomap/octomap.h>
#include <cmath>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

/**
 * @file collision_cast_cc_time_unit.hpp
 * @brief The time a cast reports for a contact on its swept hull
 *
 * A cast checks the convex hull of the shape at t = 0 and at t = 1. Every support point of that hull is a support
 * point of one of the two poses, so the nearest point on the hull is a convex combination w = sum_i l_i v_i of points
 * v_i taken from one pose or the other. The contact time is alpha, the total weight on the points from t = 1:
 * w = (1 - alpha) p0 + alpha p1, with p0 on the shape at t = 0 and p1 on the shape at t = 1. A contact is typed
 * CCType_Time0 or CCType_Time1 only when alpha is exactly 0 or 1, otherwise CCType_Between. A support point that is
 * the same point at both poses counts half to each (R-F7, 2026-11-11).
 *
 * nearest_points_local is sum_i l_i L_i, with L_i the point v_i in the link frame of its own pose (R-F11,
 * 2026-11-11). For a translation that is the point of the link at time alpha that sits at w; for a rotation it is
 * (1 - alpha) L0(p0) + alpha L1(p1), with Lk the link frame at pose k.
 *
 * Each case below is built so that alpha is the same for every simplex GJK or EPA may end on: all t = 0 vertices of
 * the hull feature that holds the nearest point lie on one plane, all t = 1 vertices on a parallel plane, so the
 * position of w between those planes fixes alpha. Every expected value is derived from the geometry, never read
 * from an engine.
 */
namespace tesseract::collision::test_suite
{
namespace detail
{
/** @brief Radius of the static sphere (m) */
constexpr double CC_TIME_SPHERE_RADIUS = 0.2;
/** @brief Gap between the swept box and the sphere (m); positive, so the contact is a distance, not a penetration */
constexpr double CC_TIME_GAP = 0.3;
/** @brief Depth the sphere reaches into the swept box in the shallow penetration case (m): less than its radius, so
 * its centre stays outside the hull and GJK on the sphere's centre resolves it, with the radius as margin */
constexpr double CC_TIME_DEPTH = 0.05;
/** @brief Depth the sphere reaches into the swept box in the deep penetration case (m): more than its radius, so its
 * centre is inside the hull and EPA resolves it */
constexpr double CC_TIME_DEEP_DEPTH = 0.25;
/** @brief Contact margin (m), larger than the gap so the pair is reported */
constexpr double CC_TIME_MARGIN = 0.5;
/** @brief Tolerance on every position (m): a tenth of the 15 um planning resolution (MH50 II-20 repeatability
 * 0.15 mm / 10). A tolerance on cc_time is this divided by the length of the sweep it is measured along. */
constexpr double CC_TIME_POSITION_TOLERANCE = 1.5e-6;

/** @brief Reported contact between the cast box and the static sphere, and the box's index in it */
struct CastBoxContact
{
  ContactResult contact;
  std::size_t box_index{ 0 };
};

inline CastBoxContact castBoxPastSphere(ContinuousContactManager& checker,
                                        const Eigen::Vector3d& box_size,
                                        const Eigen::Isometry3d& box_start,
                                        const Eigen::Isometry3d& box_end,
                                        const Eigen::Vector3d& sphere_centre)
{
  const tesseract::common::LinkId box_link("cast_box");
  const tesseract::common::LinkId sphere_link("static_sphere");

  const CollisionShapesConst box_shapes{ std::make_shared<tesseract::geometry::Box>(
      box_size.x(), box_size.y(), box_size.z()) };
  const CollisionShapesConst sphere_shapes{ std::make_shared<tesseract::geometry::Sphere>(CC_TIME_SPHERE_RADIUS) };
  const tesseract::common::VectorIsometry3d identity{ Eigen::Isometry3d::Identity() };

  checker.addCollisionObject(box_link, 0, box_shapes, identity);
  checker.addCollisionObject(sphere_link, 0, sphere_shapes, identity);
  checker.setActiveCollisionObjects({ box_link });
  checker.setCollisionMarginData(CollisionMarginData(CC_TIME_MARGIN));

  Eigen::Isometry3d sphere_pose = Eigen::Isometry3d::Identity();
  sphere_pose.translation() = sphere_centre;
  checker.setCollisionObjectsTransform(sphere_link, sphere_pose);
  checker.setCollisionObjectsTransform(box_link, box_start, box_end);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  CastBoxContact out;
  EXPECT_EQ(result_vector.size(), 1U);
  if (result_vector.empty())
    return out;
  out.contact = result_vector[0];
  out.box_index = (out.contact.link_ids[0] == box_link) ? 0 : 1;
  return out;
}

/** @brief True if the box of @p box_size at @p box_pose holds @p point, within the position tolerance */
inline bool boxHolds(const Eigen::Vector3d& box_size, const Eigen::Isometry3d& box_pose, const Eigen::Vector3d& point)
{
  const Eigen::Vector3d local = box_pose.inverse() * point;
  return (local.cwiseAbs().array() <= (box_size.array() / 2.0) + CC_TIME_POSITION_TOLERANCE).all();
}

/** @brief The pose of a box translated from @p start to @p end at time @p t */
inline Eigen::Isometry3d interpolateTranslation(const Eigen::Isometry3d& start, const Eigen::Isometry3d& end, double t)
{
  Eigen::Isometry3d pose = start;
  pose.translation() = start.translation() + t * (end.translation() - start.translation());
  return pose;
}

/**
 * @brief Checks the contact of an axis-aligned box swept along +x by @p sweep against its bottom face
 *
 * The swept bottom face spans x in [-sx/2, sweep + sx/2]. Its t = 0 vertices are the corners at x = -sx/2 and its
 * t = 1 vertices the corners at x = sweep + sx/2 (the t = 0 corners at +sx/2 and the t = 1 corners at sweep - sx/2
 * are never a support point of the hull), so alpha = (x + sx/2) / (sweep + sx). @p gap is the distance from the
 * face to the sphere, negative for a penetration.
 */
inline void checkFaceSweep(ContinuousContactManager& checker,
                           const Eigen::Vector3d& box_size,
                           double sweep,
                           const Eigen::Vector2d& contact_xy,
                           double gap)
{
  const Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation().x() = sweep;
  const double face_z = -box_size.z() / 2.0;
  const Eigen::Vector3d sphere_centre(contact_xy.x(), contact_xy.y(), face_z - gap - CC_TIME_SPHERE_RADIUS);

  const CastBoxContact c = castBoxPastSphere(checker, box_size, start, end, sphere_centre);
  const std::size_t i = c.box_index;

  const double chord = sweep + box_size.x();
  const double alpha = (contact_xy.x() + (box_size.x() / 2.0)) / chord;

  EXPECT_NEAR(c.contact.distance, gap, CC_TIME_POSITION_TOLERANCE);
  const Eigen::Vector3d nearest = c.contact.nearest_points[i];
  const Eigen::Vector3d expected_nearest(contact_xy.x(), contact_xy.y(), face_z);
  EXPECT_LT((nearest - expected_nearest).norm(), CC_TIME_POSITION_TOLERANCE) << nearest.transpose();

  EXPECT_NEAR(c.contact.cc_time[i], alpha, CC_TIME_POSITION_TOLERANCE / chord);
  EXPECT_EQ(c.contact.cc_type[i], ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(boxHolds(box_size, interpolateTranslation(start, end, c.contact.cc_time[i]), nearest))
      << "cc_time " << c.contact.cc_time[i] << " names a box pose that does not hold the nearest point "
      << nearest.transpose();

  // The link frame is the box frame: at time alpha the point of the box at w
  const Eigen::Vector3d expected_local = interpolateTranslation(start, end, alpha).inverse() * expected_nearest;
  EXPECT_LT((c.contact.nearest_points_local[i] - expected_local).norm(), CC_TIME_POSITION_TOLERANCE)
      << c.contact.nearest_points_local[i].transpose() << " expected " << expected_local.transpose();
}
}  // namespace detail

/**
 * @brief A face contact off the support chord, where a time read from the averaged support points is wrong.
 *
 * A unit box slides 2 m along +x with its bottom face level; the sphere sits below (-0.3, 0.4), so the nearest point
 * is (-0.3, 0.4, -0.5), 0.4 m to the side of the chord through the face centres. alpha = (-0.3 + 0.5) / 3 = 1/15:
 * the box at 1/15 spans x in [-0.367, 0.633] and holds the point (it does for t in [0, 0.1]).
 *
 * The averaged support points (the face centres) are at x = 0 and x = 2. The ratio of the distances to them,
 * 0.5 / (0.5 + 2.3345) = 0.176, names a box spanning x in [-0.147, 0.853] that misses the point. The projection onto
 * the chord, -0.15, clamps to 0 and types the contact CCType_Time0.
 */
inline void runCastTimeFaceSweepTest(ContinuousContactManager& checker)
{
  detail::checkFaceSweep(checker, Eigen::Vector3d::Ones(), 2.0, Eigen::Vector2d(-0.3, 0.4), detail::CC_TIME_GAP);
}

/**
 * @brief A shallow penetration of the face sweep: the time comes from the GJK simplex, as for a distance.
 *
 * The same sweep and contact point as runCastTimeFaceSweepTest, with the sphere 0.05 m into the swept bottom face.
 * Its centre is outside the hull, so GJK finds the nearest point to it and the sphere's radius makes it a
 * penetration. The witness on the hull is (-0.3, 0.4, -0.5) and alpha = 1/15.
 */
inline void runCastTimeFaceSweepPenetrationTest(ContinuousContactManager& checker)
{
  detail::checkFaceSweep(checker, Eigen::Vector3d::Ones(), 2.0, Eigen::Vector2d(-0.3, 0.4), -detail::CC_TIME_DEPTH);
}

/**
 * @brief A deep penetration of the face sweep: the time comes from the EPA face that holds the witness.
 *
 * The same sweep and contact point, with the sphere 0.25 m into the swept bottom face: its centre is 0.05 m inside
 * the hull, so GJK cannot separate the shapes and EPA finds the penetration. The shallowest way out is straight down
 * (0.25 m; out through x = -0.5 takes 0.4 m, through y = 0.5 takes 0.3 m), so the witness on the hull is
 * (-0.3, 0.4, -0.5) and alpha = 1/15.
 */
inline void runCastTimeFaceSweepDeepPenetrationTest(ContinuousContactManager& checker)
{
  detail::checkFaceSweep(
      checker, Eigen::Vector3d::Ones(), 2.0, Eigen::Vector2d(-0.3, 0.4), -detail::CC_TIME_DEEP_DEPTH);
}

/**
 * @brief The nearest point lies beyond the support point of the end pose.
 *
 * The unit box slides +1 m along x; the sphere sits below x = 1.45, 0.45 m past the end face centre. The swept bottom
 * face spans x in [-0.5, 1.5], so alpha = (1.45 + 0.5) / 2 = 0.975. The box holds the point for t >= 0.95, and alpha
 * is not an end of the sweep, so the contact is CCType_Between.
 */
inline void runCastTimeBeyondSupportPointTest(ContinuousContactManager& checker)
{
  detail::checkFaceSweep(checker, Eigen::Vector3d::Ones(), 1.0, Eigen::Vector2d(1.45, 0), detail::CC_TIME_GAP);
}

/**
 * @brief A tilted box whose support feature is an edge, swept across the edge: one time holds the nearest point.
 *
 * The unit box is rotated 45 degrees about x, so its lowest feature is the edge along x at z = -sqrt(2)/2. It slides
 * +2 m along y, perpendicular to the edge, sweeping the rectangle x in [-0.5, 0.5], y in [0, 2]. The t = 0 vertices of
 * that rectangle are at y = 0 and the t = 1 vertices at y = 2. The sphere sits below (0.4, 0.5), so the nearest point
 * is (0.4, 0.5, -sqrt(2)/2) and alpha = 0.5 / 2 = 0.25, the only time the edge holds it.
 */
inline void runCastTimeTiltedSweepTest(ContinuousContactManager& checker)
{
  using namespace detail;
  const Eigen::Vector3d box_size = Eigen::Vector3d::Ones();
  const double half_diagonal = std::sqrt(2.0) / 2.0;
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.linear() = Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Isometry3d end = start;
  end.translation().y() = 2.0;
  const Eigen::Vector3d sphere_centre(0.4, 0.5, -half_diagonal - CC_TIME_GAP - CC_TIME_SPHERE_RADIUS);

  const CastBoxContact c = castBoxPastSphere(checker, box_size, start, end, sphere_centre);
  const std::size_t i = c.box_index;

  EXPECT_NEAR(c.contact.distance, CC_TIME_GAP, CC_TIME_POSITION_TOLERANCE);
  EXPECT_EQ(c.contact.cc_type[i], ContinuousCollisionType::CCType_Between);
  const Eigen::Vector3d nearest = c.contact.nearest_points[i];
  EXPECT_LT((nearest - Eigen::Vector3d(0.4, 0.5, -half_diagonal)).norm(), CC_TIME_POSITION_TOLERANCE)
      << nearest.transpose();

  const double chord = 2.0;
  EXPECT_NEAR(c.contact.cc_time[i], 0.25, CC_TIME_POSITION_TOLERANCE / chord);
  EXPECT_TRUE(boxHolds(box_size, interpolateTranslation(start, end, c.contact.cc_time[i]), nearest))
      << "cc_time " << c.contact.cc_time[i] << " names a box pose that does not hold the nearest point "
      << nearest.transpose();

  const Eigen::Vector3d expected_local =
      interpolateTranslation(start, end, 0.25).inverse() * Eigen::Vector3d(0.4, 0.5, -half_diagonal);
  EXPECT_LT((c.contact.nearest_points_local[i] - expected_local).norm(), CC_TIME_POSITION_TOLERANCE)
      << c.contact.nearest_points_local[i].transpose() << " expected " << expected_local.transpose();
}

/**
 * @brief A rotating box: the nearest point is a combination of a point on each pose, weighted by cc_time.
 *
 * A box of size 1 x 0.5 x 1 (half extents a = 0.5, b = 0.25, c = 0.5) turns from +45 to -45 degrees about x while
 * moving +1 m along y. At +45 degrees its lowest feature is the edge along x through the corner (y, z) = (-b, -c),
 * at y0 = (c - b) / sqrt(2), z = -(b + c) / sqrt(2); at -45 degrees it is the edge through (b, -c), at
 * y1 = 1 - y0 and the same z. Both edges are equally deep, so the hull's lowest face is the rectangle
 * x in [-a, a], y in [y0, y1], with its t = 0 vertices on y = y0 and its t = 1 vertices on y = y1. The sphere sits
 * below (0.2, 0.4), so w = (0.2, 0.4, z) and alpha = (0.4 - y0) / (y1 - y0) = 0.345.
 *
 * The check states the definition: w = (1 - alpha) p0 + alpha p1 with p0 = (0.2, y0, z) on the box at t = 0 and
 * p1 = (0.2, y1, z) on the box at t = 1. The averaged support points are the edge midpoints (0, y0, z) and
 * (0, y1, z); the ratio of the distances to them is 0.390, not alpha.
 */
inline void runCastTimeRotationSweepTest(ContinuousContactManager& checker)
{
  using namespace detail;
  const Eigen::Vector3d box_size(1.0, 0.5, 1.0);
  const double b = box_size.y() / 2.0;
  const double c_half = box_size.z() / 2.0;
  const double y0 = (c_half - b) / std::sqrt(2.0);
  const double y1 = 1.0 - y0;
  const double z = -(b + c_half) / std::sqrt(2.0);

  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.linear() = Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.linear() = Eigen::AngleAxisd(-M_PI / 4.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  end.translation().y() = 1.0;
  const Eigen::Vector3d sphere_centre(0.2, 0.4, z - CC_TIME_GAP - CC_TIME_SPHERE_RADIUS);

  const CastBoxContact c = castBoxPastSphere(checker, box_size, start, end, sphere_centre);
  const std::size_t i = c.box_index;

  EXPECT_NEAR(c.contact.distance, CC_TIME_GAP, CC_TIME_POSITION_TOLERANCE);
  EXPECT_EQ(c.contact.cc_type[i], ContinuousCollisionType::CCType_Between);
  const Eigen::Vector3d nearest = c.contact.nearest_points[i];
  EXPECT_LT((nearest - Eigen::Vector3d(0.2, 0.4, z)).norm(), CC_TIME_POSITION_TOLERANCE) << nearest.transpose();

  const double chord = y1 - y0;
  const double alpha = (0.4 - y0) / chord;
  EXPECT_NEAR(c.contact.cc_time[i], alpha, CC_TIME_POSITION_TOLERANCE / chord);

  const double t = c.contact.cc_time[i];
  const Eigen::Vector3d p0(nearest.x(), y0, z);
  const Eigen::Vector3d p1(nearest.x(), y1, z);
  EXPECT_TRUE(boxHolds(box_size, start, p0)) << "p0 " << p0.transpose() << " is not on the box at t = 0";
  EXPECT_TRUE(boxHolds(box_size, end, p1)) << "p1 " << p1.transpose() << " is not on the box at t = 1";
  EXPECT_LT((((1.0 - t) * p0) + (t * p1) - nearest).norm(), CC_TIME_POSITION_TOLERANCE)
      << "cc_time " << t << " does not weight a point on each pose into the nearest point " << nearest.transpose();

  // Each pose's part of w in the link frame of that pose, weighted as in w
  const Eigen::Vector3d expected_local = ((1.0 - alpha) * (start.inverse() * Eigen::Vector3d(0.2, y0, z))) +
                                         (alpha * (end.inverse() * Eigen::Vector3d(0.2, y1, z)));
  EXPECT_LT((c.contact.nearest_points_local[i] - expected_local).norm(), CC_TIME_POSITION_TOLERANCE)
      << c.contact.nearest_points_local[i].transpose() << " expected " << expected_local.transpose();
}

/**
 * @brief A cast from a pose to the same pose: every support point is the same point at both poses (R-F7).
 *
 * The unit box sits at (0.3, -0.2, 0) at both ends; the sphere sits below (0.1, 0), so w = (0.1, 0, -0.5). Each
 * vertex of the simplex counts half to each pose, so alpha = 0.5 and the contact is CCType_Between, whatever simplex
 * GJK ends on. Tagging a tie by the support rule alone (t = 1 wins a tie) would give 1 and CCType_Time1.
 * nearest_points_local is w in the box frame, (-0.2, 0.2, -0.5).
 */
inline void runCastTimeTieTest(ContinuousContactManager& checker)
{
  using namespace detail;
  const Eigen::Vector3d box_size = Eigen::Vector3d::Ones();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.3, -0.2, 0);
  const Eigen::Vector3d sphere_centre(0.1, 0, -0.5 - CC_TIME_GAP - CC_TIME_SPHERE_RADIUS);

  const CastBoxContact c = castBoxPastSphere(checker, box_size, pose, pose, sphere_centre);
  const std::size_t i = c.box_index;

  EXPECT_NEAR(c.contact.distance, CC_TIME_GAP, CC_TIME_POSITION_TOLERANCE);
  const Eigen::Vector3d nearest = c.contact.nearest_points[i];
  EXPECT_LT((nearest - Eigen::Vector3d(0.1, 0, -0.5)).norm(), CC_TIME_POSITION_TOLERANCE) << nearest.transpose();
  EXPECT_DOUBLE_EQ(c.contact.cc_time[i], 0.5);
  EXPECT_EQ(c.contact.cc_type[i], ContinuousCollisionType::CCType_Between);
  EXPECT_LT((c.contact.nearest_points_local[i] - Eigen::Vector3d(-0.2, 0.2, -0.5)).norm(), CC_TIME_POSITION_TOLERANCE)
      << c.contact.nearest_points_local[i].transpose();
}
/**
 * @brief A cast from a rotated pose to the same pose: the cast is recognised as one pose, so every support point is a
 * tie (R-F7) and alpha = 0.5.
 *
 * The unit box sits at (0.02, -0.01, 0) turned 0.3 rad about z at both ends (the pose of the plugin's stationary-mesh
 * case); its bottom face stays level. The sphere sits below (0.1, 0), inside that face (0.081 m from its centre), so
 * w = (0.1, 0, -0.5), alpha = 0.5 and CCType_Between, and nearest_points_local = pose^-1 w. For this rotation
 * pose^-1 * pose is not the identity in floating point, so a cast that composes the two poses sees two poses an ulp
 * apart instead of one.
 */
inline void runCastTimeRotatedTieTest(ContinuousContactManager& checker)
{
  using namespace detail;
  const Eigen::Vector3d box_size = Eigen::Vector3d::Ones();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.02, -0.01, 0);
  pose.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d sphere_centre(0.1, 0, -0.5 - CC_TIME_GAP - CC_TIME_SPHERE_RADIUS);

  const CastBoxContact c = castBoxPastSphere(checker, box_size, pose, pose, sphere_centre);
  const std::size_t i = c.box_index;

  const Eigen::Vector3d w(0.1, 0, -0.5);
  EXPECT_NEAR(c.contact.distance, CC_TIME_GAP, CC_TIME_POSITION_TOLERANCE);
  EXPECT_LT((c.contact.nearest_points[i] - w).norm(), CC_TIME_POSITION_TOLERANCE)
      << c.contact.nearest_points[i].transpose();
  EXPECT_DOUBLE_EQ(c.contact.cc_time[i], 0.5);
  EXPECT_EQ(c.contact.cc_type[i], ContinuousCollisionType::CCType_Between);
  EXPECT_LT((c.contact.nearest_points_local[i] - (pose.inverse() * w)).norm(), CC_TIME_POSITION_TOLERANCE)
      << c.contact.nearest_points_local[i].transpose();
}

/**
 * @brief A contact on a hull face made only of one pose's vertices is at exactly that end of the cast.
 *
 * The unit box moves from @p start to @p end, one 0.2 m below the other and shifted sideways, so the hull's lowest
 * face is the bottom face of the lower pose alone: every simplex vertex of a contact below it is that pose's, and the
 * time is exactly 0 or 1 with the matching type, never 1 + ulp or Between. Checked over a grid of contact points across
 * the face, so the barycentric weights take many values whose floating-point sum is not exactly 1.
 * nearest_points_local is w in the lower pose's frame.
 */
inline void checkOnePoseFace(ContinuousContactManager& checker,
                             const Eigen::Isometry3d& start,
                             const Eigen::Isometry3d& end,
                             bool end_is_lower)
{
  using namespace detail;
  const Eigen::Vector3d box_size = Eigen::Vector3d::Ones();
  const Eigen::Isometry3d& lower = end_is_lower ? end : start;
  const double face_z = lower.translation().z() - 0.5;
  constexpr int GRID = 9;            // contact points per side of the grid
  constexpr double GRID_HALF = 0.4;  // half width of the grid (m), inside the 0.5 m half face
  for (int a = 0; a < GRID; ++a)
  {
    for (int b = 0; b < GRID; ++b)
    {
      const double x = lower.translation().x() - GRID_HALF + (2.0 * GRID_HALF * a / (GRID - 1));
      const double y = lower.translation().y() - GRID_HALF + (2.0 * GRID_HALF * b / (GRID - 1));
      const Eigen::Vector3d w(x, y, face_z);
      const Eigen::Vector3d sphere_centre(x, y, face_z - CC_TIME_GAP - CC_TIME_SPHERE_RADIUS);

      checker.removeCollisionObject("cast_box");
      checker.removeCollisionObject("static_sphere");
      const CastBoxContact c = castBoxPastSphere(checker, box_size, start, end, sphere_centre);
      const std::size_t i = c.box_index;

      EXPECT_LT((c.contact.nearest_points[i] - w).norm(), CC_TIME_POSITION_TOLERANCE) << w.transpose();
      EXPECT_EQ(c.contact.cc_time[i], end_is_lower ? 1.0 : 0.0) << "at " << w.transpose();
      EXPECT_EQ(c.contact.cc_type[i],
                end_is_lower ? ContinuousCollisionType::CCType_Time1 : ContinuousCollisionType::CCType_Time0)
          << "at " << w.transpose();
      EXPECT_LT((c.contact.nearest_points_local[i] - (lower.inverse() * w)).norm(), CC_TIME_POSITION_TOLERANCE)
          << "at " << w.transpose();
    }
  }
}

/** @brief The end pose is the lower one: every contact on its face is at exactly t = 1, CCType_Time1 */
inline void runCastTimeEndFaceTest(ContinuousContactManager& checker)
{
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.3, 0.1, -0.2);
  checkOnePoseFace(checker, Eigen::Isometry3d::Identity(), end, true);
}

/** @brief The start pose is the lower one: every contact on its face is at exactly t = 0, CCType_Time0 */
inline void runCastTimeStartFaceTest(ContinuousContactManager& checker)
{
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(0.3, 0.1, -0.2);
  checkOnePoseFace(checker, start, Eigen::Isometry3d::Identity(), false);
}
/**
 * @brief A mesh link cast from a rotated pose to the same pose: every triangle child is one pose, alpha = 0.5.
 *
 * A mesh is a compound of triangles, each its own CastHullShape with its own cast transform. The square plate
 * (two triangles, half side 0.5 m, z = 0) sits at (0.02, -0.01, 0) turned 0.3 rad about z at both ends of the cast;
 * a static 0.2 m box hangs 0.03 m above it. Every support of every triangle is the same point at both poses, so the
 * contact is at alpha = 0.5 exactly, CCType_Between, with cc_transform equal to the link's pose.
 */
inline void runCastTimeStationaryMeshTest(ContinuousContactManager& checker)
{
  constexpr double PLATE_HALF_SIDE = 0.5;  // m
  constexpr double BOX_EDGE = 0.2;         // m
  constexpr double GAP = 0.03;             // m, box bottom face above the plate
  constexpr double MARGIN = 0.05;          // m, above the gap so the pair is reported

  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(-PLATE_HALF_SIDE, -PLATE_HALF_SIDE, 0.0);
  vertices->emplace_back(PLATE_HALF_SIDE, -PLATE_HALF_SIDE, 0.0);
  vertices->emplace_back(PLATE_HALF_SIDE, PLATE_HALF_SIDE, 0.0);
  vertices->emplace_back(-PLATE_HALF_SIDE, PLATE_HALF_SIDE, 0.0);
  auto faces = std::make_shared<Eigen::VectorXi>(8);
  *faces << 3, 0, 1, 2, 3, 0, 2, 3;

  checker.addCollisionObject("plate_link",
                             0,
                             CollisionShapesConst{ std::make_shared<tesseract::geometry::Mesh>(vertices, faces) },
                             { Eigen::Isometry3d::Identity() });
  checker.addCollisionObject(
      "box_link",
      0,
      CollisionShapesConst{ std::make_shared<tesseract::geometry::Box>(BOX_EDGE, BOX_EDGE, BOX_EDGE) },
      { Eigen::Isometry3d::Identity() });
  Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();
  box_pose.translation() = Eigen::Vector3d(0.0, 0.0, (BOX_EDGE / 2.0) + GAP);
  checker.setCollisionObjectsTransform("box_link", box_pose);
  checker.setDefaultCollisionMargin(MARGIN);
  checker.setActiveCollisionObjects({ "plate_link" });

  Eigen::Isometry3d plate_pose = Eigen::Isometry3d::Identity();
  plate_pose.translation() = Eigen::Vector3d(0.02, -0.01, 0.0);
  plate_pose.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  checker.setCollisionObjectsTransform("plate_link", plate_pose, plate_pose);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector results;
  result.flattenMoveResults(results);
  ASSERT_EQ(results.size(), 1U);
  const ContactResult& r = results[0];
  const std::size_t plate = (r.link_ids[0].name() == "plate_link") ? 0 : 1;
  EXPECT_NEAR(r.distance, GAP, detail::CC_TIME_POSITION_TOLERANCE);
  EXPECT_EQ(r.cc_time[plate], 0.5);
  EXPECT_EQ(r.cc_type[plate], ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(r.cc_transform[plate].isApprox(plate_pose, 1e-12));
}

/**
 * @brief The endpoints of the cast time are exact: Time0 is cc_time 0, Time1 is cc_time 1, Between is strictly inside.
 *
 * A 0.3 m box turns a quarter turn about z while crossing a 2 m octree of boxes from x = -3 to x = 3, cast in six
 * segments (the subdivision a trajectory check makes), with a 0.25 m margin. Every contact on every segment is checked:
 * a time is 0 or 1 exactly when all of its weight is on one pose, and then typed as that end; otherwise it lies
 * strictly between 0 and 1. A sum of barycentric weights that rounds past 1 (1 + 2^-52) must not be reported.
 */
inline void runCastTimeExactEndpointsTest(ContinuousContactManager& checker)
{
  tesseract::common::GeneralResourceLocator locator;
  const std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
  auto octree = std::make_shared<octomap::OcTree>(path);
  checker.addCollisionObject("octomap_link",
                             0,
                             CollisionShapesConst{ std::make_shared<tesseract::geometry::Octree>(
                                 octree, tesseract::geometry::OctreeSubType::BOX) },
                             { Eigen::Isometry3d::Identity() });
  checker.addCollisionObject("kin_link",
                             0,
                             CollisionShapesConst{ std::make_shared<tesseract::geometry::Box>(0.3, 0.3, 0.3) },
                             { Eigen::Isometry3d::Identity() });
  checker.setActiveCollisionObjects({ "kin_link" });
  checker.setDefaultCollisionMargin(0.25);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond q1(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const auto at = [&](double t) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d((6.0 * t) - 3.0, 0, 0);
    pose.linear() = q0.slerp(t, q1).toRotationMatrix();
    return pose;
  };

  constexpr int SEGMENTS = 6;
  int contacts = 0;
  for (int seg = 0; seg < SEGMENTS; ++seg)
  {
    checker.setCollisionObjectsTransform("kin_link", at(double(seg) / SEGMENTS), at(double(seg + 1) / SEGMENTS));
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::ALL));
    for (const auto& pair : result)
    {
      for (const auto& r : pair.second)
      {
        const std::size_t k = (r.link_ids[0].name() == "kin_link") ? 0 : 1;
        ++contacts;
        const double t = r.cc_time[k];
        if (r.cc_type[k] == ContinuousCollisionType::CCType_Time0)
          EXPECT_EQ(t, 0.0) << "segment " << seg;
        else if (r.cc_type[k] == ContinuousCollisionType::CCType_Time1)
          EXPECT_EQ(t, 1.0) << "segment " << seg;
        else
        {
          EXPECT_EQ(r.cc_type[k], ContinuousCollisionType::CCType_Between) << "segment " << seg;
          EXPECT_GT(t, 0.0) << "segment " << seg;
          EXPECT_LT(t, 1.0) << "segment " << seg << " cc_time " << std::hexfloat << t;
        }
      }
    }
  }
  EXPECT_GT(contacts, 0);
}
}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_CAST_CC_TIME_UNIT_HPP
