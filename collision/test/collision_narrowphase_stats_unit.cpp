#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <array>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/narrowphase_stats.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;

namespace
{
/** @brief Contact margin (m) larger than every box-to-triangle distance in these scenes, so every child pair
 * passes both of Bullet's box tests and reaches the narrowphase dispatch. */
constexpr double WIDE_MARGIN = 10.0;

/** @brief The 12 triangles of an axis-aligned unit cube centred on the origin. */
std::shared_ptr<tesseract::geometry::Mesh> unitCubeMesh()
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  for (const double x : { -0.5, 0.5 })
    for (const double y : { -0.5, 0.5 })
      for (const double z : { -0.5, 0.5 })
        vertices->emplace_back(x, y, z);

  // Vertex index = 4 * ix + 2 * iy + iz
  const std::vector<std::array<int, 3>> triangles{ { 0, 1, 3 }, { 0, 3, 2 }, { 4, 6, 7 }, { 4, 7, 5 },
                                                   { 0, 4, 5 }, { 0, 5, 1 }, { 2, 3, 7 }, { 2, 7, 6 },
                                                   { 0, 2, 6 }, { 0, 6, 4 }, { 1, 5, 7 }, { 1, 7, 3 } };
  auto faces = std::make_shared<Eigen::VectorXi>(static_cast<Eigen::Index>(4 * triangles.size()));
  Eigen::Index i = 0;
  for (const auto& t : triangles)
  {
    (*faces)[i++] = 3;
    (*faces)[i++] = t[0];
    (*faces)[i++] = t[1];
    (*faces)[i++] = t[2];
  }
  return std::make_shared<tesseract::geometry::Mesh>(vertices, faces);
}

/** @brief Two boxes at non-identity origins, so Bullet builds the link as a compound with two children. */
void twoBoxShapes(CollisionShapesConst& shapes,
                  tesseract::common::VectorIsometry3d& poses,
                  const Eigen::Vector3d& first,
                  const Eigen::Vector3d& second)
{
  Eigen::Isometry3d p0 = Eigen::Isometry3d::Identity();
  p0.translation() = first;
  p0.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
  p1.translation() = second;
  p1.linear() = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  shapes.push_back(std::make_shared<tesseract::geometry::Box>(0.1, 0.1, 0.1));
  shapes.push_back(std::make_shared<tesseract::geometry::Box>(0.1, 0.1, 0.1));
  poses.push_back(p0);
  poses.push_back(p1);
}

template <typename Manager>
void addMeshLink(Manager& checker)
{
  CollisionShapesConst shapes{ unitCubeMesh() };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("mesh_link", 0, shapes, poses);
}

template <typename Manager>
void addTwoBoxLink(Manager& checker)
{
  CollisionShapesConst shapes;
  tesseract::common::VectorIsometry3d poses;
  twoBoxShapes(shapes, poses, Eigen::Vector3d(0, 0.2, 1.5), Eigen::Vector3d(0, -0.2, 1.5));
  checker.addCollisionObject("boxes_link", 0, shapes, poses);
}
}  // namespace

TEST(TesseractNarrowphaseStatsUnit, ResetZeroesEveryCounter)  // NOLINT
{
  resetNarrowphaseStats();
  countLeafTest();
  countLeafTest();
  countPrimitiveTest();
  countBoxTest();
  countBoxTest();
  countBoxTest();
  NarrowphaseStats s = narrowphaseStats();
  EXPECT_EQ(s.leaf_tests, 2U);
  EXPECT_EQ(s.primitive_tests, 1U);
  EXPECT_EQ(s.box_tests, 3U);

  addNarrowphaseStats(NarrowphaseStats{ 5, 7, 11 });
  s = narrowphaseStats();
  EXPECT_EQ(s.leaf_tests, 7U);
  EXPECT_EQ(s.primitive_tests, 8U);
  EXPECT_EQ(s.box_tests, 14U);

  resetNarrowphaseStats();
  s = narrowphaseStats();
  EXPECT_EQ(s.leaf_tests, 0U);
  EXPECT_EQ(s.primitive_tests, 0U);
  EXPECT_EQ(s.box_tests, 0U);
}

TEST(TesseractNarrowphaseStatsUnit, TotalsArePerThread)  // NOLINT
{
  resetNarrowphaseStats();
  countLeafTest();

  NarrowphaseStats in_thread_before;
  NarrowphaseStats in_thread_after;
  std::thread worker([&] {
    in_thread_before = narrowphaseStats();
    countLeafTest();
    countLeafTest();
    countLeafTest();
    countPrimitiveTest();
    countBoxTest();
    in_thread_after = narrowphaseStats();
  });
  worker.join();

  EXPECT_EQ(in_thread_before.leaf_tests, 0U);
  EXPECT_EQ(in_thread_before.primitive_tests, 0U);
  EXPECT_EQ(in_thread_before.box_tests, 0U);
  EXPECT_EQ(in_thread_after.leaf_tests, 3U);
  EXPECT_EQ(in_thread_after.primitive_tests, 1U);
  EXPECT_EQ(in_thread_after.box_tests, 1U);

  const NarrowphaseStats here = narrowphaseStats();
  EXPECT_EQ(here.leaf_tests, 1U);
  EXPECT_EQ(here.primitive_tests, 0U);
  EXPECT_EQ(here.box_tests, 0U);
}

TEST(TesseractNarrowphaseStatsUnit, BulletDiscreteTwoBoxesAgainstTwelveTriangles)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  addMeshLink(checker);
  addTwoBoxLink(checker);
  checker.setActiveCollisionObjects({ "boxes_link", "mesh_link" });
  checker.setDefaultCollisionMargin(WIDE_MARGIN);

  resetNarrowphaseStats();
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  const NarrowphaseStats s = narrowphaseStats();

  EXPECT_FALSE(result.empty());
  EXPECT_EQ(s.leaf_tests, 24U);  // 2 boxes x 12 triangles
  EXPECT_EQ(s.primitive_tests, 0U);
  // Every leaf pair passes the tree-walk test (:282) and the child-box test (:183).
  EXPECT_GE(s.box_tests, 48U);
}

TEST(TesseractNarrowphaseStatsUnit, BulletDiscreteTwoPrimitiveLinks)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  {
    CollisionShapesConst shapes;
    tesseract::common::VectorIsometry3d poses;
    twoBoxShapes(shapes, poses, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(10, 0, 0));
    checker.addCollisionObject("a_link", 0, shapes, poses);
  }
  {
    // b's first box overlaps a's first box; b's second box is far from both of a's boxes.
    CollisionShapesConst shapes;
    tesseract::common::VectorIsometry3d poses;
    twoBoxShapes(shapes, poses, Eigen::Vector3d(0.05, 0, 0), Eigen::Vector3d(20, 0, 0));
    checker.addCollisionObject("b_link", 0, shapes, poses);
  }
  checker.setActiveCollisionObjects({ "a_link", "b_link" });
  checker.setDefaultCollisionMargin(0.0);

  resetNarrowphaseStats();
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  const NarrowphaseStats s = narrowphaseStats();

  EXPECT_FALSE(result.empty());
  EXPECT_EQ(s.leaf_tests, 0U);
  EXPECT_EQ(s.primitive_tests, 1U);  // one child pair whose boxes overlap
  // Tree walk (:282): root pair + 4 leaf pairs; child-box test (:183): the 1 leaf pair that passed it.
  EXPECT_EQ(s.box_tests, 6U);
}

TEST(TesseractNarrowphaseStatsUnit, BulletCastTwoBoxesAgainstTwelveTriangles)  // NOLINT
{
  BulletCastBVHManager checker;
  addMeshLink(checker);
  addTwoBoxLink(checker);
  checker.setActiveCollisionObjects({ "boxes_link" });
  checker.setDefaultCollisionMargin(WIDE_MARGIN);

  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.3, 0, 0);
  checker.setCollisionObjectsTransform("boxes_link", start, end);

  resetNarrowphaseStats();
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  const NarrowphaseStats s = narrowphaseStats();

  EXPECT_FALSE(result.empty());
  EXPECT_EQ(s.leaf_tests, 24U);  // 2 cast hulls x 12 triangles
  EXPECT_EQ(s.primitive_tests, 0U);
  EXPECT_GE(s.box_tests, 48U);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
