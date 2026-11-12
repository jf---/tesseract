#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/test_suite/collision_cast_cc_time_unit.hpp>
#include <tesseract/collision/bullet/bullet_utils.h>
#include <cmath>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeFaceSweep)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeFaceSweepTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeFaceSweep)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeFaceSweepTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeFaceSweepPenetration)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeFaceSweepPenetrationTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeFaceSweepPenetration)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeFaceSweepPenetrationTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeFaceSweepDeepPenetration)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeFaceSweepDeepPenetrationTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeFaceSweepDeepPenetration)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeFaceSweepDeepPenetrationTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeBeyondSupportPoint)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeBeyondSupportPointTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeBeyondSupportPoint)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeBeyondSupportPointTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeTiltedSweep)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeTiltedSweepTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeTiltedSweep)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeTiltedSweepTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeRotationSweep)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeRotationSweepTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeRotationSweep)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeRotationSweepTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeTie)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeTieTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeTie)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeTieTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeEndFace)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeEndFaceTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeEndFace)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeEndFaceTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeStartFace)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeStartFaceTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeStartFace)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeStartFaceTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCastTimeExactEndpoints)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCastTimeExactEndpointsTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCastTimeExactEndpoints)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCastTimeExactEndpointsTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastContactTimeStaysInsideTheSweep)  // NOLINT
{
  using tesseract::collision::bullet_internal::castContactTime;
  // A weight below half an ulp of the other: w1 / (w0 + w1) rounds to an end, yet both poses carry weight
  const double tiny = std::ldexp(1.0, -60);  // 2^-60 < 2^-53

  const auto near_end = castContactTime(tiny, 1.0);
  EXPECT_EQ(near_end.cc_time, std::nextafter(1.0, 0.0));
  EXPECT_EQ(near_end.cc_type, ContinuousCollisionType::CCType_Between);

  const auto near_start = castContactTime(1.0, tiny);
  EXPECT_GT(near_start.cc_time, 0.0);  // 2^-60 itself is representable: no rounding to 0 here
  EXPECT_EQ(near_start.cc_type, ContinuousCollisionType::CCType_Between);

  // A quotient that underflows to 0: the smallest weight against the largest
  const auto underflow = castContactTime(std::numeric_limits<double>::max(), std::numeric_limits<double>::denorm_min());
  EXPECT_EQ(underflow.cc_time, std::nextafter(0.0, 1.0));
  EXPECT_EQ(underflow.cc_type, ContinuousCollisionType::CCType_Between);

  EXPECT_EQ(castContactTime(0.0, 1.0).cc_time, 1.0);
  EXPECT_EQ(castContactTime(0.0, 1.0).cc_type, ContinuousCollisionType::CCType_Time1);
  EXPECT_EQ(castContactTime(1.0, 0.0).cc_time, 0.0);
  EXPECT_EQ(castContactTime(1.0, 0.0).cc_type, ContinuousCollisionType::CCType_Time0);
  EXPECT_EQ(castContactTime(0.5, 0.5).cc_time, 0.5);
  EXPECT_EQ(castContactTime(0.5, 0.5).cc_type, ContinuousCollisionType::CCType_Between);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
