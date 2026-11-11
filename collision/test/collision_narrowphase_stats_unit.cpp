#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/narrowphase_stats.h>

using namespace tesseract::collision;

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


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
