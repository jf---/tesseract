/**
 * @file narrowphase_stats.h
 * @brief Per-thread counts of narrowphase and bounding-box tests, shared by every contact manager backend
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
#ifndef TESSERACT_COLLISION_NARROWPHASE_STATS_H
#define TESSERACT_COLLISION_NARROWPHASE_STATS_H

#include <cstdint>

namespace tesseract::collision
{
/**
 * @brief Totals of the tests a contact query ran, on the calling thread.
 *
 * The totals are thread-local: a query counts on the thread that runs it, and a thread reads only its own totals.
 * They are always on and never reset implicitly; bracket a query with resetNarrowphaseStats() or with two reads.
 */
struct NarrowphaseStats
{
  /** @brief Narrowphase calls on one triangle against one convex non-triangle shape */
  std::uint64_t leaf_tests{ 0 };
  /** @brief Narrowphase calls on two shapes of which neither is a triangle nor a compound */
  std::uint64_t primitive_tests{ 0 };
  /** @brief Bounding-volume tests at the backend's counted sites; backends count different tests */
  std::uint64_t box_tests{ 0 };
};

/** @brief A copy of the calling thread's totals */
NarrowphaseStats narrowphaseStats();

/** @brief Set the calling thread's totals to zero */
void resetNarrowphaseStats();

/** @brief Add a delta to the calling thread's totals (for a backend that counts in its own storage) */
void addNarrowphaseStats(const NarrowphaseStats& delta);

/** @brief Add one to the calling thread's leaf_tests */
void countLeafTest();

/** @brief Add one to the calling thread's primitive_tests */
void countPrimitiveTest();

/** @brief Add one to the calling thread's box_tests */
void countBoxTest();
}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_NARROWPHASE_STATS_H
