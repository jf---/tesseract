/**
 * @file narrowphase_stats.cpp
 * @brief Per-thread counts of narrowphase and bounding-box tests: the one definition of the storage
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

#include <tesseract/collision/narrowphase_stats.h>

namespace tesseract::collision
{
namespace
{
// Defined once, in this library, so every backend plugin and caller shares one total per thread.
thread_local NarrowphaseStats g_narrowphase_stats;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
}  // namespace

NarrowphaseStats narrowphaseStats() { return g_narrowphase_stats; }

void resetNarrowphaseStats() { g_narrowphase_stats = NarrowphaseStats{}; }

void addNarrowphaseStats(const NarrowphaseStats& delta)
{
  g_narrowphase_stats.leaf_tests += delta.leaf_tests;
  g_narrowphase_stats.primitive_tests += delta.primitive_tests;
  g_narrowphase_stats.box_tests += delta.box_tests;
}

void countLeafTest() { ++g_narrowphase_stats.leaf_tests; }

void countPrimitiveTest() { ++g_narrowphase_stats.primitive_tests; }

void countBoxTest() { ++g_narrowphase_stats.box_tests; }
}  // namespace tesseract::collision
