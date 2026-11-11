/**
 * @file contact_managers_factory_unit.cpp
 * @brief Tesseract collision contact managers factory test
 *
 * @author Levi Armstrong
 * @date October 25, 2021
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/contact_managers_plugin_factory.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/common/yaml_utils.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::collision;

void runContactManagersFactoryTest(const std::filesystem::path& config_path)
{
  tesseract::common::GeneralResourceLocator locator;
  ContactManagersPluginFactory factory(config_path, locator);
  YAML::Node plugin_config = YAML::LoadFile(config_path.string());
  tesseract::common::processYamlIncludeDirective(plugin_config, locator);

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& discrete_plugins = plugin_info["discrete_plugins"]["plugins"];
  const YAML::Node& continuous_plugins = plugin_info["continuous_plugins"]["plugins"];

  {
    std::vector<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::vector<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(discrete_plugins.size(), 3);
  for (auto cm_it = discrete_plugins.begin(); cm_it != discrete_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }

  EXPECT_EQ(continuous_plugins.size(), 2);
  for (auto cm_it = continuous_plugins.begin(); cm_it != continuous_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }

  factory.saveConfig(std::filesystem::path(tesseract::common::getTempPath()) / "contact_manager_plugins_export."
                                                                               "yaml");

  // Failures
  {
    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager("DoesNotExist");
    EXPECT_TRUE(cm == nullptr);
  }
  {
    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager("DoesNotExist");
    EXPECT_TRUE(cm == nullptr);
  }
  {
    tesseract::common::PluginInfo plugin_info;
    plugin_info.class_name = "DoesNotExistFactory";
    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager("DoesNotExist", plugin_info);
    EXPECT_TRUE(cm == nullptr);
  }
  {
    tesseract::common::PluginInfo plugin_info;
    plugin_info.class_name = "DoesNotExistFactory";
    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager("DoesNotExist", plugin_info);
    EXPECT_TRUE(cm == nullptr);
  }
}

TEST(TesseractContactManagersFactoryUnit, LoadAndExportPluginTest)  // NOLINT
{
  std::filesystem::path file_path(__FILE__);
  std::filesystem::path config_path = file_path.parent_path() / "contact_manager_plugins.yaml";
  runContactManagersFactoryTest(config_path);

  std::filesystem::path export_config_path = std::filesystem::path(tesseract::common::getTempPath()) / "contac"
                                                                                                       "t_"
                                                                                                       "manage"
                                                                                                       "r_"
                                                                                                       "plugin"
                                                                                                       "s_"
                                                                                                       "export"
                                                                                                       ".yaml";
  runContactManagersFactoryTest(export_config_path);
}

TEST(TesseractContactManagersFactoryUnit, LoadStringPluginTest)  // NOLINT
{
  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories
                              - tesseract_collision_fcl_factories
                            discrete_plugins:
                              default: BulletDiscreteBVHManager
                              plugins:
                                BulletDiscreteBVHManager:
                                  class: BulletDiscreteBVHManagerFactory
                                BulletDiscreteSimpleManager:
                                  class: BulletDiscreteSimpleManagerFactory
                                FCLDiscreteBVHManager:
                                  class: FCLDiscreteBVHManagerFactory
                            continuous_plugins:
                              default: BulletCastBVHManager
                              plugins:
                                BulletCastBVHManager:
                                  class: BulletCastBVHManagerFactory
                                BulletCastSimpleManager:
                                  class: BulletCastSimpleManagerFactory)";

  tesseract::common::GeneralResourceLocator locator;
  ContactManagersPluginFactory factory(config, locator);
  YAML::Node plugin_config = tesseract::common::loadYamlString(config, locator);

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& discrete_plugins = plugin_info["discrete_plugins"]["plugins"];
  const YAML::Node& continuous_plugins = plugin_info["continuous_plugins"]["plugins"];

  {
    std::vector<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::vector<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(discrete_plugins.size(), 3);
  for (auto cm_it = discrete_plugins.begin(); cm_it != discrete_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }

  EXPECT_EQ(continuous_plugins.size(), 2);
  for (auto cm_it = continuous_plugins.begin(); cm_it != continuous_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }
}

TEST(TesseractContactManagersFactoryUnit, InvalidDiscoveryMetadataFailsBeforeLibraryLoad)  // NOLINT
{
  const std::string config = R"(
contact_manager_plugins:
  search_paths: /tmp/plugins
  search_libraries: [library_that_does_not_exist]
)";

  tesseract::common::GeneralResourceLocator locator;
  try
  {
    ContactManagersPluginFactory factory(config, locator);
    FAIL() << "Expected plugin discovery validation to fail";
  }
  catch (const std::runtime_error& error)
  {
    const std::string message = error.what();
    EXPECT_NE(message.find("Plugin discovery validation failed"), std::string::npos);
    EXPECT_NE(message.find("search_paths"), std::string::npos);
  }
}

TEST(TesseractContactManagersFactoryUnit, PluginFactorAPIUnit)  // NOLINT
{
  ContactManagersPluginFactory factory;
  EXPECT_FALSE(factory.getSearchPaths().empty());
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_FALSE(factory.getSearchLibraries().empty());
  EXPECT_EQ(factory.getSearchLibraries().size(), 2);
  EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 0);
  EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 0);
  EXPECT_ANY_THROW(factory.getDefaultDiscreteContactManagerPlugin());    // NOLINT
  EXPECT_ANY_THROW(factory.getDefaultContinuousContactManagerPlugin());  // NOLINT
  EXPECT_FALSE(factory.hasDiscreteContactManagerPlugins());
  EXPECT_FALSE(factory.hasContinuousContactManagerPlugins());

  factory.addSearchPath("/usr/local/lib");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 2);

  factory.addSearchLibrary("tesseract_collision");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 3);

  {
    tesseract::common::PluginInfoMap map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("NotFound") == map.end());

    tesseract::common::PluginInfo pi;
    pi.class_name = "TestDiscreteManagerFactory";
    factory.addDiscreteContactManagerPlugin("TestDiscreteManager", pi);
    EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 1);
    EXPECT_TRUE(factory.hasDiscreteContactManagerPlugins());

    map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("TestDiscreteManager") != map.end());
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "TestDiscreteManager");

    tesseract::common::PluginInfo pi2;
    pi2.class_name = "Test2DiscreteManagerFactory";
    factory.addDiscreteContactManagerPlugin("Test2DiscreteManager", pi2);
    EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 2);
    EXPECT_TRUE(factory.hasDiscreteContactManagerPlugins());

    map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2DiscreteManager") != map.end());
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "Test2DiscreteManager");
    factory.setDefaultDiscreteContactManagerPlugin("TestDiscreteManager");
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "TestDiscreteManager");

    factory.removeDiscreteContactManagerPlugin("TestDiscreteManager");
    map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2DiscreteManager") != map.end());
    EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 1);
    // The default was removed so it should now be the first solver
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "Test2DiscreteManager");

    // Failures
    EXPECT_ANY_THROW(factory.removeDiscreteContactManagerPlugin("DoesNotExist"));      // NOLINT
    EXPECT_ANY_THROW(factory.setDefaultDiscreteContactManagerPlugin("DoesNotExist"));  // NOLINT
  }

  {
    tesseract::common::PluginInfoMap map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("NotFound") == map.end());

    tesseract::common::PluginInfo pi;
    pi.class_name = "TestContinuousManagerFactory";
    factory.addContinuousContactManagerPlugin("TestContinuousManager", pi);
    EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 1);
    EXPECT_TRUE(factory.hasContinuousContactManagerPlugins());

    map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("TestContinuousManager") != map.end());
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "TestContinuousManager");

    tesseract::common::PluginInfo pi2;
    pi2.class_name = "Test2ContinuousManagerFactory";
    factory.addContinuousContactManagerPlugin("Test2ContinuousManager", pi2);
    EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 2);
    EXPECT_TRUE(factory.hasContinuousContactManagerPlugins());

    map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2ContinuousManager") != map.end());
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "Test2ContinuousManager");
    factory.setDefaultContinuousContactManagerPlugin("TestContinuousManager");
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "TestContinuousManager");

    factory.removeContinuousContactManagerPlugin("TestContinuousManager");

    map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2ContinuousManager") != map.end());
    EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 1);
    // The default was removed so it should now be the first solver
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "Test2ContinuousManager");

    // Failures
    EXPECT_ANY_THROW(factory.removeContinuousContactManagerPlugin("DoesNotExist"));      // NOLINT
    EXPECT_ANY_THROW(factory.setDefaultContinuousContactManagerPlugin("DoesNotExist"));  // NOLINT
  }
}

TEST(TesseractContactManagersFactoryUnit, LoadOnlyDiscretePluginTest)  // NOLINT
{
  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories
                              - tesseract_collision_fcl_factories
                            discrete_plugins:
                              default: BulletDiscreteBVHManager
                              plugins:
                                BulletDiscreteBVHManager:
                                  class: BulletDiscreteBVHManagerFactory
                                BulletDiscreteSimpleManager:
                                  class: BulletDiscreteSimpleManagerFactory
                                FCLDiscreteBVHManager:
                                  class: FCLDiscreteBVHManagerFactory)";

  tesseract::common::GeneralResourceLocator locator;
  ContactManagersPluginFactory factory(config, locator);
  YAML::Node plugin_config = tesseract::common::loadYamlString(config, locator);

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& discrete_plugins = plugin_info["discrete_plugins"]["plugins"];

  {
    std::vector<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::vector<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(discrete_plugins.size(), 3);
  for (auto cm_it = discrete_plugins.begin(); cm_it != discrete_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }
}

TEST(TesseractContactManagersFactoryUnit, LoadOnlyContinuousPluginTest)  // NOLINT
{
  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories
                              - tesseract_collision_fcl_factories
                            continuous_plugins:
                              default: BulletCastBVHManager
                              plugins:
                                BulletCastBVHManager:
                                  class: BulletCastBVHManagerFactory
                                BulletCastSimpleManager:
                                  class: BulletCastSimpleManagerFactory)";

  tesseract::common::GeneralResourceLocator locator;
  ContactManagersPluginFactory factory(config, locator);
  YAML::Node plugin_config = tesseract::common::loadYamlString(config, locator);

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& continuous_plugins = plugin_info["continuous_plugins"]["plugins"];

  {
    std::vector<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::vector<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(continuous_plugins.size(), 2);
  for (auto cm_it = continuous_plugins.begin(); cm_it != continuous_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }
}

namespace
{
/** @brief A factory over the Bullet (and FCL) factory libraries listed in contact_manager_plugins.yaml */
std::unique_ptr<ContactManagersPluginFactory> loadTestFactory()
{
  const std::filesystem::path config_path =
      std::filesystem::path(__FILE__).parent_path() / "contact_manager_plugins.yaml";
  tesseract::common::GeneralResourceLocator locator;
  return std::make_unique<ContactManagersPluginFactory>(config_path, locator);
}

tesseract::common::PluginInfo makePluginInfo(const std::string& class_name, const std::string& config)
{
  tesseract::common::PluginInfo plugin_info;
  plugin_info.class_name = class_name;
  plugin_info.config = YAML::Load(config);
  return plugin_info;
}

/** @brief Run @p create, which must throw; return the message, or "" (and fail) when it does not throw */
template <typename CreateFn>
std::string creationErrorMessage(const CreateFn& create)
{
  try
  {
    auto manager = create();
    ADD_FAILURE() << "Expected creation to throw, got " << (manager == nullptr ? "nullptr" : "a manager");
  }
  catch (const ContactManagerCreationError& error)
  {
    return error.what();
  }
  return {};
}

void expectMentions(const std::string& message, const std::vector<std::string>& needles)
{
  for (const auto& needle : needles)
    EXPECT_NE(message.find(needle), std::string::npos) << "'" << needle << "' missing from: " << message;
}
}  // namespace

// A factory that exists but rejects its config is a configuration error: it must reach the caller.
TEST(TesseractContactManagersFactoryUnit, FactoryRejectingConfigValueThrows)  // NOLINT
{
  auto factory = loadTestFactory();
  const std::string bad_value = "max_persistent_manifold_pool_size: 99999";  // schema maximum is 4096

  auto discrete_info = makePluginInfo("BulletDiscreteBVHManagerFactory", bad_value);
  // Twice: the first call loads the factory, the second takes the cached one.
  for (int i = 0; i < 2; ++i)
  {
    const std::string message =
        creationErrorMessage([&] { return factory->createDiscreteContactManager("BadDiscrete", discrete_info); });
    expectMentions(message, { "BadDiscrete", "BulletDiscreteBVHManagerFactory", "max_persistent_manifold_pool_size" });
  }

  auto continuous_info = makePluginInfo("BulletCastBVHManagerFactory", bad_value);
  for (int i = 0; i < 2; ++i)
  {
    const std::string message =
        creationErrorMessage([&] { return factory->createContinuousContactManager("BadCast", continuous_info); });
    expectMentions(message, { "BadCast", "BulletCastBVHManagerFactory", "max_persistent_manifold_pool_size" });
  }

  // Through the registered-name path as well
  factory->addDiscreteContactManagerPlugin("BadDiscrete", discrete_info);
  expectMentions(creationErrorMessage([&] { return factory->createDiscreteContactManager("BadDiscrete"); }),
                 { "BadDiscrete", "BulletDiscreteBVHManagerFactory" });
  factory->addContinuousContactManagerPlugin("BadCast", continuous_info);
  expectMentions(creationErrorMessage([&] { return factory->createContinuousContactManager("BadCast"); }),
                 { "BadCast", "BulletCastBVHManagerFactory" });
}

TEST(TesseractContactManagersFactoryUnit, FactoryRejectingUnknownKeyThrows)  // NOLINT
{
  auto factory = loadTestFactory();
  const std::string unknown_key = "not_a_bullet_key: 1";

  auto discrete_info = makePluginInfo("BulletDiscreteSimpleManagerFactory", unknown_key);
  expectMentions(
      creationErrorMessage([&] { return factory->createDiscreteContactManager("UnknownKey", discrete_info); }),
      { "UnknownKey", "BulletDiscreteSimpleManagerFactory", "not_a_bullet_key" });

  auto continuous_info = makePluginInfo("BulletCastSimpleManagerFactory", unknown_key);
  expectMentions(
      creationErrorMessage([&] { return factory->createContinuousContactManager("UnknownKey", continuous_info); }),
      { "UnknownKey", "BulletCastSimpleManagerFactory", "not_a_bullet_key" });
}

// A factory that is not found (plugin absent) is not an error at this layer: nullptr, no throw.
TEST(TesseractContactManagersFactoryUnit, AbsentFactoryReturnsNullWithoutThrow)  // NOLINT
{
  auto factory = loadTestFactory();
  auto absent = makePluginInfo("DoesNotExistFactory", "{}");

  DiscreteContactManager::UPtr discrete;
  EXPECT_NO_THROW(discrete = factory->createDiscreteContactManager("Absent", absent));  // NOLINT
  EXPECT_TRUE(discrete == nullptr);

  ContinuousContactManager::UPtr continuous;
  EXPECT_NO_THROW(continuous = factory->createContinuousContactManager("Absent", absent));  // NOLINT
  EXPECT_TRUE(continuous == nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
