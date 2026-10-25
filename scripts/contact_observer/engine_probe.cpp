#include <ignition/plugin/Loader.hh>
#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>
#include <sdf/Root.hh>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace physics = ignition::physics;
using Features = physics::FeatureList<
    physics::GetEntities,
    physics::sdf::ConstructSdfWorld,
    physics::sdf::ConstructSdfModel>;

static std::string Read(const char *path)
{
  std::ifstream f(path);
  if (!f) throw std::runtime_error(std::string("cannot open ") + path);
  return std::string(std::istreambuf_iterator<char>(f), {});
}

static std::vector<std::string> Inventory(const std::string &json)
{
  auto key = json.find("\"collisions\"");
  auto begin = json.find('[', key);
  auto end = json.find(']', begin);
  if (key == std::string::npos || begin == std::string::npos || end == std::string::npos)
    throw std::runtime_error("invalid collision inventory JSON");
  std::vector<std::string> names;
  const std::regex quoted("\"([^\"]+)\"");
  const auto first = std::sregex_iterator(json.begin() + begin + 1, json.begin() + end, quoted);
  const auto last = std::sregex_iterator();
  for (auto i = first; i != last; ++i) names.push_back((*i)[1]);
  return names;
}

static void Errors(const sdf::Errors &errors, const char *what)
{
  for (const auto &e : errors)
    std::cerr << what << ": " << e.Message() << "\n";
  if (!errors.empty()) throw std::runtime_error(std::string("SDF parse failed: ") + what);
}

int main(int argc, char **argv)
{
  try
  {
    if (argc != 4)
    {
      std::cerr << "usage: contact_engine_probe robot.sdf collision-inventory.json dartsim-plugin.so\n";
      return 2;
    }
    const auto expected = Inventory(Read(argv[2]));
    if (expected.size() != 26) throw std::runtime_error("expected pinned 26 collision names");

    sdf::Root robotRoot;
    Errors(robotRoot.Load(argv[1]), "robot");
    if (!robotRoot.Model() || robotRoot.Model()->Name() != "b2w")
      throw std::runtime_error("expected root SDF model named b2w");

    sdf::Root worldRoot;
    Errors(worldRoot.LoadSdfString(
        "<sdf version='1.9'><world name='contact_engine_probe'>"
        "<gravity>0 0 -9.8</gravity><physics name='default' type='ode'>"
        "<max_step_size>0.0025</max_step_size></physics></world></sdf>"), "probe world");
    if (worldRoot.WorldCount() != 1) throw std::runtime_error("probe world parse count mismatch");

    ignition::plugin::Loader loader;
    const auto loaded = loader.LoadLib(argv[3]);
    if (loaded.empty()) throw std::runtime_error("DART plugin library loaded no plugins");
    const auto candidates = physics::FindFeatures3d<Features>::From(loader);
    if (candidates.empty())
    {
      std::cerr << "No DART plugin provides requested features; loaded plugins:";
      for (const auto &name : loaded) std::cerr << " " << name;
      std::cerr << "\n";
      return 3;
    }

    const auto pluginName = *candidates.begin();
    auto plugin = loader.Instantiate(pluginName);
    auto engine = physics::RequestEngine3d<Features>::From(plugin);
    if (!engine) throw std::runtime_error("RequestEngine3d rejected feature-compatible DART plugin");
    auto world = engine->ConstructWorld(*worldRoot.WorldByIndex(0));
    if (!world) throw std::runtime_error("DART failed to construct empty probe world");
    auto model = world->ConstructModel(*robotRoot.Model());
    if (!model) throw std::runtime_error("DART failed to construct b2w model from converted SDF");

    std::set<std::string> actual;
    std::vector<std::pair<std::string, bool>> records;
    for (std::size_t li = 0; li < model->GetLinkCount(); ++li)
    {
      auto link = model->GetLink(li);
      if (!link) throw std::runtime_error("DART returned null link handle");
      for (std::size_t si = 0; si < link->GetShapeCount(); ++si)
      {
        auto shape = link->GetShape(si);
        const bool valid = static_cast<bool>(shape);
        if (!valid) throw std::runtime_error("DART returned null Shape handle");
        auto name = link->GetName() + "::" + shape->GetName();
        actual.insert(name);
        records.emplace_back(name, valid);
      }
    }

    std::set<std::string> wanted(expected.begin(), expected.end());
    std::vector<std::string> missing, unexpected;
    for (const auto &name : wanted) if (!actual.count(name)) missing.push_back(name);
    for (const auto &name : actual) if (!wanted.count(name)) unexpected.push_back(name);
    const bool pass = records.size() == 26 && actual == wanted;

    std::cout << "{\n  \"result\":\"" << (pass ? "PASS" : "FAIL") << "\",\n"
              << "  \"engine_plugin\":\"" << pluginName << "\",\n"
              << "  \"engine_name\":\"" << engine->GetName() << "\",\n"
              << "  \"world\":\"contact_engine_probe\",\n"
              << "  \"model\":\"" << model->GetName() << "\",\n"
              << "  \"expected_count\":" << expected.size() << ",\n"
              << "  \"constructed_shape_handles\":" << records.size() << ",\n"
              << "  \"unique_constructed_names\":" << actual.size() << ",\n"
              << "  \"shapes\":[\n";
    for (std::size_t i = 0; i < records.size(); ++i)
      std::cout << "    {\"name\":\"" << records[i].first << "\",\"nonnull_shape_handle\":"
                << (records[i].second ? "true" : "false") << "}"
                << (i + 1 == records.size() ? "\n" : ",\n");
    std::cout << "  ],\n  \"missing\":[";
    for (std::size_t i = 0; i < missing.size(); ++i)
      std::cout << (i ? "," : "") << "\"" << missing[i] << "\"";
    std::cout << "],\n  \"unexpected\":[";
    for (std::size_t i = 0; i < unexpected.size(); ++i)
      std::cout << (i ? "," : "") << "\"" << unexpected[i] << "\"";
    std::cout << "],\n  \"stepped_physics\":false,\n"
              << "  \"gazebo_world_launched\":false,\n"
              << "  \"motion_or_contacts_exercised\":false\n}\n";
    return pass ? 0 : 1;
  }
  catch (const std::exception &e)
  {
    std::cerr << "contact_engine_probe: " << e.what() << "\n";
    return 1;
  }
}
