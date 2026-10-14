// Frozen single-pose diagnostic using installed Nav2 code. No ROS init or node.
// stdin: width height resolution origin_x origin_y pose_x pose_y yaw N [x y]*N
// argv[1]: exact raw uint8 row-major costmap; origin orientation must be identity.
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_critics/line_iterator.hpp"
#include "dwb_core/exceptions.hpp"

class Base : public dwb_critics::BaseObstacleCritic
{
public:
  explicit Base(nav2_costmap_2d::Costmap2D & map) {costmap_ = &map;}
};

class Footprint : public dwb_critics::ObstacleFootprintCritic
{
public:
  Footprint(nav2_costmap_2d::Costmap2D & map, const dwb_critics::Footprint & poly)
  {costmap_ = &map; footprint_spec_ = poly;}

  // Trace only; legality comes from unmodified installed scorePose above.
  // Use installed transform, worldToMap, LineIterator AND pointCost in the
  // upstream edge order. Verify first-cell trace agrees with actual exception.
  std::vector<int> firstIllegal(const geometry_msgs::msg::Pose2D & pose)
  {
    unsigned int cx, cy;
    if (!costmap_->worldToMap(pose.x, pose.y, cx, cy)) {return {};}
    const auto poly = dwb_critics::getOrientedFootprint(pose, footprint_spec_);
    for (size_t i = 0; i < poly.size(); ++i) {
      const auto & a = poly[i]; const auto & b = poly[(i + 1) % poly.size()];
      unsigned int x0, y0, x1, y1;
      if (!costmap_->worldToMap(a.x, a.y, x0, y0) ||
        !costmap_->worldToMap(b.x, b.y, x1, y1)) {return {};}
      for (dwb_critics::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
        try {pointCost(line.getX(), line.getY());}
        catch (const dwb_core::IllegalTrajectoryException &) {
          return {line.getX(), line.getY(), costmap_->getCost(line.getX(), line.getY())};
        }
      }
    }
    return {};
  }
};

int main(int argc, char ** argv)
{
  try {
    unsigned int width, height, n; double res, ox, oy;
    geometry_msgs::msg::Pose2D pose;
    if (argc != 2 || !(std::cin >> width >> height >> res >> ox >> oy >>
      pose.x >> pose.y >> pose.theta >> n)) {throw std::runtime_error("Invalid input");}
    if (!width || !height || width > 100000 || height > 100000 ||
      static_cast<uint64_t>(width) * height > 100000000 || n < 3 || n > 1000 ||
      !std::isfinite(res) || res <= 0 || !std::isfinite(ox) || !std::isfinite(oy) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta))
    {throw std::runtime_error("Invalid geometry");}
    dwb_critics::Footprint poly(n);
    for (auto & p : poly) {
      if (!(std::cin >> p.x >> p.y) || !std::isfinite(p.x) || !std::isfinite(p.y))
      {throw std::runtime_error("Invalid footprint");}
    }
    std::ifstream file(argv[1], std::ios::binary);
    std::vector<unsigned char> raw((std::istreambuf_iterator<char>(file)), {});
    if (raw.size() != static_cast<size_t>(width) * height)
    {throw std::runtime_error("Raw size mismatch");}
    nav2_costmap_2d::Costmap2D map(width, height, res, ox, oy, 0);
    std::copy(raw.begin(), raw.end(), map.getCharMap());
    Base base(map); Footprint footprint(map, poly);
    bool base_ok = true, foot_ok = true; double bs = 0, fs = 0;
    try {bs = base.scorePose(pose);} catch (const dwb_core::IllegalTrajectoryException &) {base_ok = false;}
    try {fs = footprint.scorePose(pose);} catch (const dwb_core::IllegalTrajectoryException &) {foot_ok = false;}
    const auto bad = footprint.firstIllegal(pose);
    if (foot_ok && !bad.empty()) {throw std::runtime_error("Trace disagrees with installed critic");}
    unsigned int cx, cy;
    std::cout << std::boolalpha << "{\"base_legal\":" << base_ok <<
      ",\"footprint_legal\":" << foot_ok << ",\"base_score\":";
    if (base_ok) {std::cout << bs;} else {std::cout << "null";}
    std::cout << ",\"footprint_score\":";
    if (foot_ok) {std::cout << fs;} else {std::cout << "null";}
    std::cout << ",\"center_cell\":";
    if (map.worldToMap(pose.x, pose.y, cx, cy)) {
      std::cout << '[' << cx << ',' << cy << ',' << static_cast<int>(map.getCost(cx, cy)) << ']';
    } else {std::cout << "null";}
    std::cout << ",\"first_illegal_cell\":";
    if (bad.empty()) {std::cout << "null";} else {
      std::cout << '[' << bad[0] << ',' << bad[1] << ',' << bad[2] << ']';
    }
    std::cout << "}\n";
    return 0;
  } catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 2;}
}
