// Isolated installed-library search/smoothing harness: no ROS init, node or DDS.
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <chrono>
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"

int main(int argc, char **argv) {
  if (argc!=12) return 2;
  unsigned int w=std::stoul(argv[2]),h=std::stoul(argv[3]);
  double resolution=std::stod(argv[4]),ox=std::stod(argv[5]),oy=std::stod(argv[6]);
  nav2_costmap_2d::Costmap2D costmap(w,h,resolution,ox,oy,0);
  std::ifstream in(argv[1],std::ios::binary);
  in.read(reinterpret_cast<char *>(costmap.getCharMap()),w*h);
  if (!in || static_cast<unsigned int>(in.gcount())!=w*h) return 3;
  nav2_smac_planner::GridCollisionChecker checker(&costmap,1,nullptr);
  checker.setFootprint({},true,0.0);
  nav2_smac_planner::SearchInfo info{};info.cost_penalty=1.0;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> astar(nav2_smac_planner::MotionModel::TWOD,info);
  int maximum=1000000,iterations=0;
  astar.initialize(false,maximum,1000,2.0,0.0,1);
  astar.setCollisionChecker(&checker);
  unsigned int sx,sy,gx,gy;
  if (!costmap.worldToMap(std::stod(argv[7]),std::stod(argv[8]),sx,sy) ||
      !costmap.worldToMap(std::stod(argv[9]),std::stod(argv[10]),gx,gy)) return 4;
  astar.setStart(sx,sy,0);astar.setGoal(gx,gy,0);
  nav2_smac_planner::Node2D::CoordinateVector coords;
  const auto began=std::chrono::steady_clock::now();
  bool found=false;std::string error;
  try {found=astar.createPath(coords,iterations,0.3/resolution);}
  catch(const std::exception &e){error=e.what();}
  nav_msgs::msg::Path path;
  for(auto it=coords.rbegin();it!=coords.rend();++it) {
    geometry_msgs::msg::PoseStamped p;
    costmap.mapToWorld(it->x,it->y,p.pose.position.x,p.pose.position.y);
    p.pose.orientation.w=1.0;path.poses.push_back(p);
  }
  nav2_smac_planner::SmootherParams params;
  params.tolerance_=1e-10;params.max_its_=1000;params.w_data_=0.2;params.w_smooth_=0.3;
  params.holonomic_=true;params.do_refinement_=true;
  nav2_smac_planner::Smoother smoother(params);smoother.initialize(1e-50);
  double remaining=2.0-std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
  bool smooth=found && smoother.smooth(path,&costmap,remaining);
  if (!path.poses.empty()) {
    path.poses.back().pose.orientation.z=std::sin(std::stod(argv[11])/2);
    path.poses.back().pose.orientation.w=std::cos(std::stod(argv[11])/2);
  }
  std::cout<<std::setprecision(17)<<"{\"found\":"<<(found?"true":"false")
           <<",\"iterations\":"<<iterations<<",\"smoother_return\":"<<(smooth?"true":"false")
           <<",\"exception\":"<<(error.empty()?"false":"true")<<",\"path\":[";
  bool first=true;
  for(const auto &p:path.poses){
    if(!first)std::cout<<",";first=false;
    const auto &q=p.pose.orientation;
    double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    std::cout<<"["<<p.pose.position.x<<","<<p.pose.position.y<<","<<yaw<<"]";
  }
  std::cout<<"]}"<<std::endl;
  return 0;
}
