#include <iostream>
#include <stdexcept>

#include <fields2cover.h>

int main() {
  F2CRobot robot(0.4, 0.4);
  robot.setMinRadius(1.0);
  robot.linear_curv_change = 0.3;

  F2CCell cell;
  cell.importFromWkt("POLYGON((0 0, 0 8, 6 8, 6 0, 0 0))");

  f2c::sg::BruteForce swath_generator;
  f2c::obj::SwathLength swath_objective;
  F2CSwaths swaths = swath_generator.generateBestSwaths(swath_objective, robot.op_width, cell);
  f2c::rp::SnakeOrder route_generator;
  swaths = route_generator.genSortedSwaths(swaths);

  f2c::pp::PathPlanning path_planner;
  f2c::pp::DubinsCurves dubins;
  path_planner.turn_point_dist = 0.1;
  F2CPath path = path_planner.searchBestPath(robot, swaths, dubins);

  const double area = cell.getArea();
  const size_t states = path.size();
  const double length = path.length();

  std::cout << "Fields2Cover smoke test" << std::endl;
  std::cout << "Cell area: " << area << std::endl;
  std::cout << "Path states: " << states << std::endl;
  std::cout << "Path length: " << length << std::endl;

  if (states == 0 || length <= 0.0) {
    throw std::runtime_error("Fields2Cover returned an empty path");
  }

  return 0;
}
