#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

struct RRT_Node {
public:
  RRT_Node() {}
  RRT_Node(geometry_msgs::msg::Point coord, std::shared_ptr<RRT_Node> parent)
    : coord(coord), parent(parent)
  {
  }
  geometry_msgs::msg::Point coord;
  std::shared_ptr<RRT_Node> parent;
};

class RRT3D : public rclcpp::Node {
public:
  RRT3D() : Node("rrt3d"), found_goal_(false), rd(), gen(rd())
  {
    // declare parameters
    declare_parameter<std::vector<double>>("start_coord", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("goal_coord", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("world_size", {0.0, 0.0, 0.0});
    declare_parameter("step_size", 0.1);

    // get parameters
    std::vector<double> start_coord =
      get_parameter("start_coord").as_double_array();
    start_coord_.x = start_coord.at(0);
    start_coord_.y = start_coord.at(1);
    start_coord_.z = start_coord.at(2);

    std::vector<double> goal_coord =
      get_parameter("goal_coord").as_double_array();
    goal_coord_.x = goal_coord.at(0);
    goal_coord_.y = goal_coord.at(1);
    goal_coord_.z = goal_coord.at(2);

    std::vector<double> world_size =
      get_parameter("world_size").as_double_array();
    world_bounds_.x = world_size.at(0);
    world_bounds_.y = world_size.at(1);
    world_bounds_.z = world_size.at(2);

    step_size_ = get_parameter("step_size").as_double();

    // initialize some variables
    std::shared_ptr<RRT_Node> start =
      std::make_shared<RRT_Node>(start_coord, nullptr);
    nodes.push_back(start);

    // initialize publishers, subscribers, etc.
    publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = create_wall_timer(500ms, std::bind(&RRT3D::run, this));
  }

  double euclidean_distance(geometry_msgs::msg::Point start,
                            geometry_msgs::msg::Point end)
  {
    return std::sqrt(std::pow(start.x - end.x, 2) +
                     std::pow(start.y - end.y, 2) +
                     std::pow(start.z - end.z, 2));
  }

  void run()
  {
    if (!found_goal_){
      // pick a random point in the workspace
      geometry_msgs::msg::Point rpoint;
      std::uniform_real_distribution<> ux(-world_bounds_.x, world_bounds_.x);
      std::uniform_real_distribution<> uy(-world_bounds_.y, world_bounds_.y);
      std::uniform_real_distribution<> uz(-world_bounds_.z, world_bounds_.z);
      rpoint.x = ux(gen);
      rpoint.y = uy(gen);
      rpoint.z = uz(gen);

      // find the closest node
      double min_dist = std::numeric_limits<double>::max();
      std::shared_ptr<RRT_Node> closest_node;
      for (const auto &node : nodes) {
        if (double dist = euclidean_distance(node->coord, rpoint) < min_dist) {
          min_dist = dist;
          closest_node = node;
        }
      }

      // add a new node in the direction of the random node stemming from the
      // closest node
      // create unit vector
      geometry_msgs::msg::Vector3 unit_vector;
      unit_vector.x = (rpoint.x - closest_node->coord.x) /
        euclidean_distance(rpoint, closest_node->coord);
      unit_vector.y = (rpoint.y - closest_node->coord.y) /
        euclidean_distance(rpoint, closest_node->coord);
      unit_vector.z = (rpoint.z - closest_node->coord.z) /
        euclidean_distance(rpoint, closest_node->coord);

      geometry_msgs::msg::Point new_point;
      new_point.x = closest_node->coord.x + unit_vector.x;
      new_point.y = closest_node->coord.y + unit_vector.y;
      new_point.z = closest_node->coord.z + unit_vector.z;

      nodes.push_back(std::make_shared<RRT_Node>(new_point, closest_node));

      if (euclidean_distance(new_point, goal_coord_) < 0.2) {
        found_goal_ = true;
        RCLCPP_INFO(get_logger(), "Found the goal!");
      }
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  geometry_msgs::msg::Point world_bounds_;
  geometry_msgs::msg::Point start_coord_;
  geometry_msgs::msg::Point goal_coord_;
  double step_size_;
  bool found_goal_;

  std::random_device rd;
  std::mt19937 gen;

  std::vector<std::shared_ptr<RRT_Node>> nodes; // start with this, can optimize

  // 847-491-7227
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRT3D>());
  rclcpp::shutdown();
  return 0;
}
