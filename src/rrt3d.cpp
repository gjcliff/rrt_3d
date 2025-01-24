#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

struct RRT_Node {
public:
  RRT_Node() {}
  RRT_Node(geometry_msgs::msg::Point coord, std::shared_ptr<RRT_Node> parent)
    : dist(0), coord(coord), parent(parent)
  {
  }
  RRT_Node(int dist, geometry_msgs::msg::Point coord,
           std::shared_ptr<RRT_Node> parent)
    : dist(dist), coord(coord), parent(parent)
  {
  }
  int dist;
  geometry_msgs::msg::Point coord;
  std::shared_ptr<RRT_Node> parent;
};

class RRT3D : public rclcpp::Node {
public:
  RRT3D()
    : Node("rrt3d"), found_goal_(false), rd(), gen(rd()), node_count_(0),
      arrow_count_(0)
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
      std::make_shared<RRT_Node>(start_coord_, nullptr);
    nodes.push_back(start);

    // initialize publishers, subscribers, etc.
    node_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("nodes", 10);
    arrow_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("arrows", 10);
    timer_ = create_wall_timer(500ms, std::bind(&RRT3D::run, this));
  }

  double euclidean_distance(geometry_msgs::msg::Point start,
                            geometry_msgs::msg::Point end)
  {
    return std::sqrt(std::pow(start.x - end.x, 2) +
                     std::pow(start.y - end.y, 2) +
                     std::pow(start.z - end.z, 2));
  }

  std::shared_ptr<visualization_msgs::msg::Marker>
  create_sphere_marker(std::shared_ptr<RRT_Node> node)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "world";
    marker.id = node_count_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = node->coord.x;
    marker.pose.position.y = node->coord.y;
    marker.pose.position.z = node->coord.z;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    return std::make_shared<visualization_msgs::msg::Marker>(marker);
  }

  std::shared_ptr<visualization_msgs::msg::Marker>
  create_arrow_marker(std::shared_ptr<RRT_Node> start,
                      std::shared_ptr<RRT_Node> end)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "world";
    marker.id = arrow_count_++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points = {start->coord, end->coord};
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    return std::make_shared<visualization_msgs::msg::Marker>(marker);
  }

  void run()
  {
    if (!found_goal_) {
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

      std::shared_ptr<RRT_Node> new_node = std::make_shared<RRT_Node>(
        closest_node->dist + 1, new_point, closest_node);
      nodes.push_back(std::make_shared<RRT_Node>(closest_node->dist + 1,
                                                 new_point, closest_node));
      node_markers_.markers.push_back(*create_sphere_marker(new_node));
      arrow_markers_.markers.push_back(
        *create_arrow_marker(closest_node, new_node));

      node_publisher_->publish(node_markers_);
      arrow_publisher_->publish(arrow_markers_);

      if (euclidean_distance(new_point, goal_coord_) < 0.2) {
        found_goal_ = true;
        RCLCPP_INFO(get_logger(), "Found the goal!");
      }
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    node_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    arrow_publisher_;

  visualization_msgs::msg::MarkerArray node_markers_;
  visualization_msgs::msg::MarkerArray arrow_markers_;
  geometry_msgs::msg::Point world_bounds_;
  geometry_msgs::msg::Point start_coord_;
  geometry_msgs::msg::Point goal_coord_;
  double step_size_;
  bool found_goal_;

  std::random_device rd;
  std::mt19937 gen;

  std::vector<std::shared_ptr<RRT_Node>> nodes; // start with this, can optimize

  int node_count_;
  int arrow_count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRT3D>());
  rclcpp::shutdown();
  return 0;
}
