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

struct RRT_Arrow;

struct RRT_Node {
public:
  RRT_Node()
    : index(0), coord(geometry_msgs::msg::Point()), parent(nullptr),
      arrow(nullptr)
  {}
  RRT_Node(int index, geometry_msgs::msg::Point coord,
           std::shared_ptr<RRT_Node> parent)
    : index(index), coord(coord), parent(parent), arrow(nullptr)
  {}
  RRT_Node(int index, geometry_msgs::msg::Point coord,
           std::shared_ptr<RRT_Node> parent, std::shared_ptr<RRT_Arrow> arrow)
    : index(index), coord(coord), parent(parent), arrow(arrow)
  {}

  int index;
  geometry_msgs::msg::Point coord;
  std::shared_ptr<RRT_Node> parent;
  std::shared_ptr<RRT_Arrow> arrow;
};

struct RRT_Arrow {
public:
  RRT_Arrow() {}
  RRT_Arrow(std::shared_ptr<RRT_Node> start, std::shared_ptr<RRT_Node> end)
    : index(0), start(start), end(end)
  {}
  RRT_Arrow(int index, std::shared_ptr<RRT_Node> start,
            std::shared_ptr<RRT_Node> end)
    : index(index), start(start), end(end)
  {}

  int index;
  std::shared_ptr<RRT_Node> start;
  std::shared_ptr<RRT_Node> end;
};

class RRT3D : public rclcpp::Node {
public:
  RRT3D()
    : Node("rrt3d"), found_goal_(false), rd(), gen(rd()), node_count_(0),
      arrow_count_(0)
  {
    // declare parameters
    declare_parameter<std::vector<double>>("world_bounds", {10.0, 10.0, 10.0});
    declare_parameter<std::vector<double>>("start_coord",
                                           std::vector<double>{});
    declare_parameter<std::vector<double>>("goal_coord", std::vector<double>{});
    declare_parameter("step_size", 1.0);

    // get parameters
    std::vector<double> world_bounds =
      get_parameter("world_bounds").as_double_array();
    world_bounds_.x = world_bounds.at(0);
    world_bounds_.y = world_bounds.at(1);
    world_bounds_.z = world_bounds.at(2);

    std::vector<double> start_coord =
      get_parameter("start_coord").as_double_array();
    if (start_coord.size() < 3) {
      start_coord_ = get_random_point();
    } else {
      start_coord_.x = start_coord.at(0);
      start_coord_.y = start_coord.at(1);
      start_coord_.z = start_coord.at(2);
    }

    std::vector<double> goal_coord =
      get_parameter("goal_coord").as_double_array();
    if (goal_coord.size() < 3) {
      goal_coord_ = get_random_point();
    } else {
      goal_coord_.x = goal_coord.at(0);
      goal_coord_.y = goal_coord.at(1);
      goal_coord_.z = goal_coord.at(2);
    }

    step_size_ = get_parameter("step_size").as_double();

    // initialize some variables
    std::shared_ptr<RRT_Node> goal =
      std::make_shared<RRT_Node>(node_count_++, goal_coord_, nullptr);
    std::shared_ptr<RRT_Node> start =
      std::make_shared<RRT_Node>(node_count_++, start_coord_, nullptr);
    nodes.push_back(start);
    visualization_msgs::msg::Marker start_marker = *create_sphere_marker(start);
    visualization_msgs::msg::Marker goal_marker = *create_sphere_marker(goal);
    start_marker.color.r = 0.0;
    start_marker.color.b = 1.0;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    node_markers_.markers.push_back(goal_marker);
    node_markers_.markers.push_back(start_marker);

    // initialize publishers, subscribers, etc.
    node_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("nodes", 10);
    arrow_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("arrows", 10);
    timer_ = create_wall_timer(10ms, std::bind(&RRT3D::run, this));
  }

  // \brief Sample a random point that's within the bounds of the world
  // \return The random point
  geometry_msgs::msg::Point get_random_point()
  {
    geometry_msgs::msg::Point rpoint;
    std::uniform_real_distribution<> ux(-world_bounds_.x, world_bounds_.x);
    std::uniform_real_distribution<> uy(-world_bounds_.y, world_bounds_.y);
    std::uniform_real_distribution<> uz(-world_bounds_.z, world_bounds_.z);
    rpoint.x = ux(gen);
    rpoint.y = uy(gen);
    rpoint.z = uz(gen);

    return rpoint;
  }

  // \brief Calculate the euclidean distance between two points
  // \param start The start point
  // \param end The end point
  // \return The euclidean distance
  double euclidean_distance(geometry_msgs::msg::Point start,
                            geometry_msgs::msg::Point end)
  {
    return std::sqrt(std::pow(start.x - end.x, 2) +
                     std::pow(start.y - end.y, 2) +
                     std::pow(start.z - end.z, 2));
  }

  // \brief Create a sphere marker object based on an RRT node
  // \param node The RRT node we want to make a marker for
  // \return A shared pointer to the sphere marker object for the RRT node
  std::shared_ptr<visualization_msgs::msg::Marker>
  create_sphere_marker(std::shared_ptr<RRT_Node> node)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "world";
    marker.id = node->index;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = node->coord.x;
    marker.pose.position.y = node->coord.y;
    marker.pose.position.z = node->coord.z;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    return std::make_shared<visualization_msgs::msg::Marker>(marker);
  }

  // \brief Create an arrow marker between two RRT nodes
  // \param arrow The arrow object to create the marker from, which stores its
  // own start and end point.
  // \return A shared pointer to an arrow marker
  std::shared_ptr<visualization_msgs::msg::Marker>
  create_arrow_marker(std::shared_ptr<RRT_Arrow> arrow)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "world";
    marker.id = arrow->index;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points = {arrow->start->coord, arrow->end->coord};
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    return std::make_shared<visualization_msgs::msg::Marker>(marker);
  }

  // \brief Turn the shortest path we found from start->goal green
  void highlight_path(std::shared_ptr<RRT_Node> node)
  {
    std::shared_ptr<RRT_Node> goal_node =
      std::make_shared<RRT_Node>(node_count_++, goal_coord_, node);
    std::shared_ptr<RRT_Arrow> new_arrow =
      std::make_shared<RRT_Arrow>(arrow_count_++, node, goal_node);
    auto new_arrow_marker = create_arrow_marker(new_arrow);
    new_arrow_marker->color.r = 0.0;
    new_arrow_marker->color.g = 1.0;
    arrow_markers_.markers.push_back(*new_arrow_marker);

    while (node->parent != nullptr) {
      node_markers_.markers.at(node->index).color.r = 0.0;
      node_markers_.markers.at(node->index).color.g = 1.0;
      arrow_markers_.markers.at(node->arrow->index).color.r = 0.0;
      arrow_markers_.markers.at(node->arrow->index).color.g = 1.0;
      node = node->parent;
    }
  }

  // \brief This execute one iteration of the RRT algorithm
  void run()
  {
    if (!found_goal_) {
      // pick a random point in the workspace
      geometry_msgs::msg::Point rpoint = get_random_point();

      // find the closest node
      double min_dist = std::numeric_limits<double>::max();
      std::shared_ptr<RRT_Node> closest_node;
      for (const auto &node : nodes) {
        double dist = euclidean_distance(node->coord, rpoint);
        if (dist < min_dist) {
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
      new_point.x = closest_node->coord.x + unit_vector.x * step_size_;
      new_point.y = closest_node->coord.y + unit_vector.y * step_size_;
      new_point.z = closest_node->coord.z + unit_vector.z * step_size_;

      std::shared_ptr<RRT_Node> new_node =
        std::make_shared<RRT_Node>(node_count_++, new_point, closest_node);
      std::shared_ptr<RRT_Arrow> new_arrow =
        std::make_shared<RRT_Arrow>(arrow_count_++, closest_node, new_node);
      new_node->arrow = new_arrow;

      nodes.push_back(new_node);
      node_markers_.markers.push_back(*create_sphere_marker(new_node));
      arrow_markers_.markers.push_back(*create_arrow_marker(new_arrow));

      if (euclidean_distance(new_point, goal_coord_) < step_size_ * 2) {
        found_goal_ = true;
        highlight_path(new_node);
        RCLCPP_INFO(get_logger(), "Found the goal!");
      }

      node_publisher_->publish(node_markers_);
      arrow_publisher_->publish(arrow_markers_);
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

  std::vector<std::shared_ptr<RRT_Node>> nodes;

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
