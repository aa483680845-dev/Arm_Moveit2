#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    /********************************************************************************/
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);


    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    rclcpp::sleep_for(std::chrono::seconds(1));
    
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(start_pose);

    geometry_msgs::msg::Pose p1 = start_pose;
    p1.position.z += 0.05;
    waypoints.push_back(p1);

    geometry_msgs::msg::Pose p2 = p1;
    p2.position.x += 0.1;
    waypoints.push_back(p2);

    geometry_msgs::msg::Pose p3 = p2;
    p3.position.z -= 0.05;
    p3.position.x -= 0.1;
    waypoints.push_back(p3);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;       // 1cm 插值
    const double jump_threshold = 0.0;  // 仿真环境：禁用关节跳变检测

    double fraction = move_group.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory);

    RCLCPP_INFO(LOGGER, "Cartesian path success rate: %.2f%%", fraction * 100.0);

    if (fraction > 0.6)
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory = trajectory;
        rclcpp::sleep_for(std::chrono::milliseconds(500)); // 等待 RViz 接收消息
        RCLCPP_INFO(LOGGER, "Executing cartesian plan...");
        move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Cartesian path planning failed! Only %.2f%% completed.", fraction * 100.0);
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
