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

    // 用于在 RViz 中显示规划路径的发布器
    auto display_publisher = move_group_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", 1);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    rclcpp::sleep_for(std::chrono::seconds(1));

    /********************************************************************************/
    // 设置目标位姿（IK 由 move_group 服务端完成）
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.2;
    target_pose.position.y = 0.1;
    target_pose.position.z = 0.3;
    tf2::Quaternion q;
    q.setRPY(0, M_PI_2, 0);
    target_pose.orientation = tf2::toMsg(q);

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 1 (pose goal): %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        // 发布轨迹到 RViz 显示
        moveit_msgs::msg::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = my_plan.start_state;
        display_trajectory.trajectory.push_back(my_plan.trajectory);
        display_publisher->publish(display_trajectory);
        RCLCPP_INFO(LOGGER, "Trajectory published to RViz, waiting 5s before execution...");

        rclcpp::sleep_for(std::chrono::seconds(5));

        // 执行轨迹
        RCLCPP_INFO(LOGGER, "Executing plan...");
        move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
