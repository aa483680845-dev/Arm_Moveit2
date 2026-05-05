#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    move_group_node, 
    "base_link",
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group.getRobotModel()};

    moveit_visual_tools.deleteAllMarkers();         
    moveit_visual_tools.loadRemoteControl();
    
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    rclcpp::sleep_for(std::chrono::seconds(1));
    /*********************************************************************************************/

    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group.getRobotModel()->getJointModelGroup(
             PLANNING_GROUP)](auto const trajectory)

    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg,rviz_visual_tools::LIME_GREEN);
    };

    /********************************************************************************/
    // 从参数读取目标点
    auto get_double = [&](const std::string &name, double def)
    {
        return move_group_node->has_parameter(name)
                   ? move_group_node->get_parameter(name).as_double()
                   : def;
    };

    auto get_bool = [&](const std::string &name, bool def)
    {
        return move_group_node->has_parameter(name)
                   ? move_group_node->get_parameter(name).as_bool()
                   : def;
    };

    auto wp_x = get_double("target_x", 0.0);
    auto wp_y = get_double("target_y", 0.0);
    auto wp_z = get_double("target_z", 0.0);
    auto wp_roll = get_double("target_roll", 0.0);
    auto wp_pitch = get_double("target_pitch", 0.0);
    auto wp_yaw = get_double("target_yaw", 0.0);
    auto use_relative = get_bool("use_relative", false);

    /********************************************************************************/
    // 构建目标点
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = wp_x;
    target_pose.position.y = wp_y;
    target_pose.position.z = wp_z;

    tf2::Quaternion q;
    if (use_relative) {
        tf2::Quaternion q_cur;
        tf2::fromMsg(move_group.getCurrentPose().pose.orientation, q_cur);
        tf2::Quaternion q_delta;
        q_delta.setRPY(wp_roll, wp_pitch, wp_yaw);
        q = (q_cur * q_delta).normalized();
    } else {
        q.setRPY(wp_roll, wp_pitch, wp_yaw);
        q.normalize();
    }
    target_pose.orientation = tf2::toMsg(q);
    RCLCPP_INFO(LOGGER, "Target: (%.3f, %.3f, %.3f)", wp_x, wp_y, wp_z);

    /********************************************************************************/
    // 可视化目标点
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.publishAxisLabeled(target_pose, "target", rviz_visual_tools::SMALL);
    moveit_visual_tools.trigger();

    /********************************************************************************/
    // 规划并执行
    
    auto max_velocity_scaling_factor = get_double("max_velocity_scaling_factor", 0.3);
    auto max_acceleration_scaling_factor = get_double("max_acceleration_scaling_factor", 0.3);
    
    move_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor); // 设置最大速度为 0.5  
    move_group.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor); // 设置最大加速度为 0.5
    
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan: %s", success ? "SUCCESS" : "FAILED");

    if (success) {
        draw_trajectory_tool_path(plan.trajectory);
        moveit_visual_tools.trigger();
        move_group.execute(plan);
    } else {
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(LOGGER, "Planning failed");
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
