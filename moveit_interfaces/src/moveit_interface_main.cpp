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
    auto const draw_title = [&moveit_visual_tools](auto text)
    {
        auto const text_pose = []
        {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 0.5; 
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);
    };

    auto const prompt = [&moveit_visual_tools](auto text)
    {
        moveit_visual_tools.prompt(text);
    };

    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group.getRobotModel()->getJointModelGroup(
             PLANNING_GROUP)](auto const trajectory)
    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };


    /********************************************************************************/
    // 从参数读取目标点
    double wp_x = 0.0, wp_y = 0.0, wp_z = 0.0;
    double wp_roll = 0.0, wp_pitch = 0.0, wp_yaw = 0.0;
    bool use_relative = false;

    move_group_node->get_parameter("target_x",     wp_x);
    move_group_node->get_parameter("target_y",     wp_y);
    move_group_node->get_parameter("target_z",     wp_z);
    move_group_node->get_parameter("target_roll",  wp_roll);
    move_group_node->get_parameter("target_pitch", wp_pitch);
    move_group_node->get_parameter("target_yaw",   wp_yaw);
    move_group_node->get_parameter("use_relative", use_relative);

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
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);

    prompt("Press 'Next' in the RvizVisualTools windows to plan");
    draw_title("Planning");
    moveit_visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan: %s", success ? "SUCCESS" : "FAILED");

    if (success) {
        draw_trajectory_tool_path(plan.trajectory);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualTools windows to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group.execute(plan);
    } else {
        draw_title("Planning failed");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(LOGGER, "Planning failed");
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
