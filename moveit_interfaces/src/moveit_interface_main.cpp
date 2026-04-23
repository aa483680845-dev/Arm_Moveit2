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
    // 从参数读取多个路径点
    std::vector<double> wp_x, wp_y, wp_z, wp_roll, wp_pitch, wp_yaw;
    bool use_relative = false;

    move_group_node->get_parameter("waypoints_x",     wp_x);
    move_group_node->get_parameter("waypoints_y",     wp_y);
    move_group_node->get_parameter("waypoints_z",     wp_z);
    move_group_node->get_parameter("waypoints_roll",  wp_roll);
    move_group_node->get_parameter("waypoints_pitch", wp_pitch);
    move_group_node->get_parameter("waypoints_yaw",   wp_yaw);
    move_group_node->get_parameter("use_relative",    use_relative);

    if (wp_x.empty()) {
        RCLCPP_ERROR(LOGGER, "No waypoints loaded, check target_pose.yaml");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    RCLCPP_INFO(LOGGER, "Loaded %zu waypoints", wp_x.size());

    /********************************************************************************/
    // 构建路径点列表
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // use_relative 模式下，从当前末端姿态开始累积旋转
    tf2::Quaternion q_prev;
    tf2::fromMsg(move_group.getCurrentPose().pose.orientation, q_prev);

    for (size_t i = 0; i < wp_x.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = wp_x[i];
        pose.position.y = wp_y[i];
        pose.position.z = wp_z[i];

        tf2::Quaternion q_delta;
        q_delta.setRPY(wp_roll[i], wp_pitch[i], wp_yaw[i]);

        tf2::Quaternion q_result;
        if (use_relative) {
            // 右乘：绕末端自身轴旋转（内旋），前一点姿态作为基准
            q_result = (q_prev * q_delta).normalized();
            q_prev = q_result;
        } else {
            // 绝对姿态：相对于规划坐标系（外旋）
            q_result = q_delta.normalized();
        }
        pose.orientation = tf2::toMsg(q_result);
        waypoints.push_back(pose);

        RCLCPP_INFO(LOGGER, "Waypoint %zu: (%.3f, %.3f, %.3f)", i, wp_x[i], wp_y[i], wp_z[i]);
    }

    /********************************************************************************/
    // 可视化路径点
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        moveit_visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
    moveit_visual_tools.trigger();

    /********************************************************************************/
    // 关节空间规划：依次规划并执行到每个路径点
    for (size_t i = 0; i < waypoints.size(); ++i) {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(waypoints[i]);

        prompt("Press 'Next' in the RvizVisualTools windows to plan");
        draw_title("Planning");
        moveit_visual_tools.trigger();// 将之前缓存的可视化内容，一次性发送到rviz中显示出来

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Waypoint %zu plan: %s", i, success ? "SUCCESS" : "FAILED");

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
            RCLCPP_ERROR(LOGGER, "Planning failed at waypoint %zu, stopping", i);
            break;
        }
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
