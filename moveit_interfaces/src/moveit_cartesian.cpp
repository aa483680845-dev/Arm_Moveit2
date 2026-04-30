#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spin_thread([&executor]()
                            { executor.spin(); }); //分出一个线程用于执行接收和发送ros2消息，不影响主线程

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        move_group_node,
        "base_link",
        rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group.getRobotModel()};

    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    rclcpp::sleep_for(std::chrono::seconds(1));

    auto get_double = [&](const std::string &name, double def)
    {
        return move_group_node->has_parameter(name)
                   ? move_group_node->get_parameter(name).as_double()
                   : def;
    };
    auto get_double_array = [&](const std::string &name, std::vector<double> def)
    {
        return move_group_node->has_parameter(name)
                   ? move_group_node->get_parameter(name).as_double_array()
                   : def;
    };
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

    auto dx     = get_double_array("waypoint_dx",     {});
    auto dy     = get_double_array("waypoint_dy",     {});
    auto dz     = get_double_array("waypoint_dz",     {});
    auto droll  = get_double_array("waypoint_droll",  std::vector<double>(dx.size(), 0.0));
    auto dpitch = get_double_array("waypoint_dpitch", std::vector<double>(dx.size(), 0.0));
    auto dyaw   = get_double_array("waypoint_dyaw",   std::vector<double>(dx.size(), 0.0));
    double eef_step          = get_double("eef_step", 0.01);
    double jump_threshold    = get_double("jump_threshold", 0.0);
    double success_threshold = get_double("success_fraction_threshold", 0.6);

    if (dx.size() != dy.size() || dx.size() != dz.size() ||
        dx.size() != droll.size() || dx.size() != dpitch.size() || dx.size() != dyaw.size())
    {
        RCLCPP_ERROR(LOGGER, "All waypoint arrays must have the same length!");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    /********************************************************************************/
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    tf2::Quaternion start_q;
    tf2::fromMsg(start_pose.orientation, start_q);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    for (size_t i = 0; i < dx.size(); ++i)
    {
        geometry_msgs::msg::Pose p = start_pose;
        p.position.x += dx[i];
        p.position.y += dy[i];
        p.position.z += dz[i];

        tf2::Quaternion delta_q;
        delta_q.setRPY(droll[i], dpitch[i], dyaw[i]);
        tf2::Quaternion result_q = (start_q * delta_q).normalized(); // 内旋：绕末端自身轴，.normalized归一化，将四元数变为单位四元数
        p.orientation = tf2::toMsg(result_q); //将四元数转换为消息ROS消息

        waypoints.push_back(p);
    }
    waypoints.push_back(start_pose); // 最后返回起始点
    RCLCPP_INFO(LOGGER, "Loaded %zu waypoints from config", dx.size());
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(LOGGER, "Cartesian path success rate: %.2f%%", fraction * 100.0);

    /********************************************************************************/
    // 可视化路径点
    moveit_visual_tools.deleteAllMarkers(); //清除之前的rviz可视化
    moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::XXXXSMALL);//发布路径点
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        moveit_visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::XXSMALL);//发布路径点的坐标轴
    moveit_visual_tools.trigger(); //将上面的可视化消息一次性发布到RViz

    /********************************************************************************/
    if (fraction > success_threshold)
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
