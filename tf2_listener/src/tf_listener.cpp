#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class EndEffectorTF : public rclcpp::Node
{
public:
    EndEffectorTF()
    : Node("ee_tf_listener")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 10 Hz
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&EndEffectorTF::timerCallback, this)
        );
    }

private:
    void timerCallback()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_->lookupTransform(
                    "base_link",   // 目标坐标系
                    "link_6",       // 末端执行器（按你实际改）
                    tf2::TimePointZero   // 最新数据
                );

            auto & t = transformStamped.transform.translation;
            auto & q = transformStamped.transform.rotation;

            RCLCPP_INFO(this->get_logger(),
                "[EE Pose] Pos(x=%.3f, y=%.3f, z=%.3f) | "
                "Quat(x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                t.x, t.y, t.z,
                q.x, q.y, q.z, q.w
            );
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndEffectorTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}