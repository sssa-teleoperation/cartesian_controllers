// Rotate decoder velocities from sdr_reference (A) to silvestrobase_link (B)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class VelocityTransformerNode : public rclcpp::Node
{
public:
  VelocityTransformerNode()
  : Node("velocity_transformer_node")
  {
    using std::placeholders::_1;

    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "input_raw", 10,
      std::bind(&VelocityTransformerNode::callback, this, _1));

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/input_in_robotframe", 10);

    RCLCPP_INFO(this->get_logger(), "Velocity transformer node started (rotation only)");

    // Rotation Matrix A->B 
    // x_A = -y_B, y_A = +z_B, z_A = -x_B
    //  R_BA = [[0, 0,-1],
    //          [-1,0, 0],
    //          [0, 1, 0]]
    // In ROS (ZYX) roll=+pi/2, pitch=0, yaw=-pi/2
    tf2::Quaternion q;
    q.setRPY(+M_PI/2.0, 0.0, -M_PI/2.0);        // roll, pitch, yaw
    R_BA_ = tf2::Matrix3x3(q);
  }

private:
  void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      RCLCPP_WARN(this->get_logger(), "Received velocity message with less than 6 elements");
      return;
    }

    // Assuming msg->data contains [v_A.x, v_A.y, v_A.z, w_A.x, w_A.y, w_A.z]
    tf2::Vector3 v_A(msg->data[0], msg->data[1], msg->data[2]);
    tf2::Vector3 w_A(msg->data[3], msg->data[4], msg->data[5]);

    // Transform to B (silvestrobase_link)
    tf2::Vector3 v_B = R_BA_ * v_A;
    tf2::Vector3 w_B = R_BA_ * w_A;

    std_msgs::msg::Float64MultiArray out;
    out.data = { v_B.x(), v_B.y(), v_B.z(), w_B.x(), w_B.y(), w_B.z() };
    pub_->publish(out);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  tf2::Matrix3x3 R_BA_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
