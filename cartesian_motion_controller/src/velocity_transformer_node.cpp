// try to convert the velocity in input from the decoder with a different sdr ( to put the silvestrobase_link such as the base_link)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

class VelocityTransformerNode : public rclcpp::Node
{
public:
  VelocityTransformerNode()
  : Node("velocity_transformer_node")
  {
    using std::placeholders::_1;

    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cartesian_motion_controller_silvestro/cartesian_input_sdr_reference", 10,
      std::bind(&VelocityTransformerNode::callback, this, _1));

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/cartesian_motion_controller_silvestro/CartesianMotionControllerInput", 10);

    RCLCPP_INFO(this->get_logger(), "Velocity transformer node started");

    // Transform from sdr_reference to silvestrobase_link
    tf2::Quaternion q;
    q.setRPY(0.0, 1.57, 1.57);
    tf2::Vector3 origin(0.0, 0.0, 0.1625);
    transform_ = tf2::Transform(q, origin);

    // Offset of the end-effector point P(E-E) in sdr_reference
    r_OP_in_A_ = tf2::Vector3(0.35, -0.06, 0.0);
  }

private:
  void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      RCLCPP_WARN(this->get_logger(), "Received velocity message with less than 6 elements");
      return;
    }
    
    tf2::Vector3 v_O_A(msg->data[0], msg->data[1], msg->data[2]);
    tf2::Vector3 omega_A(msg->data[3], msg->data[4], msg->data[5]);

    // Velocity of point P(E-E) in frame A (sdr_reference)
    tf2::Vector3 v_P_A = v_O_A + omega_A.cross(r_OP_in_A_);

    // Transform into frame B (silvestrobase_link)
    tf2::Vector3 r = transform_.getOrigin();
    tf2::Vector3 v_origin = omega_A.cross(r);
    tf2::Vector3 v_P_B = transform_.getBasis().transpose() * (v_P_A - v_origin);
    tf2::Vector3 omega_B = transform_.getBasis().transpose() * omega_A;

    std_msgs::msg::Float64MultiArray out;
    out.data = {
      v_P_B.x(), v_P_B.y(), v_P_B.z(),
      omega_B.x(), omega_B.y(), omega_B.z()
    };

    pub_->publish(out);
  }


  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  tf2::Transform transform_;
  tf2::Vector3 r_OP_in_A_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
