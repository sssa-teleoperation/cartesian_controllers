////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

//velocity controller for cartesian motion 

#include <cartesian_motion_controller/cartesian_motion_controller.h>

#include <algorithm>
#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

namespace cartesian_motion_controller
{

CartesianMotionController::CartesianMotionController() : Base::CartesianControllerBase() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  m_decoder_subscr = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    get_node()->get_name() + std::string("/CartesianMotionControllerInput"), 3,
    std::bind(&CartesianMotionController::decoderCommandCallback, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type 
CartesianMotionController::update(const rclcpp::Time & time,
                                  const rclcpp::Duration & period)
{
    // Synchronize the internal model and the real robot
    Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

    std::array<double, 6> cmd = m_latest_command;

    KDL::Twist twist_cmd(
        KDL::Vector(cmd[0], cmd[1], cmd[2]),  // linear part
        KDL::Vector(cmd[3], cmd[4], cmd[5])   // angular part
    );

    ctrl::Vector6D motion_error;
    motion_error << twist_cmd.vel.x(),
                   twist_cmd.vel.y(),
                   twist_cmd.vel.z(),
                   twist_cmd.rot.x(),
                   twist_cmd.rot.y(),
                   twist_cmd.rot.z();

    Base::computeJointControlCmds(motion_error, period);
    Base::writeJointControlCmds();

    return controller_interface::return_type::OK;
}


void CartesianMotionController::decoderCommandCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Check if the controller is active
  if (!this->isActive()) {
    return;
  }

  std::array<double, 6> data = {
    msg->twist.linear.x,
    msg->twist.linear.y,
    msg->twist.linear.z,
    msg->twist.angular.x,
    msg->twist.angular.y,
    msg->twist.angular.z
  };


  // Check if the message has NaN
  for (size_t i = 0; i < data.size(); ++i) {
    if (std::isnan(data[i])) {
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(
        get_node()->get_logger(),
        clock,
        3000,
        "NaN detected in decoder command. Ignoring input."
      );
      return;
    }
  }

  // Copy the command data
  std::copy_n(
    data.begin(),
    std::min(data.size(), m_latest_command.size()),
    m_latest_command.begin()
  );
}


}  // namespace cartesian_motion_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    cartesian_motion_controller::CartesianMotionController,
    controller_interface::ControllerInterface
)
