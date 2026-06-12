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
/*!\file    DampedLeastSquaresSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/03/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/DampedLeastSquaresSolver.h>

#include <pluginlib/class_list_macros.hpp>

/**
 * \class cartesian_controller_base::DampedLeastSquaresSolver
 *
 * Users may explicitly specify this solver with \a "damped_least_squares" as \a
 * ik_solver in their controllers.yaml configuration file for each controller:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *   ros__parameters:
 *     ik_solver: "damped_least_squares"
 *     ...
 *
 *     solver:
 *         ...
 *         damped_least_squares:
 *             alpha: 0.5
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(cartesian_controller_base::DampedLeastSquaresSolver,
                       cartesian_controller_base::IKSolver)

namespace cartesian_controller_base
{
DampedLeastSquaresSolver::DampedLeastSquaresSolver() 
  : m_alpha(0.01),
    m_enable_ocp(true),
    m_ocp_horizon(0.02),
    m_max_acceleration(15.0),
    m_ocp_init(false)
   {}

DampedLeastSquaresSolver::~DampedLeastSquaresSolver() {}

double DampedLeastSquaresSolver::computeOCP(double T, double v0, double a0, double vf, double af){
  return -2.0 * (3.0 * v0 - 3.0 * vf + 2.0 * T * a0 + T * af) / (T * T);
}


trajectory_msgs::msg::JointTrajectoryPoint DampedLeastSquaresSolver::getJointControlCmds(
  rclcpp::Duration period, const ctrl::Vector6D & net_force)
{
  const double dt = period.seconds();

  // Compute joint jacobian
  m_jnt_jacobian_solver->JntToJac(m_current_positions, m_jnt_jacobian);

  // Compute joint velocities according to:
  // \f$ \dot{q} = ( J^T J + \alpha^2 I )^{-1} J^T f \f$
  ctrl::MatrixND identity;
  identity.setIdentity(m_number_joints, m_number_joints);

  m_handle->get_parameter(m_params + ".alpha", m_alpha);
  m_handle->get_parameter(m_params + ".ocp_horizon", m_ocp_horizon);
  m_handle->get_parameter(m_params + ".max_acceleration", m_max_acceleration);
  m_handle->get_parameter(m_params + ".enable_ocp", m_enable_ocp);

  KDL::JntArray desired_joint_velocities;
  desired_joint_velocities.data =
    (m_jnt_jacobian.data.transpose() * m_jnt_jacobian.data + m_alpha * m_alpha * identity)
      .inverse() *
    m_jnt_jacobian.data.transpose() * net_force;

  std_msgs::msg::Float64MultiArray vel_msg;

  vel_msg.data.resize(m_number_joints);
  for (int i = 0; i < m_number_joints; ++i) {
    vel_msg.data[i] = desired_joint_velocities(i);
  }

  m_vel_pub->publish(vel_msg);

  ctrl::Vector6D x_dot_ik = m_jnt_jacobian.data * desired_joint_velocities.data;

  geometry_msgs::msg::TwistStamped new_msg;
  new_msg.header.stamp = m_handle->now(); 
  new_msg.twist.linear.x = x_dot_ik(0);
  new_msg.twist.linear.y = x_dot_ik(1);
  new_msg.twist.linear.z = x_dot_ik(2);
  new_msg.twist.angular.x = x_dot_ik(3);
  new_msg.twist.angular.y = x_dot_ik(4);
  new_msg.twist.angular.z = x_dot_ik(5);

  m_xdot_pub->publish(new_msg);

  // if (!m_ocp_init)
  // {
  //   m_commanded_joint_velocities.assign(m_number_joints, 0.0);
  //   m_integrated_accelerations.assign(m_number_joints, 0.0);
  //   m_ocp_init = true;
  // }

  if (m_enable_ocp)
  {
    for (int i = 0; i < m_number_joints; ++i)
    {
      const double current_joint_acceleration = m_integrated_accelerations[i];
      const double current_commanded_velocity = m_commanded_joint_velocities[i];

      double target_joint_velocity = desired_joint_velocities(i);

      const double current_position = m_current_positions.data(i);
      const double upper_limit = m_upper_pos_limits(i);
      const double lower_limit = m_lower_pos_limits(i);

      if (current_position >= upper_limit && target_joint_velocity > 0.0)
      {
        target_joint_velocity = 0.0;
      }
      else if (current_position <= lower_limit && target_joint_velocity < 0.0)
      {
        target_joint_velocity = 0.0;
      }

      const double target_joint_acceleration = 0.0;

      const double u = computeOCP(
        m_ocp_horizon,
        current_commanded_velocity,
        current_joint_acceleration,
        target_joint_velocity,
        target_joint_acceleration
      );

      m_integrated_accelerations[i] += dt * u;

      m_integrated_accelerations[i] = std::clamp(m_integrated_accelerations[i],
                   -m_max_acceleration,
                   m_max_acceleration);

      m_commanded_joint_velocities[i] += dt * m_integrated_accelerations[i];

      m_current_velocities(i) = m_commanded_joint_velocities[i];

      RCLCPP_DEBUG_THROTTLE(
            m_handle->get_logger(),
            *m_handle->get_clock(),
            1000,
            "OCP on baby!");
    }
  }
  else
  {

    RCLCPP_DEBUG_THROTTLE(
          m_handle->get_logger(),
          *m_handle->get_clock(),
          1000,
          "OCP disabled, using raw IK joint velocities directly");

    m_current_velocities = desired_joint_velocities;
  }

  // Integrate once, starting with zero motion
  m_current_positions.data =
    m_last_positions.data + 0.5 * m_current_velocities.data * dt;

  // Make sure positions stay in allowed margins
  applyJointLimits();

  // Apply results
  trajectory_msgs::msg::JointTrajectoryPoint control_cmd;
  for (int i = 0; i < m_number_joints; ++i)
  {
    control_cmd.positions.push_back(m_current_positions(i));
    control_cmd.velocities.push_back(m_current_velocities(i));

    // Accelerations should be left empty. Those values will be interpreted
    // by most hardware joint drivers as max. tolerated values. As a
    // consequence, the robot will move very slowly.
  }
  control_cmd.time_from_start = period;  // valid for this duration

  // Update for the next cycle
  m_last_positions = m_current_positions;

  return control_cmd;
}

bool DampedLeastSquaresSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
                                    const KDL::Chain & chain,
                                    const KDL::JntArray & upper_pos_limits,
                                    const KDL::JntArray & lower_pos_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

  m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
  m_jnt_jacobian.resize(m_number_joints);

  auto_declare(m_params + ".alpha", m_alpha);
  auto_declare(m_params + ".enable_ocp", m_enable_ocp);
  auto_declare(m_params + ".ocp_horizon", m_ocp_horizon);
  auto_declare(m_params + ".max_acceleration", m_max_acceleration);


  m_vel_pub = m_handle->create_publisher<std_msgs::msg::Float64MultiArray>("ik_joint_velocities", 10);
  m_xdot_pub = m_handle->create_publisher<geometry_msgs::msg::TwistStamped>("ik_velocities", 10);

  m_commanded_joint_velocities.assign(m_number_joints, 0.0);
  m_integrated_accelerations.assign(m_number_joints, 0.0);
  m_ocp_init = true;

  return true;
}

}  // namespace cartesian_controller_base
