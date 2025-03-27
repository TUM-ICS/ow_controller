/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_BALANCING_CONTROLLER_H
#define OPEN_WALKER_BALANCING_CONTROLLER_H

#include <ow_hw_interface/hw_interface.h>

#include <ow_fk/forward_kinematics.h>
#include <ow_ik/inverse_kinematics.h>
#include <ow_zmp/zmp_estimator.h>
#include <ow_balancer/balancer.h>
#include <ow_com/com_estimator.h>
#include <ow_cmd_gen/command_generator.h>
#include <ow_joint_tracker/joint_tracker.h>
#include <ow_state_publisher/state_publisher.h>
#include <ow_state_machine/state_machine.h>

#include <ow_core/interfaces/controller_base.h>

namespace ow_controller
{

  /*!
 * \brief The BalancingController class.
 * 
 * This Class uses some modules of the framework to realize balancing.
 * It obtains the real robot data through the ow::IHwInterface and 
 * calls the update functions on each module.
 * Finally the BalancingController produces a new joint command that is then
 * send to to the robot.
 * 
 * This controller can be enabled in the ow_ros_control_plugin package. It 
 * is mainly used for debugging.
 */
  class BalancingController : public ow::ControllerBase
  {
  public:
    typedef ow::ControllerBase Base;

  protected:
    ow::Parameter parameter_; //!< Configuration
    ow::Flags flags_;       //!< Flags

    // Internal modules
    ow_fk::ForwardKinematics fk_cmd_;              //!< Forward kinematics virtual
    ow_fk::ForwardKinematics fk_real_;             //!< Forward kinematics real
    ow_ik::InverseKinematics ik_;                  //!< Inverse kinematics
    ow_zmp::ZmpEstimator zmp_;                     //!< ZMP estimator
    ow_balancer::Balancer balancer_;               //!< Balancer
    ow_com::COMEstimator com_;                     //!< Center of mass estimation
    ow_cmd_gen::CommandGenerator cmd_gen_;         //!< Command generator
    ow_joint_tracker::JointTracker joint_tracker_; //!< Joint tracker
    ow_pub::StatePublisher pub_;                   //!< ros publisher
    ow_sm::StateMachine sm_;                       //!< state machine
    std::vector<ow::ModuleBase *> modules_;        //!< vector of modules

    // Internal variables.
    ow::JointPosition q_fixed_upper_body_;  //!< Upper body home posture.
    ow::JointState q_;                      //!< Output joint state.

    // virtual robot state
    ow::CartesianState Xv_l_w_;     //!< virtual left foot wrt. world.
    ow::CartesianState Xv_r_w_;     //!< virtual right foot wrt. world.
    ow::CartesianState Xv_com_w_;   //!< virtual CoM wrt. world.
    ow::CartesianState Xv_com_hip_; //!< virtual hip wrt. world.

    // real robot state
    ow::CartesianState Xreal_l_w_;   //!< Real left foot wrt. world.
    ow::CartesianState Xreal_r_w_;   //!< Real right foot wrt. world.
    ow::CartesianState Xreal_com_w_; //!< Real CoM wrt. world.

    // reference trajectories
    ow::CartesianState Xref_l_w_;   //!< Reference left foot Cart state wrt world.
    ow::CartesianState Xref_r_w_;   //!< Reference right foot Cart state wrt world.
    ow::CartesianState Xref_com_w_; //!< Reference CoM foot Cart state wrt world.

    // cartesian offsets expressed wrt to their ref frames
    ow::CartesianState Xoff_l_;   //!< Left foot offset on the left ref frame
    ow::CartesianState Xoff_r_;   //!< Right foot offset on the right ref frame
    ow::CartesianState Xoff_com_; //!< CoM offset on the ref com frame

    // cartesian robot commands 
    ow::CartesianState Xcmd_l_w_;   //!< Left foot Cart state command wrt. world.
    ow::CartesianState Xcmd_r_w_;   //!< Right foot Cart state command wrt. world.
    ow::CartesianState Xcmd_com_w_; //!< CoM Cart state wrt. command world.

    // zero moment point states
    ow::LinearState pRef_w_;   //!< Reference ZMP trajectory wrt world
    ow::LinearState p_w_;      //!< ZMP state wrt world

    // divergent component of motion states
    ow::LinearState zetaRef_w_;            //!< Reference DCM trajectory.
    ow::LinearState zeta_w_;               //!< DCM state.

  public:
    /*!
  * \brief Default constructor.
  * \param freq
  */
    BalancingController(const ow::IHwInterface &robot);

    /*!
  * \brief Deconstructor.
  */
    virtual ~BalancingController();

    /** 
  * \brief init interal\parameter
  *
  * \param ros nh for namespace
  */
    virtual bool init(const ow::Parameter &parameter, ros::NodeHandle &nh);

    /** 
  * \brief start the controller, called befor update
  *
  * \param starting time
  */
    virtual void start(const ow::IHwInterface &robot, const ros::Time &time);

    /** 
  * \brief performs update step of the controller, called periodically
  *
  * \param current jointstate
  * \param current time
  * \param current deltatime since last call
  */
    virtual ow::JointState update(
        const ow::IHwInterface &robot,
        const ros::Time &time,
        const ros::Duration &dt);
  };

} // namespace ow_controller

#endif // OPEN_WALKER_WALKING_CONTROLLER_H
