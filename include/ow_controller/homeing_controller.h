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

#ifndef OPEN_WALKER_HOMINGING_CONTROLLER_H
#define OPEN_WALKER_HOMINGING_CONTROLLER_H

#include <ow_hw_interface/hw_interface.h>

#include <ow_ik/inverse_kinematics.h>
#include <ow_joint_tracker/joint_tracker.h>
#include <ow_state_publisher/state_publisher.h>

#include <ow_core/interfaces/controller_base.h>

namespace ow_controller
{

  /*!
 * \brief The HomeingController class.
 * 
 * This Class uses some modules of the framework to execute the homing motion
 * of the robot.
 * It obtains the real robot data through the ow::IHwInterface and 
 * calls the update functions on each module.
 * Finally the HomeingController produces a new joint command that is then
 * send to to the robot.
 * 
 * This controller can be enabled in the ow_ros_control_plugin package. It 
 * is mainly used for debugging.
 */
  class HomeingController : public ow::ControllerBase
  {
  public:
    typedef ow::ControllerBase Base;

  protected:
    ow::Parameter parameter_; //!< Configuration
    ow_core::Flags flags_;  //!< Walking flags.

    // Internal modules
    ow_ik::InverseKinematics ik_;                  //!< Inverse kinematics
    ow_joint_tracker::JointTracker joint_tracker_; //!< Joint tracker
    ow_pub::StatePublisher pub_;                   //!< ros publisher
    std::vector<ow::ModuleBase *> modules_;        //!< vector of modules

    // Internal variables.
    ow::JointPosition q_fixed_upper_body_;  //!< Upper body home posture.
    ow::JointState q_;                      //!< Output joint state.

    // reference trajectories
    ow::CartesianState Xref_l_w_;   //!< Reference left foot Cart state wrt world.
    ow::CartesianState Xref_r_w_;   //!< Reference right foot Cart state wrt world.
    ow::CartesianState Xref_com_w_; //!< Reference CoM foot Cart state wrt world.

  public:
    /*!
    * \brief Default constructor.
    * \param freq
    */
    HomeingController(const ow::IHwInterface &robot);

    /*!
    * \brief Deconstructor.
    */
    virtual ~HomeingController();

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

#endif // OPEN_WALKER_HOMINGING_CONTROLLER_H
