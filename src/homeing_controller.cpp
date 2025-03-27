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

#include <ow_controller/homeing_controller.h>

namespace ow_controller
{

HomeingController::HomeingController(const ow::IHwInterface& robot):
  Base("HomeingController"),
  q_(ow::JointState::Zero()),
  q_fixed_upper_body_(ow::JointPosition::Zero()),
  Xref_l_w_(ow::CartesianState::Zero()),
  Xref_r_w_(ow::CartesianState::Zero()),
  Xref_com_w_(ow::CartesianState::Zero())
{
  pub_.add(&robot);
}

HomeingController::~HomeingController()
{
}

bool HomeingController::init(const ow::Parameter& parameter, ros::NodeHandle& nh)
{
  //----------------------------------------------------------------------------
  // get global ow parameter
  ow::Scalar hip_height = parameter.get<ow::Scalar>("hip_height");

  // load module parameter
  parameter_.add<ow::JointPosition>("q_home_upper", ow::JointPosition::Zero());
  parameter_.add<ow::Scalar>("feet_separation", 0.15);
  parameter_.add<ow::Scalar>("hip_sagital", 0.0);
  parameter_.add<ow::Scalar>("total_time", 0.5);
  if(!parameter_.load(nh, "ready_pose"))
  {
    ROS_ERROR("%s::init: Config loading failed.", Base::name().c_str());
    return false;
  }

  //----------------------------------------------------------------------------
  // initialize all modules

  // push module into vector
  modules_.push_back(&ik_);
  modules_.push_back(&joint_tracker_);

  // add modules to publisher
  pub_.add(&ik_);
  pub_.add(&joint_tracker_);
  modules_.push_back(&pub_);

  //initialize all modules
  for(size_t i = 0; i < modules_.size(); ++i)
  {
    if(!modules_[i]->initRequest(parameter, nh))
    {
      ROS_ERROR("%s::init: Error init module", modules_[i]->name().c_str());
      return false;
    }
  }

  //----------------------------------------------------------------------------
  // build homing transformations

  Xref_l_w_.setZero();
  Xref_r_w_.setZero();
  Xref_com_w_.setZero();
  Xref_l_w_.pos().linear().y()
      = 0.5*parameter_.get<ow::Scalar>("feet_separation");
  Xref_r_w_.pos().linear().y()
      = -0.5*parameter_.get<ow::Scalar>("feet_separation");
  Xref_com_w_.pos().linear().x()
      = parameter_.get<ow::Scalar>("hip_sagital");
  Xref_com_w_.pos().linear().z() = hip_height;
  
  return true;
}

void HomeingController::start(const ow::IHwInterface& robot,
                              const ros::Time& time)
{
  // start all modules
  for(size_t i = 0; i < modules_.size(); ++i)
  {
    modules_[i]->startRequest(flags_, time);
  }

  // compute jointstate for homing motiton
  ik_.update(
    flags_, 
    robot.jointStateCommand(), 
    Xref_l_w_, 
    Xref_r_w_, 
    Xref_com_w_);

  // compute the homeing joint state
  q_fixed_upper_body_ = parameter_.get<ow::JointPosition>("q_home_upper");
  ow::JointPosition q_home =  ik_.q().pos() + q_fixed_upper_body_;

  // start joint tracker
  joint_tracker_.start(
        parameter_.get<ow::Scalar>("total_time"),
        ik_.jointIndex(ow::FootId::LEFT),
        ik_.jointIndex(ow::FootId::RIGHT),
        q_home,
        time);
}

ow::JointState HomeingController::update(
    const ow::IHwInterface& robot,
    const ros::Time& time,
    const ros::Duration& dt)
{
  // joint tracker
  joint_tracker_.update(q_, flags_, time, dt);
  q_ = joint_tracker_.jointState();

  // update the publisher state
  pub_.update(flags_, time, dt);
  return q_ ;
}

}
