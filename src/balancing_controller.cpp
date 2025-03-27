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

#include <ow_controller/balancing_controller.h>

namespace ow_controller
{

BalancingController::BalancingController(const ow::IHwInterface& robot):
  Base("BalancingController"),
  q_(ow::JointState::Zero()),
  q_fixed_upper_body_(ow::JointPosition::Zero()),
  Xv_l_w_(ow::CartesianState::Zero()),
  Xv_r_w_(ow::CartesianState::Zero()),
  Xv_com_w_(ow::CartesianState::Zero()),
  Xv_com_hip_(ow::CartesianState::Zero()),
  Xcmd_l_w_(ow::CartesianState::Zero()),
  Xcmd_r_w_(ow::CartesianState::Zero()),
  Xcmd_com_w_(ow::CartesianState::Zero()),
  Xreal_l_w_(ow::CartesianState::Zero()),
  Xreal_r_w_(ow::CartesianState::Zero()),
  Xreal_com_w_(ow::CartesianState::Zero()),
  Xoff_l_(ow::CartesianState::Zero()),
  Xoff_r_(ow::CartesianState::Zero()),
  Xoff_com_(ow::CartesianState::Zero()),
  Xref_l_w_(ow::CartesianState::Zero()),
  Xref_r_w_(ow::CartesianState::Zero()),
  Xref_com_w_(ow::CartesianState::Zero()),
  pRef_w_(ow::LinearState::Zero()),
  zetaRef_w_(ow::LinearState::Zero()),
  p_w_(ow::LinearState::Zero()),
  zeta_w_(ow::LinearState::Zero())
{
  pub_.add(&robot);
}

BalancingController::~BalancingController()
{
}

bool BalancingController::init(const ow::Parameter& parameter, ros::NodeHandle& nh)
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
  modules_.push_back(&fk_real_);
  modules_.push_back(&fk_cmd_);
  modules_.push_back(&zmp_);
  modules_.push_back(&balancer_);
  modules_.push_back(&com_);
  modules_.push_back(&cmd_gen_);
  modules_.push_back(&joint_tracker_);

  // add modules to publisher
  pub_.add(&balancer_);
  pub_.add(&com_);
  pub_.add(&fk_real_, false);
  pub_.add(&fk_cmd_, true);
  pub_.add(&ik_);
  pub_.add(&zmp_);
  pub_.add(&cmd_gen_);
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

  // test
  if(!sm_.initRequest(parameter, nh))
  {
    ROS_ERROR("%s::init: Error init module", sm_.name().c_str());
    return false;
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

  pRef_w_.setZero();
  zetaRef_w_.setZero();
  zetaRef_w_.pos().z() = Xref_com_w_.pos().linear().z();
  return true;
}

void BalancingController::start(const ow::IHwInterface& robot,
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

  // set odometry for FK modules.
  fk_cmd_.setRefFootState( flags_, Xref_r_w_ );
  fk_real_.setRefFootState( flags_, Xref_r_w_ );

  // forward kinematic virtual robot
  fk_cmd_.update( flags_, robot.jointStateCommand() );
  Xv_l_w_ = fk_cmd_.X_l_w();
  Xv_r_w_ = fk_cmd_.X_r_w();
  Xv_com_w_ = fk_cmd_.X_com_w();

  // forward kinematic real robot
  fk_real_.update( flags_, robot.jointStateReal() );
  Xreal_l_w_ = fk_real_.X_l_w();
  Xreal_r_w_ = fk_real_.X_r_w();
  Xreal_com_w_ = fk_real_.X_com_w();

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

ow::JointState BalancingController::update(
    const ow::IHwInterface& robot,
    const ros::Time& time,
    const ros::Duration& dt)
{
  sm_.update(flags_, time, dt);
  flags_ = sm_.flags();

  // update the reference foot in real and cmd forwardkinematics
  if(flags_.eventOut() == ow::Flags::DOUBLE_SUPPORT_START)
  {
    if( flags_.supportFoot() == ow::FootId::RIGHT )
    {
      fk_cmd_.setRefFootState( flags_, Xv_r_w_ );
      fk_real_.setRefFootState( flags_, Xv_r_w_ );
    }
    else
    {
      fk_cmd_.setRefFootState( flags_, Xv_l_w_ );
      fk_real_.setRefFootState( flags_, Xv_l_w_ );
    }
  }

  // virtual robot
  fk_cmd_.update( flags_, robot.jointStateCommand());
  Xv_l_w_ = fk_cmd_.X_l_w();
  Xv_r_w_ = fk_cmd_.X_r_w();
  Xv_com_w_ = fk_cmd_.X_com_w();
  Xv_com_hip_ = fk_cmd_.X_com_hip();

  // real robot
  fk_real_.update( flags_, robot.jointStateReal() );
  Xreal_l_w_ = fk_real_.X_l_w();
  Xreal_r_w_ = fk_real_.X_r_w();
  Xreal_com_w_ = fk_real_.X_com_w();

  // update DCM state
  com_.update(Xv_com_w_, robot.imu());
  zeta_w_ = com_.DCMr_w();

  // update zmp estimator
  zmp_.update( flags_,
               robot.forceTorqueLeft(), Xv_l_w_,
               robot.forceTorqueRight(), Xv_r_w_);
  p_w_ = zmp_.ZMP_w();

  // update the balancer
  balancer_.update( flags_,
                    pRef_w_, p_w_,
                    zetaRef_w_, zeta_w_,
                    robot.imu(), Xref_com_w_, Xreal_com_w_,
                    Xref_l_w_, Xref_r_w_);
  Xoff_com_ = balancer_.Xoff_com();

  // update the robot command
  cmd_gen_.update( Xref_l_w_, Xoff_l_,
                   Xref_r_w_, Xoff_r_,
                   Xref_com_w_, Xoff_com_);
  Xcmd_l_w_ = cmd_gen_.Xcmd_l_w();
  Xcmd_r_w_ = cmd_gen_.Xcmd_r_w();
  Xcmd_com_w_ = cmd_gen_.Xcmd_com_w();

  // compute the ik solution
  if(flags_.state() == ow::Flags::HOMEING)
  {
    ROS_INFO_THROTTLE( 1.0, "######## Homing ###########" );
  }
  else
  {
    ik_.update( flags_,
                robot.jointStateCommand(),
                Xcmd_l_w_,
                Xcmd_r_w_,
                Xcmd_com_w_);

    q_ = ik_.q();
    q_.pos() += q_fixed_upper_body_;  
  }

  // joint tracker
  joint_tracker_.update(q_, flags_, time, dt);
  q_ = joint_tracker_.jointState();

  // update the publisher state
  pub_.update(flags_, time, dt);
  return q_ ;
}

}
