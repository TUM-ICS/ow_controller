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

#include <ow_controller/walking_controller.h>

namespace ow_controller
{

WalkingController::WalkingController(const ow::IHwInterface& robot):
  Base("WalkingController"),
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
  zeta_w_(ow::LinearState::Zero()),
  kp_imu_offset_(ow::Vector3::Zero())
{
  pub_.add(&robot);
}

WalkingController::~WalkingController()
{
}

bool WalkingController::init(const ow::Parameter& parameter, ros::NodeHandle& nh)
{
  euleranlge_pub_ = nh.advertise<geometry_msgs::Vector3>("offset_euleranlges", 1);
  offset_pub_ = nh.advertise<geometry_msgs::Vector3>("offset_com", 1);

  //----------------------------------------------------------------------------
  // get global ow parameter
  parameter.get<ow::Vector3>("kp_imu_offset", kp_imu_offset_);
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
  modules_.push_back(&com_tg_);
  modules_.push_back(&foot_tg_);
  modules_.push_back(&foot_comp_);
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
  pub_.add(&foot_tg_);
  pub_.add(&foot_comp_);
  pub_.add(&com_tg_);
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
  // build inital foot transformation, footsteps and dcm point set
  
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

  foot_steps_.resize(2, ow::FootStep::Zero());
  foot_steps_[0].pos() = Xref_r_w_.pos();
  foot_steps_[0].footId() = ow::FootId::RIGHT;
  foot_steps_[1].pos() = Xref_l_w_.pos();
  foot_steps_[1].footId() = ow::FootId::LEFT;

  dcm_set_.resize(2, ow::DCMPointSet::Zero());
  dcm_set_[0].dcm().z() = Xref_com_w_.pos().linear().z();
  dcm_set_[0].vrp() = dcm_set_[0].dcm();
  dcm_set_[1].dcm().z() = Xref_com_w_.pos().linear().z();
  dcm_set_[1].vrp() = dcm_set_[1].dcm();

  return true;
}

void WalkingController::start(const ow::IHwInterface& robot,
                              const ros::Time& time)
{
  // start all modules
  for(size_t i = 0; i < modules_.size(); ++i)
  {
    modules_[i]->startRequest(flags_, time);
  }

  // set joint postion to current position
  q_ = robot.jointStateCommand();

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
  
  // start the foot tg
  foot_tg_.start(Xref_l_w_, Xref_r_w_);
  
  // start center of mass trajectory generator
  com_tg_.start(Xref_com_w_);

  // start the state machine
  sm_.start(foot_steps_, dcm_set_, time);
}

ow::JointState WalkingController::update(
    const ow::IHwInterface& robot,
    const ros::Time& time,
    const ros::Duration& dt)
{
  // update the statemachine
  sm_.update(flags_, time, dt);
  flags_ = sm_.flags();
  dcm_set_ = sm_.DCMSet();
  foot_steps_ = sm_.footSteps();

  // update the center of mass trajectory generator
  com_tg_.update(flags_, dcm_set_, time, dt);                                   
  Xref_com_w_ = com_tg_.X_com_w();
  zetaRef_w_ = com_tg_.DCM_w();
  pRef_w_ = com_tg_.ZMP_w();

  // update the foot trajectory generator
  foot_tg_.update(flags_, foot_steps_, time, dt);                               
  Xref_l_w_ = foot_tg_.Xref_l_w();
  Xref_r_w_ = foot_tg_.Xref_r_w();

  // update the reference foot in real and cmd forwardkinematics
  if(flags_.eventOut() == ow::Flags::DOUBLE_SUPPORT_START) // == Foot Landed == Single Support Stop
  {
    if( flags_.supportFoot().isRight() )
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

  // update the foot compliance.
  foot_comp_.update( flags_,
                     Xreal_l_w_,
                     Xref_l_w_,
                     Xreal_r_w_,
                     Xref_r_w_,
                     robot.forceTorqueLeft(),
                     robot.forceTorqueRight() );
  Xoff_l_ = foot_comp_.Xoff_l();
  Xoff_r_ = foot_comp_.Xoff_r();

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
    ROS_INFO_THROTTLE( 1.0, "######## Homming ###########" );
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
