#include "GazeboCamControl.h"

namespace gazebo
{

GazeboCamControl::GazeboCamControl()
{
}

void GazeboCamControl::Load(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_cam_control");
  n_ = new ros::NodeHandle("gazebo_cam_control");

  srv_.reset(new dynamic_reconfigure::Server<igvc_self_drive_gazebo_plugins::GazeboCamConfig>(*n_));
  srv_->setCallback(boost::bind(&GazeboCamControl::reconfig, this, _1, _2));

  async_ = new ros::AsyncSpinner(3);
  async_->start();
}

void GazeboCamControl::reconfig(igvc_self_drive_gazebo_plugins::GazeboCamConfig& config, uint32_t level)
{
  cfg_ = config;
}

void GazeboCamControl::Init()
{
  connections_.push_back(event::Events::ConnectRender(boost::bind(&GazeboCamControl::Update, this)));
}

void GazeboCamControl::Reset()
{
}

void GazeboCamControl::Update()
{
  rendering::VisualPtr camera_target = rendering::get_scene()->GetVisual(cfg_.model_name);

  if (!user_cam_){
    user_cam_ = gui::get_active_camera();
  }else if (cfg_.enable){
    if (camera_target){
      lookAtRobot(camera_target);
      ignition::math::Vector3d ignition_pos;
#if GAZEBO_MAJOR_VERSION >= 9
      ignition_pos.Set(camera_pos_.X(), camera_pos_.Y(), camera_pos_.Z());
      user_cam_->SetWorldPosition(ignition_pos);
      user_cam_->SetFocalPoint(camera_target->Pose().Pos());
#else
      ignition_pos.Set(camera_pos_.x, camera_pos_.y, camera_pos_.z);
      user_cam_->SetWorldPosition(ignition_pos);
      user_cam_->SetFocalPoint(camera_target->GetPose().pos);
#endif
    }
  }
}

void GazeboCamControl::lookAtRobot(const rendering::VisualPtr& camera_target)
{
  // Get current orientation of the camera and compute equivalent rotation matrix
  tf::Matrix3x3 rot_mat;
  rot_mat.setRotation(tf::Quaternion(user_cam_->WorldRotation().X(), user_cam_->WorldRotation().Y(), user_cam_->WorldRotation().Z(), user_cam_->WorldRotation().W()));

  // Move camera such that LOS vector points toward robot
#if GAZEBO_MAJOR_VERSION >= 9
  camera_pos_.X(camera_target->Pose().Pos().X() - cfg_.view_dist * rot_mat.getColumn(0).x());
  camera_pos_.Y(camera_target->Pose().Pos().Y() - cfg_.view_dist * rot_mat.getColumn(0).y());
  camera_pos_.Z(camera_target->Pose().Pos().Z() - cfg_.view_dist * rot_mat.getColumn(0).z());
#else
  camera_pos_.x = camera_target->GetPose().pos.x - cfg_.view_dist * rot_mat.getColumn(0).x();
  camera_pos_.y = camera_target->GetPose().pos.y - cfg_.view_dist * rot_mat.getColumn(0).y();
  camera_pos_.z = camera_target->GetPose().pos.z - cfg_.view_dist * rot_mat.getColumn(0).z();
#endif
}

GazeboCamControl::~GazeboCamControl()
{
  this->connections_.clear();
  this->user_cam_.reset();
}

}
