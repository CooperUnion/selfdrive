#ifndef GAZEBOCAMCONTROL_H_
#define GAZEBOCAMCONTROL_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <igvc_self_drive_gazebo_plugins/GazeboCamConfig.h>
#include <tf/tf.h>

namespace gazebo
{

class GazeboCamControl : public SystemPlugin
{
public:
  GazeboCamControl();
  virtual ~GazeboCamControl();
protected:
  virtual void Load(int argc, char** argv);
  virtual void Init();
  virtual void Reset();
private:
  void reconfig(igvc_self_drive_gazebo_plugins::GazeboCamConfig& config, uint32_t level);
  void Update();
  void lookAtRobot(const rendering::VisualPtr& camera_target);

  rendering::UserCameraPtr user_cam_;
  std::vector<event::ConnectionPtr> connections_;
  ros::AsyncSpinner* async_;
  ros::NodeHandle* n_;
  boost::shared_ptr<dynamic_reconfigure::Server<igvc_self_drive_gazebo_plugins::GazeboCamConfig> > srv_;
  igvc_self_drive_gazebo_plugins::GazeboCamConfig cfg_;
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d camera_pos_;
#else
  math::Vector3 camera_pos_;
#endif
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboCamControl)
}

#endif // GAZEBOCAMCONTROL_H_
