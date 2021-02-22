#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class ModelRotate : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelRotate::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double pi = 2*acos(0.0);
      auto pose = this->model->WorldPose();
      if(pose.Rot().Yaw() <(pi/4))
      {
       // Apply a small linear velocity to the model.
       this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.5);
       this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0.0);
      }
      if(pose.Rot().Yaw() >=(pi/4))
      {
       // Apply a small linear velocity to the model.
       this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.0);
       this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0.0);
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelRotate)
}
