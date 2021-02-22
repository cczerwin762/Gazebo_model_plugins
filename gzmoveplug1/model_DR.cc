#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <cmath>

// Driving in a straight line while rejecting disturbances
// currently only has a Kp term, may implement a Kd term

namespace gazebo
{
  class ModelDR : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelDR::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      auto pose = this->model->WorldPose();
      double x = pose.Pos()[0];
      double y = pose.Pos()[1];
      double Yaw = pose.Rot().Yaw();
      double de_dt = tan(Yaw);
      double Kp = (1);
      double Kd = (1);
      // Apply a small linear velocity to the model.
      this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,(1.0 - (y*Kp) - (de_dt*Kd)));
      this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,(1.0 + (y*Kp) + (de_dt*Kd)));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelDR)
}
