#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// Continuos square. To make it stop after one just include 
// another if statement.

namespace gazebo
{
  class ModelDriveSqr : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelDriveSqr::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Get Position of the model
      double pi = 2*acos(0.0);
      auto pose = this->model->WorldPose();
      double x = pose.Pos()[0];
      double y = pose.Pos()[1];
      double Yaw = pose.Rot().Yaw();
      if (Yaw<0)
      {
       Yaw = Yaw + (2*pi);
      }
      if ((x>=3.0 && y<1.0 && Yaw<((pi/2)-0.03)) || (x>=2.0 && y>=3.0 && Yaw<(pi-0.03)) || (x<=0 && y>=2.0 && Yaw<((3*pi/2)-0.03)) || (x<=1.0 && y<0.0 && Yaw>pi && Yaw<((2*pi) - 0.03)))
      {
       // Turning
       this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.2);
       this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,-0.2);
      }
      else
      {
       // Straight 
       this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,1.0);
       this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,1.0);
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelDriveSqr)
}
