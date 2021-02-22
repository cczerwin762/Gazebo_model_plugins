#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// Follow the model in front and stop briefly when about to make contact

namespace gazebo
{
  class Modelfollow : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&Modelfollow::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    { 
      auto world = this->model->GetWorld();
      auto leader = world->ModelByName("simple_tb");
      auto leaderpose = leader->WorldPose();
      double xleader = leaderpose.Pos()[0];
      double yleader = leaderpose.Pos()[1];
      auto pose = this->model->WorldPose();
      double x = pose.Pos()[0] - 1;
      double y = pose.Pos()[1];
      double followdist = xleader - x;
      if (followdist <1)
      {
       // If too close, stop for a brief moment
       this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.0);
       this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0.0);
      }
      else
      {
       // Apply a small linear velocity to the model.
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
  GZ_REGISTER_MODEL_PLUGIN(Modelfollow)
}
