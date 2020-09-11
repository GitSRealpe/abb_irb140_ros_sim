#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class PoseMod : public ModelPlugin
  {
    public: PoseMod() : ModelPlugin()
            {
              printf("pose_mod cargado!\n");
              //160
            }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->world= this->model->GetWorld();
      this->box=this->world->ModelByName("box");
      model->SetGravityMode(false);
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PoseMod::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      //i=i+1;
      //printf("%i",i);
      std::cout << this->model->RelativePose().Pos()<<"\n";
      pose=this->box->RelativePose();
      pose.Pos().Y(this->box->RelativePose().Pos().Y()+3);
      this->model->SetWorldPose(pose);
    }
    // Pointer to the model
    private:
      physics::ModelPtr model,box;
      physics::WorldPtr world;
      ignition::math::Pose3<double> pose;
      ignition::math::Vector3d pos;
      ignition::math::Vector3d rot;

      int i=0;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PoseMod)
}
