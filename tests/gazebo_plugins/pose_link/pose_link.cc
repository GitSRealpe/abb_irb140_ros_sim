#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class PoseLink : public ModelPlugin
  {
    public: PoseLink() : ModelPlugin()
            {
              printf("pose_link cargado 2!\n");
              //160
            }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      model->SetGravityMode(false);
      this->world= this->model->GetWorld();
      //this->bot=this->world->ModelByName("box");
      //this->link=this->bot->GetLink("link");
      //this->bot=this->world->ModelByName("irb_140");
      //this->link=this->bot->GetLink("irb_140::link_6");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PoseLink::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      if(i<3000){
        i=i+1;
        std::cout << i<<"\n";
      }else{
      if(i==3000){
        std::cout <<" llegaste jermano\n";
        this->bot=this->world->ModelByName("irb_140");
        //this->link=this->bot->GetLink("irb_140::link_6");
        this->link=this->bot->GetLink("irb_140::fake_link");
        std::cout << this->link->RelativePose().Pos()<<"\n";
        i=i+1;
        }
        pose=this->link->RelativePose();
        //pose.Pos().X(pose.Pos().X()+0.5);
        this->model->SetWorldPose(pose);
      }

    }
    // Pointer to the model
    private:
      physics::ModelPtr model,bot;
      physics::LinkPtr link;
      physics::WorldPtr world;
      ignition::math::Pose3<double> pose;
      ignition::math::Vector3d pos;
      ignition::math::Vector3d rot;

      int i=0;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PoseLink)
}
