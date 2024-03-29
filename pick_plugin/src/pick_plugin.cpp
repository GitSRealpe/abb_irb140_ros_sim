#include "pick_plugin/pick_plugin.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <iostream>
#include <string>

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(pick_plugin)
/////////////////////////////////////////////////
pick_plugin::pick_plugin(){}

/////////////////////////////////////////////////
pick_plugin::~pick_plugin(){}

void pick_plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::cout<<"cargandino el pluginino gripperino"<<"\n";

  ignorados=_sdf->GetElement("ignore")->Get<std::string>();
  std::cout<<ignorados<<"\n";

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
  ros::NodeHandle node_handle;
  pub = node_handle.advertise<std_msgs::String>("gazebo/gripper/gripped", 1000);

  this->model = _parent;
  old_obj="nada";
  this->mundo = this->model->GetWorld();
  physics::PhysicsEnginePtr physics = this->mundo->Physics();

  this->fixedJoint = physics->CreateJoint("revolute");
  // link del robot
  this->palmLink = this->model->GetLink("link_6");
  // std::cout<<this->palmLink->GetName()<<"\n";

  // Create the node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());
  // Create a topic name
  std::string topicName = "~/"+this->model->GetName()+"/left_finger/detector/models";
  // Subscribe to the topic, and register a callback
  this->sub = this->node->Subscribe(topicName,&pick_plugin::OnMsg, this);

  // Create a topic name
  std::string topic2 = "~/"+this->model->GetName()+"/right_finger/detector_right/models";
  // Subscribe to the topic, and register a callback
  this->sub2 = this->node->Subscribe(topic2,&pick_plugin::OnMsg2, this);
}

void pick_plugin::OnMsg(ConstLogicalCameraImagePtr &_msg)
{
  if(_msg->model().size()>0){
    drop=true;
    for (int i = 0; i <= _msg->model().size()-1; i++) {
      // std::cout<<_msg->model(i).name()<<"\n";
      if( ignorados.find(_msg->model(i).name()) == std::string::npos){
        obj=_msg->model(i).name();
        // std::cout<<"left_"<<obj<<"\n";
        if(obj.compare(old_obj)!=0  & obj==obj2){
          old_obj=obj;
          Agarrar(obj);
          aviso.data = "gripped";
          pub.publish(aviso);
        }
        drop=false;
      }
    }
    if ((old_obj.compare("nada")!=0) & drop ) {
      obj2="nadac";
      Soltar(old_obj);
      old_obj="nada";
      aviso.data = "soltado";
      pub.publish(aviso);
    }
  }
}

void pick_plugin::OnMsg2(ConstLogicalCameraImagePtr &_msg)
{
  if(_msg->model().size()>0){
    for (int i = 0; i <= _msg->model().size()-1; i++) {
      if( ignorados.find(_msg->model(i).name()) == std::string::npos){
        obj2=_msg->model(i).name();
        // std::cout<<"right_"<<obj2<<"\n";
      }
    }
  }else{
    obj2="nadac";
  }
  }


void pick_plugin::Agarrar(std::string obja){
  std::cout<<"ay papaaa, agarrado "<<obja<<"\n";
  // objeto
  this->objeto = this->mundo->ModelByName(obja);
  // link del objeto
  this->objeto_link = this->objeto->GetLink();
  ignition::math::Pose3d var = this->objeto_link->WorldPose() - this->palmLink->WorldPose();
  std::cout<<var<<"\n";
  this->fixedJoint->Load(this->palmLink, this->objeto_link, var);
  this->fixedJoint->Init();
  this->fixedJoint->SetUpperLimit(0, 0);
  this->fixedJoint->SetLowerLimit(0, 0);

}

void pick_plugin::Soltar(std::string obja){
  std::cout<<"ay señor bendito, soltado "<<obja<<"\n";
  this->fixedJoint->Detach();
}
