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

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
  ros::NodeHandle node_handle;
  pub = node_handle.advertise<std_msgs::String>("gazebo/gripper/gripped", 1000);

  this->model = _parent;
  // std::cout<<model->GetName()<<"\n";
  // std::cout<<model->GetWorld()->Name()<<"\n";
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
}

void pick_plugin::OnMsg(ConstLogicalCameraImagePtr &_msg)
{
  // std::cout << ". . ."<<"\n";
  //mayor a 1 para descartar el ground_plane
  if(_msg->model().size()>1){
    // for (int i = 1; i <= _msg->model().size()-1; i++) {
    //   std::cout<<_msg->model(i).name()<<"\n";
    // }
    obj=_msg->model(1).name();
    if(obj.compare(old_obj)!=0){
      // std::cout << "pasamos de "<<old_obj<<" a "<<obj<< '\n';
      old_obj=obj;
      // std::cout<<"cambiando objeto"<<"\n";
      Agarrar(obj);
    }
  }else{
    if (old_obj.compare("nada")!=0) {
      Soltar(old_obj);
      old_obj="nada";
    }
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
  std_msgs::String aviso;
  aviso.data = "gripped";
  pub.publish(aviso);
}

void pick_plugin::Soltar(std::string obja){
  std::cout<<"ay señor bendito, soltado "<<obja<<"\n";
  this->fixedJoint->Detach();
  std_msgs::String aviso;
  aviso.data = "soltado";
  pub.publish(aviso);
}
