#include "ContactROSBridge.h"
#include <tf2/utils.h>

#include <cnoid/BodyLoader>
#include <cnoid/SceneGraph>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>
#include <eigen_conversions/eigen_msg.h>

ContactROSBridge::ContactROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_contactROSOut_("contactOut", m_contactROS_),
  m_contactRTMIn_("contactIn", m_contactRTM_)
{
}

RTC::ReturnCode_t ContactROSBridge::onInitialize(){
  addOutPort("contactOut", m_contactROSOut_);
  addInPort("contactIn", m_contactRTMIn_);

  cnoid::BodyLoader bodyLoader;

  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  for(int l=0;l<this->robot_->numLinks() ; l++){
    cnoid::SgGroup* shape = this->robot_->link(l)->shape();
    if(shape && shape->numChildObjects() > 0){
      this->VRMLToURDFLinkNameMap_[this->robot_->link(l)->name()] = shape->child(0)->name();
      this->URDFToVRMLLinkNameMap_[shape->child(0)->name()] = this->robot_->link(l)->name();
    }
  }

  ros::NodeHandle pnh("~");

  sub_ = pnh.subscribe("input", 1, &ContactROSBridge::topicCallback, this);
  pub_ = pnh.advertise<contact_state_msgs::ContactArray>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ContactROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_contactRTMIn_.isNew()){
    this->m_contactRTMIn_.read();

    contact_state_msgs::ContactArray msg;
    msg.header.stamp = ros::Time::now();
    for(int i=0;i<m_contactRTM_.data.length();i++){
      contact_state_msgs::Contact state;
      if(this->VRMLToURDFLinkNameMap_.find(std::string(m_contactRTM_.data[i].link1)) != this->VRMLToURDFLinkNameMap_.end()){
        state.pose.header.frame_id = this->VRMLToURDFLinkNameMap_[std::string(m_contactRTM_.data[i].link1)];
      }else{
        state.pose.header.frame_id = std::string(m_contactRTM_.data[i].link1);
      }
      Eigen::Isometry3d pose;
      eigen_rtm_conversions::poseRTMToEigen(m_contactRTM_.data[i].local_pose, pose);
      tf::poseEigenToMsg(pose, state.pose.pose);
      if(this->VRMLToURDFLinkNameMap_.find(std::string(m_contactRTM_.data[i].link2)) != this->VRMLToURDFLinkNameMap_.end()){
        state.link = this->VRMLToURDFLinkNameMap_[std::string(m_contactRTM_.data[i].link2)];
      }else{
        state.link = std::string(m_contactRTM_.data[i].link2);
      }
      state.free_x = m_contactRTM_.data[i].free_x;
      state.free_y = m_contactRTM_.data[i].free_y;
      msg.contacts.push_back(state);
    }
    this->pub_.publish(msg);
  }

  return RTC::RTC_OK;
}

void ContactROSBridge::topicCallback(const contact_state_msgs::ContactArray::ConstPtr& msg) {
  m_contactROS_.tm.sec  = msg->header.stamp.sec;
  m_contactROS_.tm.nsec = msg->header.stamp.nsec;
  m_contactROS_.data.length(msg->contacts.size());
  for(int i=0;i<msg->contacts.size();i++){
    if(this->URDFToVRMLLinkNameMap_.find(msg->contacts[i].pose.header.frame_id) != this->URDFToVRMLLinkNameMap_.end()){
      m_contactRTM_.data[i].link1 = this->URDFToVRMLLinkNameMap_[msg->contacts[i].pose.header.frame_id].c_str();
    }else{
      m_contactRTM_.data[i].link1 = msg->contacts[i].pose.header.frame_id.c_str();
    }
    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(msg->contacts[i].pose.pose, pose);
    eigen_rtm_conversions::poseEigenToRTM(pose, m_contactRTM_.data[i].local_pose);
    if(this->URDFToVRMLLinkNameMap_.find(msg->contacts[i].link) != this->URDFToVRMLLinkNameMap_.end()){
      m_contactRTM_.data[i].link2 = this->URDFToVRMLLinkNameMap_[msg->contacts[i].link].c_str();
    }else{
      m_contactRTM_.data[i].link2 = msg->contacts[i].pose.header.frame_id.c_str();
    }
    m_contactRTM_.data[i].free_x = msg->contacts[i].free_x;
    m_contactRTM_.data[i].free_y = msg->contacts[i].free_y;
  }

  m_contactROSOut_.write();
}

static const char* ContactROSBridge_spec[] = {
  "implementation_id", "ContactROSBridge",
  "type_name",         "ContactROSBridge",
  "description",       "ContactROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void ContactROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(ContactROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<ContactROSBridge>, RTC::Delete<ContactROSBridge>);
    }
};
