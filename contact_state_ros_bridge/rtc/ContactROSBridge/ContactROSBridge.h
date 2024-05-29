#ifndef ContactROSBridge_H
#define ContactROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <contact_state_msgs/idl/ContactState.hh>

#include <ros/ros.h>
#include <contact_state_msgs/ContactArray.h>

class ContactROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh; // これがないとうまく通信できなくなったり、CPU使用率100%になったりする

  std::string tf_prefix_;

  contact_state_msgs::TimedContactSeq m_contactRTM_;
  RTC::InPort <contact_state_msgs::TimedContactSeq> m_contactRTMIn_;
  ros::Publisher pub_;

  ros::Subscriber sub_;
  contact_state_msgs::TimedContactSeq m_contactROS_;
  RTC::OutPort <contact_state_msgs::TimedContactSeq> m_contactROSOut_;

public:
  ContactROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void topicCallback(const contact_state_msgs::ContactArray::ConstPtr& msg);
};


extern "C"
{
  void ContactROSBridgeInit(RTC::Manager* manager);
};

#endif // ContactROSBridge_H
