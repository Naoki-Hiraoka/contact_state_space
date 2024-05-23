#include "ContactStateHolderService_impl.h"
#include "ContactStateHolder.h"

ContactStateHolderService_impl::ContactStateHolderService_impl()
{
}

ContactStateHolderService_impl::~ContactStateHolderService_impl()
{
}

void ContactStateHolderService_impl::setComponent(ContactStateHolder *i_component)
{
  this->component_ = i_component;
}

CORBA::Boolean ContactStateHolderService_impl::setContactStateHolderParam(const contact_state_rtc::ContactStateHolderService::ContactStateHolderParam& i_param)
{
  return this->component_->setContactStateHolderParam(i_param);
};

CORBA::Boolean ContactStateHolderService_impl::getContactStateHolderParam(contact_state_rtc::ContactStateHolderService::ContactStateHolderParam_out o_param)
{
  o_param = new contact_state_rtc::ContactStateHolderService::ContactStateHolderParam();
  return this->component_->getContactStateHolderParam(*o_param);
};

CORBA::Boolean ContactStateHolderService_impl::setRobotPos(const RTC::Point3D& pos) {
  return this->component_->setRobotPos(pos);
}

CORBA::Boolean ContactStateHolderService_impl::setRobotPose(const RTC::Pose3D& pose) {
  return this->component_->setRobotPose(pose);
}

