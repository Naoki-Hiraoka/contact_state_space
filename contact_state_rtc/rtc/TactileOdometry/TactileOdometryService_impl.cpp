#include "TactileOdometryService_impl.h"
#include "TactileOdometry.h"

TactileOdometryService_impl::TactileOdometryService_impl()
{
}

TactileOdometryService_impl::~TactileOdometryService_impl()
{
}

void TactileOdometryService_impl::setComponent(TactileOdometry *i_component)
{
  this->component_ = i_component;
}

CORBA::Boolean TactileOdometryService_impl::setTactileOdometryParam(const contact_state_rtc::TactileOdometryService::TactileOdometryParam& i_param)
{
  return this->component_->setTactileOdometryParam(i_param);
};

CORBA::Boolean TactileOdometryService_impl::getTactileOdometryParam(contact_state_rtc::TactileOdometryService::TactileOdometryParam_out o_param)
{
  o_param = new contact_state_rtc::TactileOdometryService::TactileOdometryParam();
  return this->component_->getTactileOdometryParam(*o_param);
};

CORBA::Boolean TactileOdometryService_impl::setRobotPos(const RTC::Point3D& pos) {
  return this->component_->setRobotPos(pos);
}

CORBA::Boolean TactileOdometryService_impl::setRobotPose(const RTC::Pose3D& pose) {
  return this->component_->setRobotPose(pose);
}

