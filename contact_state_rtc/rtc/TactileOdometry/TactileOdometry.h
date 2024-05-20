#ifndef TactileOdometry_H
#define TactileOdometry_H

#include <memory>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>

#include "TactileOdometryService_impl.h"

class TactileOdometry : public RTC::DataFlowComponentBase{
public:
  TactileOdometry(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool setTactileOdometryParam(const contact_state_rtc::TactileOdometryService::TactileOdometryParam& i_param);
  bool getTactileOdometryParam(contact_state_rtc::TactileOdometryService::TactileOdometryParam& o_param);
  bool setRobotPos(const RTC::Point3D& pos);
  bool setRobotPose(const RTC::Pose3D& pose);

protected:
  RTC::TimedDoubleSeq m_tactileSensorArray_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tactileSensorArrayIn_;
  TactileOdometryService_impl m_service0_;
  RTC::CorbaPort m_TactileOdometryServicePort_;
};

extern "C"
{
  void TactileOdometryInit(RTC::Manager* manager);
}

#endif // TactileOdometry_H
