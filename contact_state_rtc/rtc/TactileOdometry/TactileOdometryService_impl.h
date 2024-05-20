#ifndef TactileOdometry_SERVICE_IMPL_H
#define TactileOdometry_SERVICE_IMPL_H

#include "contact_state_rtc/idl/TactileOdometryService.hh"

class TactileOdometry;

class TactileOdometryService_impl
  : public virtual POA_contact_state_rtc::TactileOdometryService,
    public virtual PortableServer::RefCountServantBase
{
public:
  TactileOdometryService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  virtual ~TactileOdometryService_impl();

  CORBA::Boolean setTactileOdometryParam(const contact_state_rtc::TactileOdometryService::TactileOdometryParam& i_param);
  CORBA::Boolean getTactileOdometryParam(contact_state_rtc::TactileOdometryService::TactileOdometryParam_out o_param);
  CORBA::Boolean setRobotPos(const RTC::Point3D& pos);
  CORBA::Boolean setRobotPose(const RTC::Pose3D& pose);

  void setComponent(TactileOdometry *i_component);
protected:
  TactileOdometry *component_;
};

#endif
