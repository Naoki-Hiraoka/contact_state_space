#ifndef ContactStateHolder_SERVICE_IMPL_H
#define ContactStateHolder_SERVICE_IMPL_H

#include "contact_state_rtc/idl/ContactStateHolderService.hh"

class ContactStateHolder;

class ContactStateHolderService_impl
  : public virtual POA_contact_state_rtc::ContactStateHolderService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ContactStateHolderService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  virtual ~ContactStateHolderService_impl();

  CORBA::Boolean setContactStateHolderParam(const contact_state_rtc::ContactStateHolderService::ContactStateHolderParam& i_param);
  CORBA::Boolean getContactStateHolderParam(contact_state_rtc::ContactStateHolderService::ContactStateHolderParam_out o_param);
  CORBA::Boolean setRobotPose(const RTC::Pose3D& pose);

  void setComponent(ContactStateHolder *i_component);
protected:
  ContactStateHolder *component_;
};

#endif
