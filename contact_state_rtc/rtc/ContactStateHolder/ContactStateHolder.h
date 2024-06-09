#ifndef ContactStateHolder_H
#define ContactStateHolder_H

#include <memory>
#include <mutex>
#include <unordered_map>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>

#include "ContactStateHolderService_impl.h"

#include <cnoid/Body>
#include <cnoid/SceneGraph>

#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <ik_constraint2/ik_constraint2.h>

#include <contact_state_msgs/idl/ContactState.hh>

#include "tactile_shm.h"

class ContactStateHolder : public RTC::DataFlowComponentBase{
public:
  ContactStateHolder(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool setContactStateHolderParam(const contact_state_rtc::ContactStateHolderService::ContactStateHolderParam& i_param);
  bool getContactStateHolderParam(contact_state_rtc::ContactStateHolderService::ContactStateHolderParam& o_param);
  bool setRobotPose(const RTC::Pose3D& pose);

protected:
  std::mutex mutex_;

  std::unordered_map<std::string, std::string> URDFToVRMLLinkNameMap_;
  std::unordered_map<std::string, std::string> VRMLToURDFLinkNameMap_;

  class Ports {
  public:
    Ports();
    void onInitialize(ContactStateHolder* component);
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedOrientation3D m_actImu_; // Actual Imu World Frame. robotのgyrometerという名のRateGyroSensorの傾きを表す
    RTC::InPort<RTC::TimedOrientation3D> m_actImuIn_;
    RTC::TimedDoubleSeq m_tactileSensor_;
    RTC::InPort<RTC::TimedDoubleSeq> m_tactileSensorIn_;
    RTC::TimedDoubleSeq m_dqOdom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_dqOdomOut_;
    RTC::TimedPose3D m_odomBasePose_;
    RTC::OutPort<RTC::TimedPose3D> m_odomBasePoseOut_;
    RTC::TimedDoubleSeq m_odomBaseTform_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_odomBaseTformOut_; // for HrpsysSeqStateROSBridge
    RTC::TimedVelocity3D m_odomBaseVel_;
    RTC::OutPort<RTC::TimedVelocity3D> m_odomBaseVelOut_;
    contact_state_msgs::TimedContactSeq m_contactState_;
    RTC::OutPort<contact_state_msgs::TimedContactSeq> m_contactStateOut_;
    ContactStateHolderService_impl m_service0_;
    RTC::CorbaPort m_ContactStateHolderServicePort_;

    struct tactile_shm *t_shm_ = nullptr;
  };
  Ports ports_;

  class TactileSensor {
  public:
    // from config
    std::string name = "";
    std::string linkName = ""; // 親リンク名 (!= ジョイント名)
    cnoid::LinkPtr prevLink = nullptr;  // 親リンク
    cnoid::LinkPtr curLink = nullptr;  // 親リンク
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity(); // リンク座標系でどこに取り付けられているか．センサzがリンク内側方向

    std::shared_ptr<ik_constraint2::PositionConstraint> ikc = std::make_shared<ik_constraint2::PositionConstraint>(); // A_linkにcurLink. A_localposにlocalPose. Bは未定義.

    // from sensor
    bool isContact = false;
  };
  std::vector<TactileSensor> tactileSensors_;

  class ContactState {
  public:
    cnoid::LinkPtr prevLink1;
    cnoid::LinkPtr curLink1;
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr prevLink2;
    cnoid::LinkPtr curLink2;
    bool freeX = false;
    bool freeY = false;

    std::shared_ptr<ik_constraint2::PositionConstraint> ikc;
  };
  std::vector<ContactState> contactStates_;

  cnoid::BodyPtr prevRobot_;
  cnoid::BodyPtr curRobot_;
  std::vector<cpp_filters::FirstOrderLowPassFilter<double> > dqOdom_; // cutoffを2loopぶんにするために、passFilterのdtは常に1/2[s], cutOffは1[Hz]とする.
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> odomBaseVel_ = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>(3.5, cnoid::Vector6::Zero());

protected:
  bool getProperty(const std::string& key, std::string& ret);

  static bool curRobot2PrevRobot(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> curRobot,
                                 cnoid::BodyPtr prevRobot);
  static bool readInPortData(const std::string& instance_name, ContactStateHolder::Ports& ports,
                             cnoid::BodyPtr curRobot, std::vector<TactileSensor>& tactileSensors);
  static bool calcContactState(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> prevRobot, const std::vector<TactileSensor>& tactileSensors,
                               std::vector<ContactState>& contactStates);
  static bool calcOdometry(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> prevRobot, const std::vector<ContactState>& contactStates,
                           cnoid::BodyPtr curRobot);
  static bool calcVelocity(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> prevRobot, double dt,
                           cnoid::BodyPtr curRobot,
                           std::vector<cpp_filters::FirstOrderLowPassFilter<double> >& dqOdom, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& odomBaseVel);
  static bool writeOutPortData(const std::string& instance_name, const std::unordered_map<std::string, std::string>& VRMLToURDFLinkNameMap, cnoid::ref_ptr<const cnoid::Body> curRobot, const std::vector<ContactState>& contactStates,
                               ContactStateHolder::Ports& ports);
};

extern "C"
{
  void ContactStateHolderInit(RTC::Manager* manager);
}

#endif // ContactStateHolder_H
