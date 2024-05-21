#ifndef TactileOdometry_H
#define TactileOdometry_H

#include <memory>
#include <mutex>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>

#include "TactileOdometryService_impl.h"

#include <cnoid/Body>

#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <ik_constraint2/ik_constraint2.h>

class TactileOdometry : public RTC::DataFlowComponentBase{
public:
  TactileOdometry(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool setTactileOdometryParam(const contact_state_rtc::TactileOdometryService::TactileOdometryParam& i_param);
  bool getTactileOdometryParam(contact_state_rtc::TactileOdometryService::TactileOdometryParam& o_param);
  bool setRobotPos(const RTC::Point3D& pos);
  bool setRobotPose(const RTC::Pose3D& pose);

protected:
  std::mutex mutex_;

  class Ports {
  public:
    Ports();
    void onInitialize(TactileOdometry* component);
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedOrientation3D m_actImu_; // Actual Imu World Frame. robotのgyrometerという名のRateGyroSensorの傾きを表す
    RTC::InPort<RTC::TimedOrientation3D> m_actImuIn_;
    RTC::TimedDoubleSeq m_tactileSensor_;
    RTC::InPort<RTC::TimedDoubleSeq> m_tactileSensorIn_;
    RTC::TimedPose3D m_odomBasePose_;
    RTC::OutPort<RTC::TimedPose3D> m_odomBasePoseOut_;
    RTC::TimedVelocity3D m_odomBaseVel_;
    RTC::OutPort<RTC::TimedVelocity3D> m_odomBaseVelOut_;
    TactileOdometryService_impl m_service0_;
    RTC::CorbaPort m_TactileOdometryServicePort_;
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

    std::shared_ptr<ik_constraint2::PositionConstraint> ikc; // A_linkにcurRobot. B_linkにworld.

    // from sensor
    bool isContact = false;
  };
  std::vector<TactileSensor> tactileSensors_;

  cnoid::BodyPtr prevRobot_;
  cnoid::BodyPtr curRobot_;
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> odomBaseVel_ = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>(3.5, cnoid::Vector6::Zero());

protected:
  bool getProperty(const std::string& key, std::string& ret);

  static bool curRobot2PrevRobot(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> curRobot, cnoid::BodyPtr prevRobot);
  static bool readInPortData(const std::string& instance_name, TactileOdometry::Ports& ports, cnoid::BodyPtr curRobot, std::vector<TactileSensor>& tactileSensors);
  static bool calcOdometry(const std::string& instance_name, cnoid::BodyPtr curRobot, cnoid::ref_ptr<const cnoid::Body> prevRobot, std::vector<TactileSensor>& tactileSensors);
  static bool calcVelocity(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> curRobot, cnoid::ref_ptr<const cnoid::Body> prevRobot, double dt, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& odomBaseVel);
  static bool writeOutPortData(const std::string& instance_name, TactileOdometry::Ports& ports, cnoid::ref_ptr<const cnoid::Body> curRobot, const cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& odomBaseVel);
};

extern "C"
{
  void TactileOdometryInit(RTC::Manager* manager);
}

#endif // TactileOdometry_H
