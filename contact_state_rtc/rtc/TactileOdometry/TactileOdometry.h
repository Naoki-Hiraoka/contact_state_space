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
    cnoid::Vector3 translation = cnoid::Vector3::Zero(); // リンク座標系でどこに取り付けられているか
    cnoid::Matrix3 rotation = cnoid::Matrix3::Identity(); // リンク座標系でセンサの姿勢．zがリンク内側方向

    // from sensor
    bool isContact = false;
  };
  std::vector<TactileSensor> tactileSensors_;

  cnoid::BodyPtr prevRobot_;
  cnoid::BodyPtr curRobot_;

protected:
  bool getProperty(const std::string& key, std::string& ret);
  static bool readInPortData(const std::string& instance_name, TactileOdometry::Ports& ports, cnoid::BodyPtr curRobot, std::vector<TactileSensor>& tactileSensors);
  static bool writeOutPortData(const std::string& instance_name, TactileOdometry::Ports& ports);
};

extern "C"
{
  void TactileOdometryInit(RTC::Manager* manager);
}

#endif // TactileOdometry_H
