#include "TactileOdometry.h"
#include <cnoid/YAMLReader>
#include <cnoid/BodyLoader>
#include <cnoid/SceneGraph>
#include <cnoid/RateGyroSensor>
#include <cnoid/EigenUtil>

#include "MathUtil.h"

TactileOdometry::Ports::Ports() :
  m_qActIn_("qAct", m_qAct_),
  m_actImuIn_("actImuIn", m_actImu_),
  m_tactileSensorIn_("tactileSensorIn", m_tactileSensor_),

  m_TactileOdometryServicePort_("TactileOdometryService") {
}

void TactileOdometry::Ports::onInitialize(TactileOdometry* component) {
  component->addInPort("tactileSensorIn", this->m_tactileSensorIn_);
  component->addInPort("qAct", this->m_qActIn_);
  component->addInPort("actImuIn", this->m_actImuIn_);

  this->m_TactileOdometryServicePort_.registerProvider("service0", "TactileOdometryService", this->m_service0_);
  component->addPort(this->m_TactileOdometryServicePort_);
  return;
}

TactileOdometry::TactileOdometry(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  ports_()
{
  this->ports_.m_service0_.setComponent(this);
}

RTC::ReturnCode_t TactileOdometry::onInitialize(){
  this->ports_.onInitialize(this);

  {
    // load robot model
    cnoid::BodyLoader bodyLoader;
    std::string fileName; this->getProperty("model", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    cnoid::BodyPtr robot = bodyLoader.load(fileName);
    if(!robot){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    this->prevRobot_ = robot;
    this->curRobot_ = robot->clone();
  }


  // load tactile_sensor_file
  {
    std::string fileName;
    if(this->getProperties().hasKey("tactile_sensor_file")) fileName = std::string(this->getProperties()["tactile_sensor_file"]);
    else fileName = std::string(this->m_pManager->getConfig()["tactile_sensor_file"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] tactile_sensor_file: " << fileName <<std::endl;
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    try {
      node = reader.loadDocument(fileName)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << ex.message() << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    // load
    cnoid::ListingPtr tactileSensorList = node->findListing("tactile_sensor");
    if (!tactileSensorList->isValid()) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "cannot load config file" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }

    for (int i=0; i< tactileSensorList->size(); i++) {
      cnoid::Mapping* info = tactileSensorList->at(i)->toMapping();
      TactileOdometry::TactileSensor sensor;
      // name
      info->extract("name", sensor.name);
      // link
      info->extract("link", sensor.linkName);
      if(sensor.linkName == ""){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "link name is not specified" << "\e[0m" << std::endl;
        continue;
      }
      if(this->curRobot_->link(sensor.linkName)){
        sensor.curLink = this->curRobot_->link(sensor.linkName);
        sensor.prevLink = this->prevRobot_->link(sensor.linkName);
      }else{
        for(int l=0;l<this->curRobot_->numLinks() && !(sensor.curLink);l++){
          cnoid::SgGroup* shape = this->curRobot_->link(l)->shape();
          for(int j=0;j<shape->numChildObjects();j++){
            if(shape->child(j)->name() == sensor.linkName){
              sensor.curLink = this->curRobot_->link(l);
              sensor.prevLink = this->prevRobot_->link(l);
              break;
            }
          }
        }
      }
      if (!(sensor.curLink)) {
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link " << sensor.linkName << " not found" << "\e[0m" << std::endl;
        continue;
      }
      // translation
      cnoid::ValueNodePtr translation_ = info->extract("translation");
      if(translation_){
        cnoid::ListingPtr translationTmp = translation_->toListing();
        if(translationTmp->size()==3){
          sensor.translation = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
        }
      }
      // rotation
      cnoid::ValueNodePtr rotation_ = info->extract("rotation");
      if(rotation_){
        cnoid::ListingPtr rotationTmp = rotation_->toListing();
        if(rotationTmp->size() == 4){
          sensor.rotation = cnoid::AngleAxisd(rotationTmp->at(3)->toDouble(),
                                              cnoid::Vector3{rotationTmp->at(0)->toDouble(), rotationTmp->at(1)->toDouble(), rotationTmp->at(2)->toDouble()}).toRotationMatrix();
        }
      }
      this->tactileSensors_.push_back(sensor);
    }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileOdometry::onActivated(RTC::UniqueId ec_id) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileOdometry::onDeactivated(RTC::UniqueId ec_id) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileOdometry::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::string instance_name = std::string(this->m_profile.instance_name);

  if(!TactileOdometry::readInPortData(instance_name, this->ports_, this->curRobot_, this->tactileSensors_)) return RTC::RTC_OK;  // q が届かなければ何もしない

  TactileOdometry::writeOutPortData(instance_name, this->ports_);

  return RTC::RTC_OK;
}

bool TactileOdometry::setTactileOdometryParam(const contact_state_rtc::TactileOdometryService::TactileOdometryParam& i_param){
  return true;
}

bool TactileOdometry::getTactileOdometryParam(contact_state_rtc::TactileOdometryService::TactileOdometryParam& o_param){
  return true;
}

bool TactileOdometry::setRobotPos(const RTC::Point3D& pos) {
  return true;
}

bool TactileOdometry::setRobotPose(const RTC::Pose3D& pose){
  return true;
}

bool TactileOdometry::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
  return true;
}

bool TactileOdometry::readInPortData(const std::string& instance_name, TactileOdometry::Ports& ports, cnoid::BodyPtr curRobot, std::vector<TactileSensor>& tactileSensors) {
  bool qAct_updated = false;
  if(ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
    qAct_updated = true;
    if(ports.m_qAct_.data.length() == curRobot->numJoints()){
      for(int i=0;i<ports.m_qAct_.data.length();i++){
        if(std::isfinite(ports.m_qAct_.data[i])) curRobot->joint(i)->q() = ports.m_qAct_.data[i];
        else std::cerr << "m_qAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_actImuIn_.isNew()){
    ports.m_actImuIn_.read();
    if(std::isfinite(ports.m_actImu_.data.r) && std::isfinite(ports.m_actImu_.data.p) && std::isfinite(ports.m_actImu_.data.y)){
      curRobot->calcForwardKinematics();
      cnoid::RateGyroSensorPtr imu = curRobot->findDevice<cnoid::RateGyroSensor>("gyrometer");
      cnoid::Matrix3 tmpImuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actImuRraw = cnoid::rotFromRpy(ports.m_actImu_.data.r, ports.m_actImu_.data.p, ports.m_actImu_.data.y);
      cnoid::Matrix3 actImuR = mathutil::orientCoordToAxis(tmpImuR, actImuRraw * cnoid::Vector3::UnitZ());
      curRobot->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actImuR) * Eigen::AngleAxisd(tmpImuR.transpose() * curRobot->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
    }else{
      std::cerr << "[" << instance_name << "] " << "m_actImu is not finite!" << std::endl;
    }
  }
  curRobot->calcForwardKinematics();

  if(ports.m_tactileSensorIn_.isNew()){
    ports.m_tactileSensorIn_.read();
    if(ports.m_tactileSensor_.data.length() == tactileSensors.size() * 3){
      for (int i=0; i<tactileSensors.size(); i++) {
        // TODO threshould
        tactileSensors[i].isContact = (ports.m_tactileSensor_.data[i*3+2] == 0.0);
      }
    }else{
      std::cerr << "[" << instance_name << "] " << "tactile sensor dimension mismatch!" << std::endl;
    }
  }
  return qAct_updated;
}

bool TactileOdometry::writeOutPortData(const std::string& instance_name, TactileOdometry::Ports& ports) {
  return true;
}

static const char* TactileOdometry_spec[] = {
  "implementation_id", "TactileOdometry",
  "type_name",         "TactileOdometry",
  "description",       "TactileOdometry component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};


extern "C"{
  void TactileOdometryInit(RTC::Manager* manager) {
    RTC::Properties profile(TactileOdometry_spec);
    manager->registerFactory(profile, RTC::Create<TactileOdometry>, RTC::Delete<TactileOdometry>);
  }
};
