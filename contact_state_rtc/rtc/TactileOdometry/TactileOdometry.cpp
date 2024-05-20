#include "TactileOdometry.h"
#include <cnoid/YAMLReader>

TactileOdometry::TactileOdometry(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_tactileSensorArrayIn_("tactileSensorArrayIn", m_tactileSensorArray_),

  m_TactileOdometryServicePort_("TactileOdometryService")
{
  this->m_service0_.setComponent(this);
}

RTC::ReturnCode_t TactileOdometry::onInitialize(){
  addInPort("estWrenchesIn", this->m_tactileSensorArrayIn_);

  this->m_TactileOdometryServicePort_.registerProvider("service0", "TactileOdometryService", this->m_service0_);
  this->addPort(this->m_TactileOdometryServicePort_);

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
    //this->num_sensor = tactileSensorList->size();
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileOdometry::onExecute(RTC::UniqueId ec_id){
  if(this->m_tactileSensorArrayIn_.isNew()){
    this->m_tactileSensorArrayIn_.read();
  }

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
