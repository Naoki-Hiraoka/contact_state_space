#include "ContactStateHolder.h"
#include <cnoid/YAMLReader>
#include <cnoid/BodyLoader>
#include <cnoid/SceneGraph>
#include <cnoid/RateGyroSensor>
#include <cnoid/EigenUtil>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>
#include <sr_inverse_kinematics_solver/sr_inverse_kinematics_solver.h>

#include "MathUtil.h"

ContactStateHolder::Ports::Ports() :
  m_qActIn_("qAct", m_qAct_),
  m_actImuIn_("actImuIn", m_actImu_),
  m_tactileSensorIn_("tactileSensorIn", m_tactileSensor_),
  m_dqOdomOut_("dqOdomOut", m_dqOdom_),
  m_odomBasePoseOut_("odomBasePoseOut", m_odomBasePose_),
  m_odomBaseTformOut_("odomBaseTformOut", m_odomBaseTform_),
  m_odomBaseVelOut_("odomBaseVelOut", m_odomBaseVel_),
  m_contactStateOut_("contactStateOut", m_contactState_),

  m_ContactStateHolderServicePort_("ContactStateHolderService") {
}

void ContactStateHolder::Ports::onInitialize(ContactStateHolder* component) {
  component->addInPort("tactileSensorIn", this->m_tactileSensorIn_);
  component->addInPort("qAct", this->m_qActIn_);
  component->addInPort("actImuIn", this->m_actImuIn_);
  component->addOutPort("dqOdomOut", this->m_dqOdomOut_);
  component->addOutPort("odomBasePoseOut", this->m_odomBasePoseOut_);
  component->addOutPort("odomBaseTformOut", this->m_odomBaseTformOut_);
  component->addOutPort("odomBaseVelOut", this->m_odomBaseVelOut_);
  component->addOutPort("contactStateOut", this->m_contactStateOut_);

  this->m_ContactStateHolderServicePort_.registerProvider("service0", "ContactStateHolderService", this->m_service0_);
  component->addPort(this->m_ContactStateHolderServicePort_);
  return;
}

ContactStateHolder::ContactStateHolder(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  ports_()
{
  this->ports_.m_service0_.setComponent(this);
}

RTC::ReturnCode_t ContactStateHolder::onInitialize(){
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
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

  for(int l=0;l<this->curRobot_->numLinks() ; l++){
    cnoid::SgGroup* shape = this->curRobot_->link(l)->shape();
    if(shape && shape->numChildObjects() > 0 && shape->child(0)->name().size()!=0){
      this->VRMLToURDFLinkNameMap_[this->curRobot_->link(l)->name()] = shape->child(0)->name();
      this->URDFToVRMLLinkNameMap_[shape->child(0)->name()] = this->curRobot_->link(l)->name();
    }
  }

  this->dqOdom_.resize(this->curRobot_->numJoints(), cpp_filters::FirstOrderLowPassFilter<double>(1.0, 0.0));

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
      ContactStateHolder::TactileSensor sensor;
      // name
      info->extract("name", sensor.name);
      // link
      info->extract("link", sensor.linkName);
      if(sensor.linkName == ""){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "link name is not specified" << "\e[0m" << std::endl;
        continue;
      }
      if(this->URDFToVRMLLinkNameMap_.find(sensor.linkName) != this->URDFToVRMLLinkNameMap_.end()){
        std::string linkName = this->URDFToVRMLLinkNameMap_[sensor.linkName];
        sensor.curLink = this->curRobot_->link(linkName);
        sensor.prevLink = this->prevRobot_->link(linkName);
      }else{
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link " << sensor.linkName << " not found" << "\e[0m" << std::endl;
        continue;
      }
      // translation
      cnoid::ValueNodePtr translation_ = info->extract("translation");
      if(translation_){
        cnoid::ListingPtr translationTmp = translation_->toListing();
        if(translationTmp->size()==3){
          sensor.localPose.translation() = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
        }
      }
      // rotation
      cnoid::ValueNodePtr rotation_ = info->extract("rotation");
      if(rotation_){
        cnoid::ListingPtr rotationTmp = rotation_->toListing();
        if(rotationTmp->size() == 4){
          sensor.localPose.linear() = cnoid::AngleAxisd(rotationTmp->at(3)->toDouble(),
                                                        cnoid::Vector3{rotationTmp->at(0)->toDouble(), rotationTmp->at(1)->toDouble(), rotationTmp->at(2)->toDouble()}).toRotationMatrix();
        }
      }

      // constraint
      sensor.ikc->A_link() = sensor.curLink;
      sensor.ikc->A_localpos() = sensor.localPose;

      this->tactileSensors_.push_back(sensor);
    }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ContactStateHolder::onActivated(RTC::UniqueId ec_id) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ContactStateHolder::onDeactivated(RTC::UniqueId ec_id) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ContactStateHolder::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  const std::string instance_name = std::string(this->m_profile.instance_name);
  const double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // curRobotをprevRobotへコピー
  ContactStateHolder::curRobot2PrevRobot(instance_name, this->curRobot_,
                                         this->prevRobot_);

  // InPortの値をcurRobotへ反映
  if(!ContactStateHolder::readInPortData(instance_name, this->ports_,
                                         this->curRobot_, this->tactileSensors_)) {
    return RTC::RTC_OK;  // q が届かなければ何もしない
  }

  ContactStateHolder::calcContactState(instance_name, this->prevRobot_, this->tactileSensors_,
                                       this->contactStates_);

  // Odometryを計算. curRobotの姿勢を更新
  ContactStateHolder::calcOdometry(instance_name, this->prevRobot_, this->contactStates_,
                                   this->curRobot_);

  // curRobotとprevRobotから速度を計算.
  ContactStateHolder::calcVelocity(instance_name, this->prevRobot_, dt,
                                   this->curRobot_,
                                   this->dqOdom_, this->odomBaseVel_);

  // curRobotと速度を出力
  ContactStateHolder::writeOutPortData(instance_name, this->VRMLToURDFLinkNameMap_, this->curRobot_, this->contactStates_, this->ports_);

  return RTC::RTC_OK;
}

bool ContactStateHolder::setContactStateHolderParam(const contact_state_rtc::ContactStateHolderService::ContactStateHolderParam& i_param){
  return true;
}

bool ContactStateHolder::getContactStateHolderParam(contact_state_rtc::ContactStateHolderService::ContactStateHolderParam& o_param){
  return true;
}

bool ContactStateHolder::setRobotPos(const RTC::Point3D& pos) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) return false;
  eigen_rtm_conversions::pointRTMToEigen(pos, this->curRobot_->rootLink()->p());
  return true;
}

bool ContactStateHolder::setRobotPose(const RTC::Pose3D& pose){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z)) return false;
  if(!std::isfinite(pose.orientation.r) || !std::isfinite(pose.orientation.p) || !std::isfinite(pose.orientation.y)) return false;
  eigen_rtm_conversions::poseRTMToEigen(pose, this->curRobot_->rootLink()->T());
  return true;
}

bool ContactStateHolder::getProperty(const std::string& key, std::string& ret) {
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

bool ContactStateHolder::curRobot2PrevRobot(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> curRobot, cnoid::BodyPtr prevRobot) {
  prevRobot->rootLink()->T() = curRobot->rootLink()->T();
  for(int i=0;i<curRobot->numJoints();i++) {
    prevRobot->joint(i)->q() = curRobot->joint(i)->q();
  }
  prevRobot->calcForwardKinematics();

  return true;
}

bool ContactStateHolder::readInPortData(const std::string& instance_name, ContactStateHolder::Ports& ports, cnoid::BodyPtr curRobot, std::vector<TactileSensor>& tactileSensors) {
  bool qAct_updated = false;
  if(ports.m_qActIn_.isNew()){
    while(ports.m_qActIn_.isNew()) ports.m_qActIn_.read();
    qAct_updated = true;
    if(ports.m_qAct_.data.length() == curRobot->numJoints()){
      for(int i=0;i<ports.m_qAct_.data.length();i++){
        if(std::isfinite(ports.m_qAct_.data[i])) curRobot->joint(i)->q() = ports.m_qAct_.data[i];
        else std::cerr << "m_qAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_actImuIn_.isNew()){
    while(ports.m_actImuIn_.isNew()) ports.m_actImuIn_.read();
    if(std::isfinite(ports.m_actImu_.data.r) && std::isfinite(ports.m_actImu_.data.p) && std::isfinite(ports.m_actImu_.data.y)){
      curRobot->calcForwardKinematics();
      cnoid::RateGyroSensorPtr imu = curRobot->findDevice<cnoid::RateGyroSensor>("gyrometer");
      cnoid::Matrix3 prevImuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actImuRraw; eigen_rtm_conversions::orientationRTMToEigen(ports.m_actImu_.data, actImuRraw);
      cnoid::Matrix3 actImuR = mathutil::orientCoordToAxis(prevImuR, cnoid::Vector3::UnitZ(), actImuRraw.transpose() * cnoid::Vector3::UnitZ());
      curRobot->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actImuR) * Eigen::AngleAxisd(prevImuR.transpose() * curRobot->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
    }else{
      std::cerr << "[" << instance_name << "] " << "m_actImu is not finite!" << std::endl;
    }
  }
  curRobot->calcForwardKinematics();

  if(ports.m_tactileSensorIn_.isNew()){
    while(ports.m_tactileSensorIn_.isNew()) ports.m_tactileSensorIn_.read();
    if(ports.m_tactileSensor_.data.length() == tactileSensors.size() * 3){
      for (int i=0; i<tactileSensors.size(); i++) {
        // TODO threshould
        tactileSensors[i].isContact = (ports.m_tactileSensor_.data[i*3+2] != 0.0);
      }
    }else{
      std::cerr << "[" << instance_name << "] " << "tactile sensor dimension mismatch!" << std::endl;
    }
  }
  return qAct_updated;
}
bool ContactStateHolder::calcContactState(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> prevRobot, const std::vector<TactileSensor>& tactileSensors, std::vector<ContactState>& contactStates) {
  contactStates.clear();
  for(int i=0;i<tactileSensors.size();i++){
    if(!tactileSensors[i].isContact) continue;
    ContactState contact;
    contact.prevLink1 = tactileSensors[i].prevLink;
    contact.curLink1 = tactileSensors[i].curLink;
    contact.localPose1 = tactileSensors[i].localPose;
    contact.prevLink2 = nullptr;
    contact.curLink2 = nullptr;
    contact.freeX = false;
    contact.freeY = false;
    contact.ikc = tactileSensors[i].ikc;
    contactStates.push_back(contact);
  }

  return true;
}

bool ContactStateHolder::calcOdometry(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> prevRobot, const std::vector<ContactState>& contactStates, cnoid::BodyPtr curRobot) {

  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > ikc_list;
  for(int i=0;i<contactStates.size();i++){
    std::shared_ptr<ik_constraint2::PositionConstraint> ikc = contactStates[i].ikc;
    cnoid::Isometry3 pose1 = contactStates[i].prevLink1 ? contactStates[i].prevLink1->T() * contactStates[i].localPose1 : contactStates[i].localPose1; // world系.
    cnoid::Isometry3 localPose2 = contactStates[i].prevLink2 ? contactStates[i].prevLink2->T().inverse() * pose1 : pose1; // link2 local

    ikc->A_link() = contactStates[i].curLink1;
    ikc->A_localpos() = contactStates[i].localPose1;
    ikc->B_link() = contactStates[i].curLink2;
    ikc->B_localpos() = localPose2;
    ikc->eval_link() = contactStates[i].curLink1;
    ikc->eval_localR() = contactStates[i].localPose1.linear();
    ikc->weight() << (contactStates[i].freeX ? 0.0 : 1.0), (contactStates[i].freeY ? 0.0 : 1.0), 1.0, 0.0, 0.0, 0.0;
    //ikc->debugLevel() = 1;
    ikc_list.push_back(ikc);
  }

  std::vector<cnoid::LinkPtr> variables;
  variables.push_back(curRobot->rootLink());

  sr_inverse_kinematics_solver::IKParam param;
  param.maxIteration = 3;
  param.minIteration = 3;
  param.dqWeight = std::vector<double>{1.0, 1.0, 1.0, 0.0, 0.0, 1.0};
  //param.debugLevel = 1;

  sr_inverse_kinematics_solver::solveIKLoop(variables, ikc_list, param);

  return true;
}

bool ContactStateHolder::calcVelocity(const std::string& instance_name, cnoid::ref_ptr<const cnoid::Body> prevRobot, double dt,
                                      cnoid::BodyPtr curRobot,
                                      std::vector<cpp_filters::FirstOrderLowPassFilter<double> >& dqOdom, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& odomBaseVel) {
  {
    for(int i=0;i<curRobot->numJoints();i++){
      // cutoffを2loopぶんにするために、passFilterのdtは常に1/2[s], cutOffは1[Hz]とする.
      dqOdom[i].passFilter((curRobot->joint(i)->q() - prevRobot->joint(i)->q()) / dt, 0.5);
      curRobot->joint(i)->dq() = dqOdom[i].value();
    }
  }

  {
    cnoid::Isometry3 diff = curRobot->rootLink()->T() * prevRobot->rootLink()->T().inverse();
    cnoid::Vector6 vel;
    vel.head<3>() = diff.translation() / dt;
    cnoid::AngleAxisd dr(diff.linear());
    vel.tail<3>() = dr.axis() * dr.angle() / dt;
    odomBaseVel.passFilter(vel, dt);
    curRobot->rootLink()->v() = odomBaseVel.value().head<3>();
    curRobot->rootLink()->w() = odomBaseVel.value().tail<3>();
  }

  return true;
}

bool ContactStateHolder::writeOutPortData(const std::string& instance_name, const std::unordered_map<std::string, std::string>& VRMLToURDFLinkNameMap, cnoid::ref_ptr<const cnoid::Body> curRobot, const std::vector<ContactState>& contactStates,
                                          ContactStateHolder::Ports& ports) {
  ports.m_dqOdom_.tm = ports.m_qAct_.tm;
  ports.m_dqOdom_.data.length(curRobot->numJoints());
  for(int i=0;i<curRobot->numJoints();i++){
    ports.m_dqOdom_.data[i] = curRobot->joint(i)->dq();
  }
  ports.m_dqOdomOut_.write();

  ports.m_odomBasePose_.tm = ports.m_qAct_.tm;
  eigen_rtm_conversions::poseEigenToRTM(curRobot->rootLink()->T(), ports.m_odomBasePose_.data);
  ports.m_odomBasePoseOut_.write();

  ports.m_odomBaseTform_.tm = ports.m_qAct_.tm;
  ports.m_odomBaseTform_.data.length(12);
  for(int i=0;i<3;i++){
    ports.m_odomBaseTform_.data[i] = curRobot->rootLink()->p()[i];
  }
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      ports.m_odomBaseTform_.data[3+i*3+j] = curRobot->rootLink()->R()(i,j);// row major
    }
  }
  ports.m_odomBaseTformOut_.write();

  ports.m_odomBaseVel_.tm = ports.m_qAct_.tm;
  eigen_rtm_conversions::velocityEigenToRTM(curRobot->rootLink()->v(), curRobot->rootLink()->w(), ports.m_odomBaseVel_.data);
  ports.m_odomBaseVelOut_.write();

  ports.m_contactState_.tm = ports.m_qAct_.tm;
  ports.m_contactState_.data.length(contactStates.size());
  for(int i=0;i<contactStates.size();i++){
    ports.m_contactState_.data[i].link1 = contactStates[i].curLink1 ? VRMLToURDFLinkNameMap.find(contactStates[i].curLink1->name())->second.c_str() : "";
    eigen_rtm_conversions::poseEigenToRTM(contactStates[i].localPose1, ports.m_contactState_.data[i].local_pose);
    ports.m_contactState_.data[i].link2 = contactStates[i].curLink2 ? VRMLToURDFLinkNameMap.find(contactStates[i].curLink2->name())->second.c_str() : "";
    ports.m_contactState_.data[i].free_x = contactStates[i].freeX;
    ports.m_contactState_.data[i].free_y = contactStates[i].freeY;
  }
  ports.m_contactStateOut_.write();


  return true;
}

static const char* ContactStateHolder_spec[] = {
  "implementation_id", "ContactStateHolder",
  "type_name",         "ContactStateHolder",
  "description",       "ContactStateHolder component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};


extern "C"{
  void ContactStateHolderInit(RTC::Manager* manager) {
    RTC::Properties profile(ContactStateHolder_spec);
    manager->registerFactory(profile, RTC::Create<ContactStateHolder>, RTC::Delete<ContactStateHolder>);
  }
};
