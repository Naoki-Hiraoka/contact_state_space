#include <rtm/Manager.h>
#include "ContactStateHolder.h"

void MyModuleInit(RTC::Manager* manager)
{
  ContactStateHolderInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("ContactStateHolder"); // skip root name space for OpenRTM instance name

  return;

}

int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}
