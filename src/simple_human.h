#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>
#include <mc_robots/api.h>

namespace mc_robots
{

  struct MC_ROBOTS_DLLAPI SimpleHumanRobotModule : public mc_rbdyn::RobotModule
  {
  public:
    SimpleHumanRobotModule();

  protected:
    std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;
    std::vector< std::map<std::string, std::vector<double> > > nominalBounds(const rbd::parsers::Limits & limits) const;
    std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;
    std::map<std::string, std::pair<std::string, std::string> > getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const;

  public:
    std::vector<std::string> virtualLinks;
    std::map< std::string, std::vector<double> > halfSitting;
    rbd::parsers::Limits limits;
    std::vector<std::string> excludedLinks;
    std::vector<std::string> gripperJoints;
  };

} // namespace mc_robots


extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"simple_human"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
  {
    ROBOT_MODULE_CHECK_VERSION("simple_human")
    if(name == "simple_human")
    {
      return new mc_robots::SimpleHumanRobotModule();
    }

  }
}
