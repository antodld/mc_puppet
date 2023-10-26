#include "simple_human.h"
#include "config.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <mc_rtc/logging.h>
#include <fstream>

#include <RBDyn/parsers/urdf.h>

namespace bfs = boost::filesystem;

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_robots
{
SimpleHumanRobotModule::SimpleHumanRobotModule()
 : RobotModule(SIMPLE_HUMAN_DESCRIPTION_PATH, "simple_human")
 {
  /* Path to surface descriptions */
  rsdf_dir = path + "/rsdf";

  /* Virtual links */
 	virtualLinks.push_back("base_link");
  virtualLinks.push_back("HEAD_R_LINK");
  virtualLinks.push_back("HEAD_P_LINK");
  virtualLinks.push_back("L_SHOULDER_R_LINK");
  virtualLinks.push_back("L_SHOULDER_P_LINK");
  virtualLinks.push_back("L_WRIST_R_LINK");
  virtualLinks.push_back("L_WRIST_P_LINK");
  virtualLinks.push_back("R_SHOULDER_R_LINK");
  virtualLinks.push_back("R_SHOULDER_P_LINK");
  virtualLinks.push_back("R_WRIST_R_LINK");
  virtualLinks.push_back("R_WRIST_P_LINK");

  /* Reference joint order */
  _ref_joint_order = {
 	"HEAD_R", // 0
  "HEAD_P", // 1
  "HEAD_Y", // 2
 	"L_SHOULDER_R", // 3
  "L_SHOULDER_P", // 4
  "L_SHOULDER_Y", // 5
 	"L_ELBOW_P", // 6
 	"L_WRIST_R", // 7
  "L_WRIST_P", // 8
  "L_WRIST_Y", // 9
 	"R_SHOULDER_R", // 10
  "R_SHOULDER_P", // 11
  "R_SHOULDER_Y", // 12
 	"R_ELBOW_P", // 13
 	"R_WRIST_R", // 14
  "R_WRIST_P", // 15
  "R_WRIST_Y", // 16
  };

  /* Default posture joint values in degrees */
  for (auto & j : _ref_joint_order)
  {
    halfSitting[j] = {0.0};
  }
  

  /* Read URDF file */
  init(rbd::parsers::from_urdf_file(urdf_path, false, excludedLinks));

  /* Collision hulls */
  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);

  /* Halfsit posture */
  _stance = halfSittingPose(mb);

  /* Critical self collisions */
  _minimalSelfCollisions = {
    // mc_rbdyn::Collision("HeadLink", "LArmLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("HeadLink", "LForearmLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("HeadLink", "LHandLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("HeadLink", "RArmLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("HeadLink", "RForearmLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("HeadLink", "RHandLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("TorsoLink", "LForearmLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("TorsoLink", "LHandLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("TorsoLink", "RForearmLink", 0.03, 0.01, 0.),
    // mc_rbdyn::Collision("TorsoLink", "RHandLink", 0.03, 0.01, 0.),
  };
  _commonSelfCollisions = _minimalSelfCollisions;

  /* Default kinematic tree root pose */
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.987}};

 }

  std::map<std::string, std::vector<double>> SimpleHumanRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
  {
    std::map<std::string, std::vector<double>> res;
    for (const auto & j : mb.joints())
    {
      if(halfSitting.count(j.name()))
      {
        res[j.name()] = halfSitting.at(j.name());
        for (auto & ji : res[j.name()])
        {
          ji = M_PI*ji / 180;
        }
      }
      else if(j.name() != "Root" && j.dof() > 0)
      {
        mc_rtc::log::warning("Joint {} has {} dof, but is not part of half sitting posture.", j.name(), j.dof());
      }
    }
    return res;
  }


  std::map<std::string, std::pair<std::string, std::string> > SimpleHumanRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
  {
    std::string convexPath = path + "/convex/";

    std::map<std::string, std::pair<std::string, std::string> > res;
    for(const auto & f : files)
    {
      bfs::path fpath = bfs::path(convexPath)/(f.second.second+"-ch.txt");
      if (bfs::exists(fpath))
      {
       res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
      }
    }
    return res;
  }


  std::vector< std::map<std::string, std::vector<double> > > SimpleHumanRobotModule::nominalBounds(const rbd::parsers::Limits & limits) const
  {
    std::vector< std::map<std::string, std::vector<double> > > res(0);
    res.push_back(limits.lower);
    res.push_back(limits.upper);
    {
      auto mvelocity = limits.velocity;
      for (auto & mv : mvelocity)
      {
        for (auto & mvi : mv.second)
        {
          mvi = -mvi;
        }
      }
      res.push_back(mvelocity);
    }
    res.push_back(limits.velocity);
    {
      auto mtorque = limits.torque;
      for (auto & mt : mtorque)
      {
        for (auto & mti : mt.second)
        {
          mti = -mti;
        }
      }
      res.push_back(mtorque);
    }
    res.push_back(limits.torque);
    return res;
  }

  std::map<std::string, std::pair<std::string, std::string>> SimpleHumanRobotModule::stdCollisionsFiles(const rbd::MultiBody &/*mb*/) const
  {
    std::map<std::string, std::pair<std::string, std::string>> res;
    for(const auto & b : mb.bodies())
    {
      // Filter out virtual links without convex files
      if(std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
      {
        res[b.name()] = {b.name(), b.name()};
      }
    }
    return res;
  }

}
