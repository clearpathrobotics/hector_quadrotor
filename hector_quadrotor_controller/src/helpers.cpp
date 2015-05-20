#include <hector_quadrotor_controller/helpers.h>
#include <urdf_parser/urdf_parser.h>

namespace hector_quadrotor_controller
{

  bool getMassAndInertia(const ros::NodeHandle &nh, double &mass, double inertia[3])
  {

    std::string robot_description;
    if (!nh.getParam("robot_description", robot_description))
    {
      ROS_ERROR_STREAM("getMassAndIntertia couldn't find URDF at " << nh.getNamespace() << "/robot_description");
      return false;
    }

    boost::shared_ptr<urdf::ModelInterface> model;
    try
    {
      model = urdf::parseURDF(robot_description);
    }
    catch (std::exception ex)
    {
      ROS_ERROR_STREAM(
          "getMassAndIntertia couldn't parse URDF at " << nh.getNamespace() << "/robot_description: " << ex.what());
      return false;
    }

    boost::shared_ptr<urdf::Inertial> inertial = model->getRoot()->inertial;
    if (!inertial || !inertial->mass || !inertial->ixx || !inertial->iyy || !inertial->izz)
    {
      ROS_ERROR_STREAM(
          "getMassAndIntertia requires intertial information stored on the root link " << nh.getNamespace() <<
          "/robot_description");
      return false;
    }

    mass = inertial->mass;
    inertia[0] = inertial->ixx;
    inertia[1] = inertial->iyy;
    inertia[2] = inertial->izz;
    return true;
  }

}