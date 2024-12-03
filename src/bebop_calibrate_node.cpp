

#include <algorithm>
#include <string>
#include <vector>

#include <nodelet/loader.h>
// #include <rclcpp/rclcpp.hpp>
#include <ros/ros.h>


int
main( int argc, char *argv[] )
{
  ros::init( argc, argv, "bebop_calibrate", ros::init_options::NoSigintHandler );
  nodelet::Loader nll;

  nodelet::M_string remap( ros::names::getRemappings() );
  nodelet::V_string nargv;
  const std::string nl_name = ros::this_node::getName();
  nll.load( nl_name, "bebop_calibrate_nodelet", remap, nargv );

  ROS_INFO("[DEMO BEBOP 2] INIT");
  const std::vector< std::string > &loaded_nodelets = nll.listLoadedNodelets();
  if ( std::find( loaded_nodelets.begin(),
                  loaded_nodelets.end(),
                  nl_name ) == loaded_nodelets.end() )
  {
    // Nodelet OnInit() failed
    ROS_FATAL( "bebop_calibrate nodelet failed to load." );
    return 1;
  }

  // It reaches here when OnInit() succeeds
  ROS_INFO( "bebop_calibrate nodelet loaded." );
  ros::spin();
  ROS_INFO("[DEMO BEBOP 2] AFTER");
  return 0;
}
