#include "fsa_process.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  FSA_process fsa_process;
  fsa_process.setUp();

  fsa_process.start();

  ros::Rate loop_rate(5);
  while(ros::ok())
  {
    ros::spinOnce();
	fsa_process.run();
    loop_rate.sleep();
  }

  return 0;
}
