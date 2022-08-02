#include "Poisson.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "poisson_reconstruction");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  Poisson oPoisson(privateNode);

  ros::spin();

  return 0;
  
}

