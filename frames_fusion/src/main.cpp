#include "FramesFusion.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "frames_fusion");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  FramesFusion oFramesFusion(node,privateNode);

  ros::spin();

  return 0;
  
}

