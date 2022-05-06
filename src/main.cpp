#include "FrameRecon.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "frame_reconstruction");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  FrameRecon FrameReBuilder(node,privateNode);

  ros::spin();

  return 0;
}

