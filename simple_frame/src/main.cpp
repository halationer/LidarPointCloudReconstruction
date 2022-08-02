#include "SimpleRecon.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "simple_frame");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  bool UseGHPR;
  privateNode.param("use_ghpr", UseGHPR, false);

  boost::unique_ptr<FrameRecon> FrameReBuilder(UseGHPR ? new FrameRecon(node, privateNode) : new SimpleRecon(node,privateNode));
 
  FrameReBuilder->LazyLoading();

  ros::spin();

  return 0;
}

