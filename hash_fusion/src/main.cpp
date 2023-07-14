#include "FramesFusion.h"

#include <memory>

int main(int argc, char** argv){

  ros::init(argc, argv, "frames_fusion");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  int iFusionMode = 0;
  // privateNode.param("fusion_mode", iFusionMode, 0);
  std::unique_ptr<FramesFusion> pFramesFusion;
  switch(iFusionMode) {
    // case 1: pFramesFusion.reset(new MultiResolutionFusion(node,privateNode)); break;
    // case 2: pFramesFusion.reset(new PreConvFusion(node,privateNode));         break;
    default: pFramesFusion.reset(new FramesFusion(node,privateNode));
  }

  pFramesFusion->LazyLoading();

  ros::spin();

  return 0;
  
}

