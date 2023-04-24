#include "FramesFusion.h"
#include "MultiResolutionFusion.h"
#include "PreConvFusion.h"

#include <memory>

int main(int argc, char** argv){

  ros::init(argc, argv, "frames_fusion");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  int iFusionMode;
  privateNode.param("fusion_mode", iFusionMode, 0);
  std::unique_ptr<FramesFusion> pFramesFusion;
  switch(iFusionMode) {
    case 0: pFramesFusion.reset(new FramesFusion(node,privateNode));          break;
    case 1: pFramesFusion.reset(new MultiResolutionFusion(node,privateNode)); break;
    case 2: pFramesFusion.reset(new PreConvFusion(node,privateNode));         break;
  }

  pFramesFusion->LazyLoading();

  ros::spin();

  return 0;
  
}

