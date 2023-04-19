#include "FramesFusion.h"
#include "MultiResolutionFusion.h"

#include <memory>

int main(int argc, char** argv){

  ros::init(argc, argv, "frames_fusion");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  bool bUseMultiResolution;
  privateNode.param("use_multi_resolution", bUseMultiResolution, false);
  std::unique_ptr<FramesFusion> pFramesFusion(bUseMultiResolution ? new MultiResolutionFusion(node,privateNode) : new FramesFusion(node,privateNode));

  pFramesFusion->LazyLoading();

  ros::spin();

  return 0;
  
}

