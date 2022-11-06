#include "Confidence.h"


inline void Confidence::ComputeConfidence(const pcl::PointXYZI& view_point, const pcl::PointNormal& current_point) {

    Eigen::Vector3f view_pos(view_point.x, view_point.y, view_point.z);
    Eigen::Vector3f current_pos(current_point.x, current_point.y, current_point.z);
    Eigen::Vector3f current_normal(current_point.normal_x, current_point.normal_y, current_point.normal_z);
    ComputeConfidence(view_pos, current_pos, current_normal);
}

//深度置信度计算方法
float Confidence::GetDepthConfidence_Inverse(float depth) {

    const float depth_confidence = 1.0 / (depth + 1);
    min_depth_confidence = min_depth_confidence > depth_confidence || max_depth_confidence == 0 ? depth_confidence : min_depth_confidence;
    max_depth_confidence = max_depth_confidence < depth_confidence || max_depth_confidence == 0 ? depth_confidence : max_depth_confidence;
    return depth_confidence;
}
float Confidence::GetDepthConfidence_Inverse(const Eigen::Vector3f& view_pos, const Eigen::Vector3f& current_pos) {

    return GetDepthConfidence_Inverse((current_pos - view_pos).norm());
}


// counter
float Confidence::min_depth_confidence = 0;
float Confidence::max_depth_confidence = 0;
void Confidence::StartCounter() {min_depth_confidence = 0; max_depth_confidence = 0;}