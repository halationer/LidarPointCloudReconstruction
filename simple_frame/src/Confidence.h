
#include<iostream>
#include<Eigen/Dense>
#include<pcl/point_types.h>

#ifndef __CONFIDENCE__
#define __CONFIDENCE__


class Confidence {

public:
    float depth_confidence;
    float normal_confidence;

    Confidence() {}

    template<typename T>
    void ComputeConfidence(const Eigen::Vector3f& view_pos, const Eigen::Vector3f& current_pos, const T& current_normal);

    template<typename T>
    void ComputeConfidence(const pcl::PointXYZI& view_point, const pcl::PointXYZI& current_point, const T& current_normal);

    void ComputeConfidence(const pcl::PointXYZI& view_point, const pcl::PointNormal& current_point);

    static float GetDepthConfidence_Inverse(float depth);
    static float GetDepthConfidence_Inverse(const Eigen::Vector3f& view_pos, const Eigen::Vector3f& current_pos);
    
    template<typename T>
    static float GetNormalConfidence_Consine(const Eigen::Vector3f& standard_normal, const T& current_normal);

//counters
public:
    static float min_depth_confidence;
    static float max_depth_confidence;
    static void StartCounter();
};


// #### templates ####

//根据视点位置以及观测点属性计算置信度
template<typename T>
void Confidence::ComputeConfidence(const Eigen::Vector3f& view_pos, const Eigen::Vector3f& current_pos, const T& current_normal) {

    Eigen::Vector3f current_to_view = view_pos - current_pos;
    depth_confidence = GetDepthConfidence_Inverse(current_to_view.norm());

    current_to_view.normalize();
    normal_confidence = GetNormalConfidence_Consine(current_to_view, current_normal);
}

template<typename T>
void Confidence::ComputeConfidence(const pcl::PointXYZI& view_point, const pcl::PointXYZI& current_point, const T& current_normal) {

    //TODO: Maybe can speed up.
    Eigen::Vector3f view_pos(view_point.x, view_point.y, view_point.z);
    Eigen::Vector3f current_pos(current_point.x, current_point.y, current_point.z);
    ComputeConfidence(view_pos, current_pos, current_normal);
}

//法向置信度计算方法
template<typename T>
float Confidence::GetNormalConfidence_Consine(const Eigen::Vector3f& standard_normal, const T& current_normal) {

    return standard_normal.dot(current_normal);
}

#endif