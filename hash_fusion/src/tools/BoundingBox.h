#ifndef TOOL_BOUNDING_BOX
#define TOOL_BOUNDING_BOX

#include<Eigen/Core>

namespace tools {
    
    struct BoundingBox {
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        union EIGEN_ALIGN32{
            float data[12];
            struct{
                float data_min[4];
                float data_max[4];
                float inited;
                float point_count;
            };
        };

        BoundingBox():data({}) {}

        inline Eigen::Map<Eigen::Vector3f> GetMinBound() { return Eigen::Map<Eigen::Vector3f>(data_min); }
        inline Eigen::Map<Eigen::Vector3f> GetMaxBound() { return Eigen::Map<Eigen::Vector3f>(data_max); }
        inline Eigen::Map<const Eigen::Vector3f> GetMinBound() const { return Eigen::Map<const Eigen::Vector3f>(data_min); }
        inline Eigen::Map<const Eigen::Vector3f> GetMaxBound() const { return Eigen::Map<const Eigen::Vector3f>(data_max); }
    };
}

#endif