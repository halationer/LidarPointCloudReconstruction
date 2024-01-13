#ifndef TOOL_OBJECT
#define TOOL_OBJECT

#include<Eigen/Core>

namespace tools {
    
    struct Object_ {
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        union EIGEN_ALIGN32{
            float data[12];
            struct{
                float center[4];
                float velocity[4];
                float inited;
                float radius;
                float framestamp;
                float timestamp;
            };
        };

        Object_():data({}) {}
    };

    class Object : public Object_ {
    
    public:
        inline Eigen::Map<Eigen::Vector3f> GetPosition() { return Eigen::Map<Eigen::Vector3f>(center); }
        inline Eigen::Map<Eigen::Vector3f> GetVelocity() { return Eigen::Map<Eigen::Vector3f>(velocity); }
        inline Eigen::Map<const Eigen::Vector3f> GetPosition() const { return Eigen::Map<const Eigen::Vector3f>(center); }
        inline Eigen::Map<const Eigen::Vector3f> GetVelocity() const { return Eigen::Map<const Eigen::Vector3f>(velocity); }
    };
}

#endif