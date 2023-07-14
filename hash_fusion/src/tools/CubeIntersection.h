#ifndef CUBE_INTERSECTION
#define CUBE_INTERSECTION

#include <pcl/point_types.h>
#include <Eigen/Core>


/**
 *   /|
 *  y1|
 * /__|____   
 * |  |_x1_|___
 * z0 /    |  /
 * | y0    z1/
 * |/__x0__|/
 * 
 *    ___x2___ 
 *   /|      /|
 *  / z3   y2 |
 * /__|_x3_/  z2
 *    |____|__|
 *         |  /
 *         | y3
 *         |/
*/

// surfel in cube intersect which face of the cube
class CubeIntersection {

private:
    // Ax + By + Cz = neg_D - plane expression
    float A, B, C, neg_D;
    float AL, BL, CL, AH, BH, CH;
    Eigen::Vector3f vNormal, vCubeSize, vCubeBaseLow, vCubeBaseHigh;

    float cross_x0, cross_x1, cross_x2, cross_x3;
    float cross_y0, cross_y1, cross_y2, cross_y3;
    float cross_z0, cross_z1, cross_z2, cross_z3;

    bool neg_init, pos_init;

    bool CrossValid(float fCrossValue) { return fCrossValue > 0.0f && fCrossValue <= 1.0f; }
    // negtive
    float CrossX00() { return (A ? (neg_D       - BL  - CL) / A - vCubeBaseLow.x() : -1.0f) / vCubeSize.x(); }
    float Cross0Y0() { return (B ? (neg_D - AL        - CL) / B - vCubeBaseLow.y() : -1.0f) / vCubeSize.y(); }
    float Cross00Z() { return (C ? (neg_D - AL  - BL      ) / C - vCubeBaseLow.z() : -1.0f) / vCubeSize.z(); }
    float CrossX10() { return (A ? (neg_D       - BH  - CL) / A - vCubeBaseLow.x() : -1.0f) / vCubeSize.x(); }
    float Cross0Y1() { return (B ? (neg_D - AL        - CH) / B - vCubeBaseLow.y() : -1.0f) / vCubeSize.y(); }
    float Cross10Z() { return (C ? (neg_D - AH  - BL      ) / C - vCubeBaseLow.z() : -1.0f) / vCubeSize.z(); }
    // positive
    float CrossX11() { return (A ? (neg_D       - BH  - CH) / A - vCubeBaseLow.x() : -1.0f) / vCubeSize.x(); }
    float Cross1Y1() { return (B ? (neg_D - AH        - CH) / B - vCubeBaseLow.y() : -1.0f) / vCubeSize.y(); }
    float Cross11Z() { return (C ? (neg_D - AH  - BH      ) / C - vCubeBaseLow.z() : -1.0f) / vCubeSize.z(); }
    float CrossX01() { return (A ? (neg_D       - BL  - CH) / A - vCubeBaseLow.x() : -1.0f) / vCubeSize.x(); }
    float Cross1Y0() { return (B ? (neg_D - AH        - CL) / B - vCubeBaseLow.y() : -1.0f) / vCubeSize.y(); }
    float Cross01Z() { return (C ? (neg_D - AL  - BH      ) / C - vCubeBaseLow.z() : -1.0f) / vCubeSize.z(); }

    void InitNegParams() {
        if(!neg_init) {
            cross_x0 = CrossX00();
            cross_y0 = Cross0Y0();
            cross_z0 = Cross00Z();
            cross_x1 = CrossX10();
            cross_y1 = Cross0Y1();
            cross_z1 = Cross10Z();
            neg_init = true;
        }
    }
    void InitPosParams() {
        if(!pos_init) {
            cross_x2 = CrossX11();
            cross_y2 = Cross1Y1();
            cross_z2 = Cross11Z();
            cross_x3 = CrossX01();
            cross_y3 = Cross1Y0();
            cross_z3 = Cross01Z();
            pos_init = true;
        }
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // left face
    bool CrossNegX() { InitNegParams(); return CrossValid(cross_y0) || CrossValid(cross_z0) || CrossValid(cross_y1); }
    // back face
    bool CrossNegY() { InitNegParams(); return CrossValid(cross_x0) || CrossValid(cross_z0) || CrossValid(cross_z1); }
    // bottom face
    bool CrossNegZ() { InitNegParams(); return CrossValid(cross_x0) || CrossValid(cross_y0) || CrossValid(cross_x1); }
    
    // right face
    bool CrossPosX() { InitPosParams(); return CrossValid(cross_y2) || CrossValid(cross_z2) || CrossValid(cross_y3); }
    // front face
    bool CrossPosY() { InitPosParams(); return CrossValid(cross_z2) || CrossValid(cross_z3) || CrossValid(cross_x2); }
    // top face
    bool CrossPosZ() { InitPosParams(); return CrossValid(cross_x2) || CrossValid(cross_y2) || CrossValid(cross_x3); }


    bool CrossNeg(int iAxisId) { 
        switch(iAxisId) {
            case 0: return CrossNegX();
            case 1: return CrossNegY();
            case 2: return CrossNegZ();
        }
        return false;
    }

    bool CrossPos(int iAxisId) { 
        switch(iAxisId) {
            case 0: return CrossPosX();
            case 1: return CrossPosY();
            case 2: return CrossPosZ();
        }
        return false;
    }

    void Reset(const pcl::PointNormal& oPoint, const Eigen::Vector3f& vCubeSize, const Eigen::Vector3f& vCubeBase) {
        
        vNormal = oPoint.getNormalVector3fMap();
        this->vCubeSize = vCubeSize;
        this->vCubeBaseLow = vCubeBase;
        vCubeBaseHigh = vCubeBaseLow + vCubeSize;

        neg_init = false;
        pos_init = false;
        		
        A = oPoint.normal_x;
        B = oPoint.normal_y;
        C = oPoint.normal_z;
		neg_D = A * oPoint.x + B * oPoint.y + C * oPoint.z;


        AL = A * vCubeBaseLow.x();
        BL = B * vCubeBaseLow.y();
        CL = C * vCubeBaseLow.z();
        AH = A * vCubeBaseHigh.x();
        BH = B * vCubeBaseHigh.y();
        CH = C * vCubeBaseHigh.z();
    }

    CubeIntersection() { }
    CubeIntersection(const pcl::PointNormal& oPoint, const Eigen::Vector3f& vCubeSize, const Eigen::Vector3f& vCubeBase){
        Reset(oPoint, vCubeSize, vCubeBase);
    }
};

#endif