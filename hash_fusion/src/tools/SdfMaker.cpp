#include "SdfMaker.h"

#include <ros/ros.h>
#include "OutputUtils.h"

SdfMaker::SdfMaker()
{
    m_pDevice = InitializeDevice(NULL);
    SetDefaultIntersectMode();
}

SdfMaker::~SdfMaker()
{
    for(auto& [_,scene] : m_vpScene) {
        if(scene != nullptr){
            rtcReleaseScene(scene);
            scene = nullptr;
        }
    }
    rtcReleaseDevice(m_pDevice);
}


// first make scene
void SdfMaker::NewScene(const pcl::PointCloud<pcl::PointXYZI> & vClouds, const std::vector<pcl::Vertices> & vMeshVertices, const int iSectorId) 
{
    // get ref of the scene id 
    RTCScene& scene = m_vpScene[iSectorId];

    // release old scene
    if(scene != nullptr) {
        rtcReleaseScene(scene);
        scene = nullptr;
    }

    // create new scene
    scene = PushSingleMeshToScene(vClouds, vMeshVertices);
}

// second query
void SdfMaker::QuerySdf(const pcl::PointXYZ & oViewPoint, pcl::PointCloud<pcl::DistanceIoVoxel> & vQueryPoints, const int iSectorId) 
{
    // get ref of the scene id 
    RTCScene& scene = m_vpScene[iSectorId];

    // check scene
    if(scene == nullptr) {
        ROS_ERROR("The Scene %d not find!", iSectorId);
        return;
    }

    // do query
    CastRay(scene, oViewPoint, vQueryPoints);
}

tools::BoundingBox SdfMaker::GetBoundingBox(const int iSectorId) {

    // get ref of the scene id 
    RTCScene& scene = m_vpScene[iSectorId];

    // check scene
    if(scene == nullptr) {
        ROS_ERROR("The Scene %d not find!", iSectorId);
        return tools::BoundingBox();
    }

    RTCBounds oBound;
    rtcGetSceneBounds(scene, &oBound);
    
    tools::BoundingBox oBoundingBox;
    oBoundingBox.GetMinBound() = Eigen::Map<Eigen::Vector3f>((float*)&oBound);
    oBoundingBox.GetMaxBound() = Eigen::Map<Eigen::Vector3f>((float*)(&oBound)+4);
    return oBoundingBox;
}

std::vector<float> SdfMaker::MakeSdf(
    const pcl::PointXYZ & oViewPoint, 
    const pcl::PointCloud<pcl::PointXYZ> & vQueryPoints, 
    const pcl::PointCloud<pcl::PointXYZI>& vClouds, 
    const std::vector<pcl::Vertices>& vMeshVertices) 
{
    m_oTimeDebugger.DebugTime("block");

    RTCScene scene = PushSingleMeshToScene(vClouds, vMeshVertices);
    m_oTimeDebugger.DebugTime("make_scene");

    // ray cast
    std::vector<float> vDis = CastRay(scene, oViewPoint, vQueryPoints);
    m_oTimeDebugger.DebugTime("cast_ray");

    rtcReleaseScene(scene);

    return vDis;
}

void SdfMaker::CoutDetailedTime() 
{
    m_oTimeDebugger.CoutCurrentLine();
}

void SdfMaker::NewTimeRecord() 
{
    m_oTimeDebugger.NewLine();
}

void SdfMaker::ErrorCallback(void* userPtr, enum RTCError error, const char* str) 
{
    ROS_ERROR("Embree Device Error %d: %s\n", error, str);
}

RTCDevice SdfMaker::InitializeDevice(const char* config) 
{
    // create device for embree
    m_pDevice = rtcNewDevice(config);

    // error process
    if (m_pDevice == nullptr)
        ROS_ERROR("error %d: cannot create device\n", rtcGetDeviceError(NULL));
    rtcSetDeviceErrorFunction(m_pDevice, SdfMaker::ErrorCallback, NULL);

    // get and print embree config infos
    ssize_t version = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_VERSION);
    ssize_t order04 = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED);
    ssize_t order08 = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED);
    ssize_t order16 = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED);

    ROS_INFO(
        "\nEmbree Version %d.\nIntersectInOrder4/8/16:%s/%s/%s.\n",
        version,
        order04?output::format_yes:output::format_no,
        order08?output::format_yes:output::format_no,
        order16?output::format_yes:output::format_no
    );

    return m_pDevice;
}

void SdfMaker::SetDefaultIntersectMode() 
{
    rtcInitIntersectArguments(&m_oIntersectArgument);

    // do not affect speed in this application
    // coherent mode for rays start from same point (the lidar center)
    m_oIntersectArgument.flags = RTC_RAY_QUERY_FLAG_COHERENT;
    // m_oIntersectArgument.flags = RTC_RAY_QUERY_FLAG_INCOHERENT; // default
}

RTCScene SdfMaker::PushSingleMeshToScene(const pcl::PointCloud<pcl::PointXYZI> & vClouds, const std::vector<pcl::Vertices> & vMeshVertices) 
{
    RTCScene scene = rtcNewScene(m_pDevice);
    
    
    //TODO： 可能的优化项-shared buffer-省去拷贝步骤
    // rtcSetSharedGeometryBuffer
    RTCGeometry geom = rtcNewGeometry(m_pDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

    // rtcSetSceneBuildQuality / rtcSetSceneFlags 选择构建用到的数据结构

    // low quality 很有效的优化，时间从20ms变为10ms左右
    rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_LOW);
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);
    rtcSetGeometryBuildQuality(geom, RTC_BUILD_QUALITY_LOW);

    // high quality 似乎也没有多高质量
    // rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_HIGH);
    // rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST);

    int iVerticesNum = vClouds.points.size();
    int iMeshNum = vMeshVertices.size();
    Vertex* vertices = (Vertex*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), iVerticesNum);
    Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), iMeshNum);

    //copy vertices value
    for (int i = 0; i < iVerticesNum; ++i) {
        vertices[i].x = vClouds.points[i].x;
        vertices[i].y = vClouds.points[i].y;
        vertices[i].z = vClouds.points[i].z;
    }

    //copy indices value
    for (int i = 0; i < iMeshNum; ++i) {
        triangles[i].v0 = vMeshVertices[i].vertices[0];
        triangles[i].v1 = vMeshVertices[i].vertices[1];
        triangles[i].v2 = vMeshVertices[i].vertices[2];
    }

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    return scene;
}

// 8ray 打包，有用但不多
#define RAY_PACKAGE_8
std::vector<float> SdfMaker::CastRay(RTCScene& scene, const pcl::PointXYZ& oViewPoint, const pcl::PointCloud<pcl::PointXYZ>& vQueryPoints) 
{
    // const params
    constexpr int bit_move = 3;
    constexpr int ray_group_size = 1 << bit_move;
    constexpr int valid_mask = -1;
    constexpr int invalid_mask = 0;
    
    // create result container
    std::vector<float> vHitDis(vQueryPoints.size());
    memset(vHitDis.data(), -1, vHitDis.size() * sizeof(float));

    #ifdef RAY_PACKAGE_8

    // make valid list, to fit the ray group size.
    int iListSize = vQueryPoints.size() >> bit_move << bit_move;
    if(iListSize < vQueryPoints.size()) iListSize += ray_group_size;
    std::vector<int> vValidVector(iListSize);
    int* vValidList = vValidVector.data();
    memset(vValidList, valid_mask, iListSize * sizeof(int));
    for(int i = vQueryPoints.size(); i < iListSize; ++i) vValidList[i] = invalid_mask;

    // make rayhit packages
    RTCRayHit8 rayhit;
    for(int i = 0; i < vQueryPoints.size(); ++i) {
        
        const int offset = i % ray_group_size;
        pcl::PointXYZ oUnitVec;

        oUnitVec.getVector3fMap() = (vQueryPoints[i].getVector3fMap() - oViewPoint.getVector3fMap()).normalized();
        rayhit.ray.org_x  [offset] = vQueryPoints[i].x;
        rayhit.ray.org_y  [offset] = vQueryPoints[i].y;
        rayhit.ray.org_z  [offset] = vQueryPoints[i].z;
        rayhit.ray.dir_x  [offset] = oUnitVec.x;
        rayhit.ray.dir_y  [offset] = oUnitVec.y;
        rayhit.ray.dir_z  [offset] = oUnitVec.z;
        rayhit.ray.tnear  [offset] = 0.1f;
        rayhit.ray.tfar   [offset] = std::numeric_limits<float>::infinity();
        rayhit.ray.mask   [offset] = -1;
        rayhit.ray.flags  [offset] = 0;
        rayhit.hit.geomID [offset] = RTC_INVALID_GEOMETRY_ID;

        // do ray intersect
        if(offset == ray_group_size - 1 || i == vQueryPoints.size() - 1) {

            rtcIntersect8(vValidList+i-offset, scene, &rayhit, &m_oIntersectArgument);

            for(int k = 0; k <= offset; ++k) {
                if(rayhit.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID)
                    vHitDis[i-offset+k] = rayhit.ray.tfar[k];
            }
        }
    }

    #endif
    
    #ifdef RAY_PACKAGE_1
    for (int i = 0; i < vQueryPoints.size(); ++i) {

        pcl::PointXYZ oUnitVec;
        oUnitVec.getVector3fMap() = (vQueryPoints[i].getVector3fMap() - oViewPoint.getVector3fMap()).normalized();

        struct RTCRayHit rayhit;
        rayhit.ray.org_x = vQueryPoints[i].x;
        rayhit.ray.org_y = vQueryPoints[i].y;
        rayhit.ray.org_z = vQueryPoints[i].z;
        rayhit.ray.dir_x = oUnitVec.x;
        rayhit.ray.dir_y = oUnitVec.y;
        rayhit.ray.dir_z = oUnitVec.z;
        rayhit.ray.tnear = 0;
        rayhit.ray.tfar = std::numeric_limits<float>::infinity();
        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        rtcIntersect1(scene, &rayhit, &m_oIntersectArgument);

        if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) vHitDis[i] = rayhit.ray.tfar;

    }//end for i
    #endif

    return vHitDis;
}

void SdfMaker::CastRay(RTCScene & scene, const pcl::PointXYZ& oViewPoint, pcl::PointCloud<pcl::DistanceIoVoxel>& vQueryPoints) 
{
    constexpr int bit_move = 3;
    constexpr int ray_group_size = 1 << bit_move;
    constexpr int valid_mask = -1;
    constexpr int invalid_mask = 0;

    // make valid list, to fit the ray group size.
    int iListSize = vQueryPoints.size() >> bit_move << bit_move;
    if(iListSize < vQueryPoints.size()) iListSize += ray_group_size;
    std::vector<int> vValidVector(iListSize);
    int* vValidList = vValidVector.data();
    memset(vValidList, valid_mask, iListSize * sizeof(int));
    for(int i = vQueryPoints.size(); i < iListSize; ++i) vValidList[i] = invalid_mask;

    // make rayhit packages
    RTCRayHit8 rayhit, downhit, uphit;
    for(int i = 0; i < vQueryPoints.size(); ++i) {
        
        const int offset = i % ray_group_size;
        vQueryPoints[i].io = -std::numeric_limits<float>().infinity();
        Eigen::Vector3f vRayVec = vQueryPoints[i].getVector3fMap() - oViewPoint.getVector3fMap();
        vQueryPoints[i].distance = vRayVec.norm();
        vRayVec /= vQueryPoints[i].distance;

        rayhit.ray.org_x  [offset] = oViewPoint.x;
        rayhit.ray.org_y  [offset] = oViewPoint.y;
        rayhit.ray.org_z  [offset] = oViewPoint.z;
        rayhit.ray.dir_x  [offset] = vRayVec.x();
        rayhit.ray.dir_y  [offset] = vRayVec.y();
        rayhit.ray.dir_z  [offset] = vRayVec.z();
        rayhit.ray.tnear  [offset] = 0.1f;
        rayhit.ray.tfar   [offset] = std::numeric_limits<float>::infinity();
        rayhit.ray.mask   [offset] = -1;
        rayhit.ray.flags  [offset] = 0;
        rayhit.hit.geomID [offset] = RTC_INVALID_GEOMETRY_ID;

        // another ray to keep the floor
        downhit.ray.org_x  [offset] = vQueryPoints[i].x;
        downhit.ray.org_y  [offset] = vQueryPoints[i].y;
        downhit.ray.org_z  [offset] = vQueryPoints[i].z;
        downhit.ray.dir_x  [offset] = 0.0f;
        downhit.ray.dir_y  [offset] = 0.0f;
        downhit.ray.dir_z  [offset] = -1.0f;
        downhit.ray.tnear  [offset] = 0.1f;
        downhit.ray.tfar   [offset] = std::numeric_limits<float>::infinity();
        downhit.ray.mask   [offset] = -1;
        downhit.ray.flags  [offset] = 0;
        downhit.hit.geomID [offset] = RTC_INVALID_GEOMETRY_ID;

        // another ray to keep the floor
        uphit.ray.org_x  [offset] = vQueryPoints[i].x;
        uphit.ray.org_y  [offset] = vQueryPoints[i].y;
        uphit.ray.org_z  [offset] = vQueryPoints[i].z;
        uphit.ray.dir_x  [offset] = 0.0f;
        uphit.ray.dir_y  [offset] = 0.0f;
        uphit.ray.dir_z  [offset] = 1.0f;
        uphit.ray.tnear  [offset] = 0.1f;
        uphit.ray.tfar   [offset] = std::numeric_limits<float>::infinity();
        uphit.ray.mask   [offset] = -1;
        uphit.ray.flags  [offset] = 0;
        uphit.hit.geomID [offset] = RTC_INVALID_GEOMETRY_ID;

        // do ray intersect
        if(offset == ray_group_size - 1 || i == vQueryPoints.size() - 1) {

            rtcIntersect8(vValidList+i-offset, scene, &rayhit, &m_oIntersectArgument);
            rtcIntersect8(vValidList+i-offset, scene, &downhit, &m_oIntersectArgument);
            rtcIntersect8(vValidList+i-offset, scene, &uphit, &m_oIntersectArgument);

            for(int k = 0; k <= offset; ++k) {

                pcl::DistanceIoVoxel& oQueryPoint = vQueryPoints[i-offset+k];

                if(rayhit.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID) {
                    float sdf = rayhit.ray.tfar[k] - oQueryPoint.distance;
                    oQueryPoint.io = sdf < 0 ? 0.0f : 1.0f;
                    oQueryPoint.distance = abs(sdf);
                    if(downhit.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID) {
                        float sdf = downhit.ray.tfar[k];
                        oQueryPoint.distance = std::min(oQueryPoint.distance, abs(sdf));
                    }
                    if(uphit.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID) {
                        float sdf = uphit.ray.tfar[k];
                        oQueryPoint.distance = std::min(oQueryPoint.distance, abs(sdf));
                    }
                }
                else {
                    oQueryPoint.io = 0.0f;
                    oQueryPoint.distance = std::numeric_limits<float>::infinity();
                }
            }
        }
    }
}