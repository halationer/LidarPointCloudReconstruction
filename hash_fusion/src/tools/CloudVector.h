#ifndef __CLOUD_VECTOR__
#define __CLOUD_VECTOR__

#include <vector>
#include <map>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

// TODO: New cloud data structure
// 接下来要添加的功能 
//     1.点云帧数控制，超出的帧将被剔除，并且输出到结果点云中去
//     2.不同帧的点使用不同颜色显示
//     3.如何解决回环问题？ 将删除的帧绑定一个Box，每次检测与Box的碰撞，如果有重叠区域，则将此帧重新加载（进阶选做）- 如果实现重加载则需要修改key的逻辑，不能为seq

class CloudVector {

public:

    CloudVector():dirty_flag(true),max_window_size(100),auto_release_frames(true),auto_save_frames(true) {}
    CloudVector(const CloudVector &vCloud) = delete;
    CloudVector& operator=(const CloudVector &vCloud) = delete;

    void SetMaxWindow(int window_size) { max_window_size = window_size; }
    void SetAutoRelease(bool auto_release) { auto_release_frames = auto_release; }
    void SetAutoSave(bool auto_save) { auto_save_frames = auto_save; }

    unsigned int size() {
        
        ComputeSize();

        return size_pre_sum.size() ? size_pre_sum.back() : 0;
    }
  
    friend pcl::PointCloud<pcl::PointNormal>& operator+=(pcl::PointCloud<pcl::PointNormal>& vCloudA, const CloudVector& vCloudB);    
    friend CloudVector& operator+=(CloudVector &vCloudA, const CloudVector &vCloudB);
    friend CloudVector& operator+=(CloudVector &vCloudA, const pcl::PointCloud<pcl::PointNormal> vCloudB);
    friend pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(CloudVector& cloud_vector);
    
    pcl::PointNormal& at(const int index);
    void erase(int index);

    pcl::PointNormal& operator[](const int index) {
        
        return this->at(index);
    }

    void push_back(const pcl::PointNormal& point, int seq = 0) {
        
        dirty_flag = true;
        data[seq]->push_back(point);
    }

    void ReleaseFrames();
    void SaveFrame(int seq);

private:

    void ComputeSize();

    bool dirty_flag;
    
    bool auto_release_frames;
    bool auto_save_frames;
    unsigned int max_window_size;

    std::vector<unsigned int> size_pre_sum;
    std::vector<int> seq_record;

    // key := pointcloud.header.seq | value := &pointcloud
    std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> data;
};

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(CloudVector& cloud_vector);

#endif