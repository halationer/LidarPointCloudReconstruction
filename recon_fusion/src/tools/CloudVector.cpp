#include "CloudVector.h"

#include <iostream>
#include <thread>
#include <algorithm>

#include "OutputUtils.h"

pcl::PointCloud<pcl::PointNormal>& operator+=(pcl::PointCloud<pcl::PointNormal>& vCloudA, const CloudVector& vCloudB) {

    
    for(auto [_, pc] : vCloudB.data) {

        vCloudA += *pc;
    }

    return vCloudA;
}

CloudVector& operator+=(CloudVector &vCloudA, const CloudVector &vCloudB) {

	vCloudA.dirty_flag = true;

	for(auto [seq, pc] : vCloudB.data) {

		if(vCloudA.data.count(seq)) {
		
			*vCloudA.data[seq] += *pc;
		}
		else {

			vCloudA.data[seq] = pc;
		}
	}

	// release frames to keep storage
	if(vCloudA.auto_release_frames) {

		vCloudA.ReleaseFrames();
	}

	return vCloudA;
}

CloudVector& operator+=(CloudVector &vCloudA, const pcl::PointCloud<pcl::PointNormal> vCloudB) {

	// std::cout << "Add frame: " << vCloudB.header.seq << ";\t"
	// 			<< "Past size: " << (vCloudA.data.count(vCloudB.header.seq) ? vCloudA.data[vCloudB.header.seq]->size() : 0) << ";\t"
	// 			<< "After size: " << vCloudB.size() + (vCloudA.data.count(vCloudB.header.seq) ? vCloudA.data[vCloudB.header.seq]->size() : 0) << std::endl;

	vCloudA.dirty_flag = true;

	if(vCloudA.data.count(vCloudB.header.seq)) {
	
		*vCloudA.data[vCloudB.header.seq] += vCloudB;
	}
	else {

		vCloudA.data[vCloudB.header.seq] = vCloudB.makeShared();
	}

	// release frames to keep storage
	if(vCloudA.auto_release_frames) {

		vCloudA.ReleaseFrames();
	}

	// std::cout << "Add seq " << vCloudB.header.seq << " Finish, final size: " << vCloudA.size() << std::endl;

	return vCloudA;
}  

void CloudVector::ComputeSize() {

	if(dirty_flag) {

		dirty_flag = false;
		size_pre_sum.clear();
		seq_record.clear();
		size_pre_sum.reserve(data.size());
		seq_record.reserve(data.size()); 

		for(auto [_,pc] : data) {
			
			unsigned int last_size = size_pre_sum.empty() ? 0 : size_pre_sum.back();
			size_pre_sum.push_back(pc->size() + last_size);
			seq_record.push_back(pc->header.seq);
		}
	}
}

pcl::PointNormal& CloudVector::at(const int index) {

	ComputeSize();
	
	auto iter = upper_bound(size_pre_sum.begin(), size_pre_sum.end(), index);

	// std::cout << "iter: " << (iter != size_pre_sum.end() ? data[seq_record[iter - size_pre_sum.begin()]]->size() - *iter + index : -1) << std::endl;
	
	if(iter == size_pre_sum.end()) {
		
		std::cout << std::format_red << "CloudVector out of range!!!" << std::format_white << std::endl;
		return data[seq_record.back()]->back();
	}
	else return data[seq_record[iter - size_pre_sum.begin()]]->at(data[seq_record[iter - size_pre_sum.begin()]]->size() - *iter + index);
}

void CloudVector::erase(int index) {

	ComputeSize();
	
	dirty_flag = true;
	auto iter = upper_bound(size_pre_sum.begin(), size_pre_sum.end(), index);
	
	if(iter == size_pre_sum.end()) {
		
		std::cout << std::format_red << "CloudVector out of range!!!" << std::format_white << std::endl;
		return;
	}
	std::swap(
		data[seq_record[iter - size_pre_sum.begin()]]->at(data[seq_record[iter - size_pre_sum.begin()]]->size() - *iter + index), 
		data[seq_record[iter - size_pre_sum.begin()]]->back()
	);
	data[seq_record[iter - size_pre_sum.begin()]]->points.pop_back();
	--data[seq_record[iter - size_pre_sum.begin()]]->width;
}

void CloudVector::ReleaseFrames() {

	int release_num = data.size() - max_window_size;
	std::vector<int> release_seq;

	dirty_flag = true;

	for(auto [seq,pc] : data) {

		if(release_num-- <= 0) break;

		release_seq.push_back(seq);
	}

	for(auto seq : release_seq) {
		
		if(auto_save_frames) {

			SaveFrame(seq);
		}
		data.erase(seq);
	}
}

void CloudVector::SaveFrame(int seq) {

	// std::cout << "release seq: " << seq << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr save_cloud = data[seq];
	std::thread save_thread([save_cloud, seq]() {
		
		system("mkdir -p /tmp/lidar_recon_temp");
		std::stringstream save_path;
		save_path << "/tmp/lidar_recon_temp/frame" << std::setw(6) << std::setfill('0') << seq << ".ply";
		pcl::io::savePLYFileASCII(save_path.str(), *save_cloud);
	});
	save_thread.detach();
}

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(CloudVector& cloud_vector) {

    pcl::PointCloud<pcl::PointNormal>::Ptr vOutputCloud(new pcl::PointCloud<pcl::PointNormal>());

    for(auto [_,pc] : cloud_vector.data) {

        *vOutputCloud += *pc;
    }

    return vOutputCloud;
}