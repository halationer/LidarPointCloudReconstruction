#include "VolumeBase.h"
#include "HashBlock.h"
#include "HashVoxeler.h"


unsigned int VoxelBase::TypeByQueue() {

	if(IsUnknown()) return Unknown;
	if(__builtin_popcount(iStatusQueue) >= 4) return Free;
	return Occupied;
}

VolumeBase* VolumeBase::CreateVolume(enum VolumeType volumeType) {

    switch(volumeType) {
        case HASH_VOXELER: return new HashVoxeler;
        case HASH_BLOCK: return new HashBlock;
        default: return nullptr;
    }
}

void VolumeBase::GetCornerPoses(const HashPos & oVoxel, std::vector<HashPos> & vCornerPoses){

	vCornerPoses.clear();

	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1,	oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y + 1, 	oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x + 1,		oVoxel.y, 		oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1, 	oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x + 1,		oVoxel.y + 1,	oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y,	 	oVoxel.z + 1);
}


void VolumeBase::HashPosTo3DPos(const HashPos & oCornerPos, const Eigen::Vector3f & oVoxelLength, Eigen::Vector3f & oCorner3DPos) {
    oCorner3DPos.x() = oCornerPos.x * oVoxelLength.x();
    oCorner3DPos.y() = oCornerPos.y * oVoxelLength.y();
    oCorner3DPos.z() = oCornerPos.z * oVoxelLength.z();
}

/**
 * @brief draw debug volume
 * @param vVolume - input volume to draw
 * @return oOutputVolume - output marker for publish
*/
void VolumeBase::DrawVolume(const HashVolume & vVolume, visualization_msgs::MarkerArray & oOutputVolume) {

	constexpr int index = 1e4;

	visualization_msgs::Marker oVolumeMarker;
	oVolumeMarker.header.frame_id = "map";
	oVolumeMarker.header.stamp = ros::Time::now();
	oVolumeMarker.type = visualization_msgs::Marker::CUBE_LIST;
	oVolumeMarker.action = visualization_msgs::Marker::MODIFY;
	oVolumeMarker.id = index; 

	oVolumeMarker.scale.x = GetVoxelLength().x();
	oVolumeMarker.scale.y = GetVoxelLength().y();
	oVolumeMarker.scale.z = GetVoxelLength().z();

	oVolumeMarker.pose.position.x = 0.0;
	oVolumeMarker.pose.position.y = 0.0;
	oVolumeMarker.pose.position.z = 0.0;

	oVolumeMarker.pose.orientation.x = 0.0;
	oVolumeMarker.pose.orientation.y = 0.0;
	oVolumeMarker.pose.orientation.z = 0.0;
	oVolumeMarker.pose.orientation.w = 1.0;

	oVolumeMarker.color.a = 1.0;
	oVolumeMarker.color.r = random() / (float)RAND_MAX;
	oVolumeMarker.color.g = random() / (float)RAND_MAX;
	oVolumeMarker.color.b = random() / (float)RAND_MAX;

	for(auto && [oPos, _] : vVolume) {

		pcl::PointXYZ o3DPos = HashPosTo3DPos(oPos);

		geometry_msgs::Point point;
		point.x = o3DPos.x + GetVoxelLength().x() / 2;
		point.y = o3DPos.y + GetVoxelLength().y() / 2;
		point.z = o3DPos.z + GetVoxelLength().z() / 2;
		oVolumeMarker.points.push_back(point);
	}

	oOutputVolume.markers.push_back(oVolumeMarker);
}