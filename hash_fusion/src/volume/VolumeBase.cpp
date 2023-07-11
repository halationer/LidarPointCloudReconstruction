#include "VolumeBase.h"
#include "HashBlock.h"
#include "HashVoxeler.h"

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