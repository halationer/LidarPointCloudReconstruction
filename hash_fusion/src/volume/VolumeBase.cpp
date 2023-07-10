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