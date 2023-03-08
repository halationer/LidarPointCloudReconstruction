#include "VolumeUpdateStrategy.h"

VolumeUpdateStrategy* CreateStrategy(enum vus eStrategy) {
    switch(eStrategy) {
        case eEmptyStrategy: return new EmptyStrategy;
        case eRealTimeStrategy: return new RealTimeStrategy;
        case eStrictStaticStrategy: return new StrictStaticStrategy;
        default: return nullptr;
    }
}