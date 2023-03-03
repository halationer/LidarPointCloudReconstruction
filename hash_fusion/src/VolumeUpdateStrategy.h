#ifndef __VOLUME_UPDATE_STRATEGY__
#define __VOLUME_UPDATE_STRATEGY__

class VolumeUpdateStrategy {

// Input :  fUpdateParam - the update param should be processed only by this class
public:

    // Output : bool - is the point a dynamic point?
    virtual bool Conflict(float& fUpdateParam) = 0;

    // Output : bool - is the point a static point?
    virtual bool Support(float& fUpdateParam) = 0;

    // Output : bool - is the point a static point?
    virtual bool IsStatic(const float& fUpdateParam) = 0;
};

//////////////////////////////////////////////////////////
// Empty strategy means do not remove any dynamic points
// 不剔除动态点
class EmptyStrategy : public VolumeUpdateStrategy {

public:

    virtual bool Conflict(float& fUpdateParam) {
        return false;
    }

    virtual bool Support(float& fUpdateParam) {
        return true;
    }

    virtual bool IsStatic(const float& fUpdateParam) {
        return true;
    }
};

//////////////////////////////////////////////////////////
// Real time strategy means what current exist is static
// 连续穿透两次则剔除
class RealTimeStrategy : public VolumeUpdateStrategy {

private:

    constexpr static int max_conflict = 1;

public:

    virtual bool Conflict(float& fUpdateParam) {
        return ++fUpdateParam > max_conflict;
    }

    virtual bool Support(float& fUpdateParam) {
        fUpdateParam = 0;
        return true;
    }

    virtual bool IsStatic(const float& fUpdateParam) {
        return fUpdateParam <= max_conflict;
    }
};

//////////////////////////////////////////////////////////
// strict static strategy means only reconstruct static points
// 感觉像一个状态机
class StrictStaticStrategy : public VolumeUpdateStrategy {

private:

    constexpr static int static_need_support = 3;
    constexpr static int max_conflict = -1;

public:

    virtual bool Conflict(float& fUpdateParam) {
        if(fUpdateParam >= static_need_support) fUpdateParam = static_need_support - 1;
        else if(fUpdateParam > 0) fUpdateParam = 0;
        else --fUpdateParam;
        return fUpdateParam < max_conflict;
    }

    virtual bool Support(float& fUpdateParam) {
        return ++fUpdateParam >= static_need_support;
    }

    virtual bool IsStatic(const float& fUpdateParam) {
        return fUpdateParam >= static_need_support;
    }
};

#endif