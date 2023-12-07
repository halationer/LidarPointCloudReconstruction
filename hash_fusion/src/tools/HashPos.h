#ifndef __HASH_POS__
#define __HASH_POS__

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Core>

// 3d index of voxel(or corner)
struct HashPos {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int x, y, z;
    HashPos():x(0),y(0),z(0){}
    HashPos(int x, int y, int z):x(x),y(y),z(z){}
};
// operator funcs
bool operator == (const HashPos & a, const HashPos & b);
HashPos operator +(const HashPos & a, const HashPos & b);
HashPos operator *(const HashPos & a, int i);
std::ostream & operator << (std::ostream & out, const HashPos & pos);


// hash func of HashPos
struct HashFunc {
    size_t operator()(const HashPos& pos) const; 
};

// align hash pos
void AlignToStepFloor(HashPos& pos, size_t step);
void AlignToStepCeil(HashPos& pos, size_t step);
bool IsAlignedToStep(const HashPos& pos, size_t step);
bool IsAlignedToLevel(const HashPos& pos, size_t level);

typedef std::unordered_set<HashPos, HashFunc> HashPosSet;
typedef std::unordered_map<HashPos, int, HashFunc> HashPosDic;

#endif