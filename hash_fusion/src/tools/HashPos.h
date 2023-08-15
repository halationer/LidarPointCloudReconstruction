#ifndef __HASH_POS__
#define __HASH_POS__

#include <iostream>
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
std::ostream & operator << (std::ostream & out, const HashPos & pos);


// hash func of HashPos
struct HashFunc {
    size_t operator()(const HashPos& pos) const; 
};

#endif