#ifndef __HASH_POS__
#define __HASH_POS__

#include <iostream>

// 3d index of voxel(or corner)
struct HashPos {
    int x, y, z;
    HashPos():x(0),y(0),z(0){}
    HashPos(int x, int y, int z):x(x),y(y),z(z){}
};
// operator funcs
bool operator == (const HashPos & a, const HashPos & b);
std::ostream & operator << (std::ostream & out, const HashPos & pos);


// hash func of HashPos
struct HashFunc {
    size_t operator()(const HashPos& pos) const {
        return abs((pos.x * 131.1f + pos.y) * 131.2f + pos.z); // only simple func, maybe have better func
    }
};

#endif