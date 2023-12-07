#include "HashPos.h"

bool operator == (const HashPos & a, const HashPos & b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

HashPos operator + (const HashPos & a, const HashPos & b) {
	return HashPos(a.x + b.x, a.y + b.y, a.z + b.z);
}

HashPos operator * (const HashPos & a, int i) {
	return HashPos(a.x * i, a.y * i, a.z * i);
}

std::ostream & operator << (std::ostream & out, const HashPos & pos) {
	out << "(" << pos.x << "," << pos.y << "," << pos.z << ")";
	return out;
}

void AlignToStepFloor(HashPos& pos, size_t step) {

    pos.x = floor(pos.x / (float)step) * step;
    pos.y = floor(pos.y / (float)step) * step;
    pos.z = floor(pos.z / (float)step) * step;
}

void AlignToStepCeil(HashPos& pos, size_t step) {

    pos.x = ceil(pos.x / (float)step) * step;
    pos.y = ceil(pos.y / (float)step) * step;
    pos.z = ceil(pos.z / (float)step) * step;
}

bool IsAlignedToStep(const HashPos& pos, size_t step) {

	return pos.x % step == 0 && pos.y % step == 0 && pos.z % step == 0;
}

bool IsAlignedToLevel(const HashPos& pos, size_t level) {

	return IsAlignedToStep(pos, 1u << level);
}

size_t HashFunc::operator()(const HashPos& pos) const {
	// different func only influence the speed
	return abs((pos.x * 131.1f + pos.y) * 131.2f + pos.z); 					// baseline but the best - only simple func, maybe have better func
	// return static_cast<size_t>((pos.z * 131.1f + pos.y) * 131.2f + pos.x); 	// just ok
	// return static_cast<size_t>((pos.x * 131.1f + pos.z) * 131.2f + pos.y); 	// just ok
	// return static_cast<size_t>(pos.x << 7 ^ pos.y << 8 ^ pos.z); 			// worse
	// return std::hash<int>()(pos.x); 											// much worse
}