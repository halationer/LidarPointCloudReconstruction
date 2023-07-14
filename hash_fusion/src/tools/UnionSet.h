#ifndef __UNION_SET__
#define __UNION_SET__

#include <unordered_map>
#include <vector>

#include "tools/HashPos.h"

// Union-Find Set
class UnionSet {
    
private:
    // container
	std::unordered_map<HashPos, std::pair<HashPos,int>, HashFunc> m_vUnionSet;

public:
    // the size of the largest set
	int m_iMaxSetSize = 0;

public:
	// clear the set and ready to rebuild it
	void Clear() {
		std::unordered_map<HashPos, std::pair<HashPos,int>, HashFunc> temp;
		m_vUnionSet.swap(temp);
		m_iMaxSetSize = 0;
	}

    // find the set which contains the voxel
	HashPos Find(const HashPos& oPos, bool bNewNode = true) {
		if(m_vUnionSet.count(oPos)) {
			if(oPos == m_vUnionSet[oPos].first) return oPos;
			else return Find(m_vUnionSet[oPos].first);
		}
		if(bNewNode) m_vUnionSet[oPos] = {oPos, 1};
		return oPos;
	}

    // put two voxel in the same set
	void Union(const HashPos& oPosA, const HashPos& oPosB) {
		HashPos oRootAPos = Find(oPosA), oRootBPos = Find(oPosB);
		if(oRootAPos == oRootBPos) return;
		auto &oRootA = m_vUnionSet[oRootAPos], &oRootB = m_vUnionSet[oRootBPos];
		if(oRootA.second > oRootB.second) {
			oRootB.first = oRootA.first;
			oRootA.second += oRootB.second;
			m_iMaxSetSize = std::max(m_iMaxSetSize, oRootA.second);
		}
		else {
			oRootA.first = oRootB.first;
			oRootB.second += oRootA.second;
			m_iMaxSetSize = std::max(m_iMaxSetSize, oRootB.second);
		}
	}

    // get the size of the set that contains the voxel
	int GetSetSize(const HashPos& oPos) {
		HashPos oRootPos = Find(oPos, false);
		if(m_vUnionSet.count(oPos)) return m_vUnionSet[oRootPos].second;
		else return 1;
	}

    // if a voxel is in the largest set
	bool InMaxSet(const HashPos& oPos) {
		return GetSetSize(oPos) == m_iMaxSetSize;
	}

    // save different set voxel in vectors
	std::unordered_map<HashPos, std::vector<HashPos>, HashFunc> GetSets() {
		std::unordered_map<HashPos, std::vector<HashPos>, HashFunc> vSetBuffer;
		for(auto && [oPos, oPair] : m_vUnionSet) {
			vSetBuffer[Find(oPos, false)].push_back(oPos); 
		}
		return vSetBuffer;
	}
};

#endif