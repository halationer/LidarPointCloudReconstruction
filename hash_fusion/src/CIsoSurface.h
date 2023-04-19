#ifndef CISOSURFACE_H
#define CISOSURFACE_H
// File Name: CIsoSurface.h
// Last Modified: 5/8/2000
// Author: Raghavendra Chandrashekara (basesd on source code
// provided by Paul Bourke and Cory Gene Bloyd)
// Email: rc99@doc.ic.ac.uk, rchandrashekara@hotmail.com
//
// Description: This is the interface file for the CIsoSurface class.
// CIsoSurface can be used to construct an isosurface from a scalar
// field.

#include <map>
#include <vector>
#include <functional>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/Vertices.h>
#include "Vectors.h"
#include "HashVoxeler.h"

struct POINT3DID {
	unsigned int newID;
	float x, y, z;
};

struct EDGEID {
	int x, y, z, w;
	EDGEID(){}
	EDGEID(int x, int y, int z, int w):x(x),y(y),z(z),w(w){}
	EDGEID(const EDGEID& id):x(id.x),y(id.y),z(id.z),w(id.w){}
};
bool operator == (const EDGEID & a, const EDGEID & b);

struct HashEdgeFunc {
    size_t operator()(const EDGEID& id) const {
        return abs(((id.x * 131.1f + id.y) * 131.2f + id.z) * 131.2f + id.w);
    }
};

typedef std::unordered_map<EDGEID, POINT3DID, HashEdgeFunc> ID2POINT3DID;

struct TRIANGLE {
	unsigned int pointID[3];
	unsigned int pointAttr;
	EDGEID edgeID[3];
};

typedef std::vector<TRIANGLE> TRIANGLEVECTOR;

template <class T> class CIsoSurface {
public:
	// Constructor and destructor.
	CIsoSurface();
	~CIsoSurface();
	
	// Generates the isosurface from the scalar field contained in the
	// buffer ptScalarField[].
	void GenerateSurface(const std::unordered_map<HashPos, T, HashFunc> & ptScalarField, const HashVoxeler::HashVolume & vVolume, T tIsoLevel, float fCellLengthX, float fCellLengthY, float fCellLengthZ);


	// Returns true if a valid surface has been generated.
	bool IsSurfaceValid();

	// Deletes the isosurface.
	void DeleteSurface();

	// Returns the length, width, and height of the volume in which the
	// isosurface in enclosed in.  Returns -1 if the surface is not
	// valid.
	int GetVolumeLengths(float& fVolLengthX, float& fVolLengthY, float& fVolLengthZ);

	// output reconstruction result
	void OutputMesh(const pcl::PointXYZ & oOffset, pcl::PolygonMesh & oCBModel);

	void OutputMesh(const pcl::PointXYZ & oOffset, pcl::PolygonMesh & oCBModel, pcl::PointCloud<pcl::PointXYZ>::Ptr & pMarchCuberCloud);

	// The number of vertices which make up the isosurface.
	unsigned int m_nVertices;

	// The vertices which make up the isosurface.
	POINT3D* m_ppt3dVertices;

	// The number of triangles which make up the isosurface.
	unsigned int m_nTriangles;

	// The indices of the vertices which make up the triangles.
	unsigned int* m_piTriangleIndices;

	unsigned int* m_piTriangleAttributes;

	// The number of normals.
	unsigned int m_nNormals;

	// The normals.
	VECTOR3D* m_pvec3dNormals;

	// List of POINT3Ds which form the isosurface.
	ID2POINT3DID m_i2pt3idVertices;

	// List of TRIANGLES which form the triangulation of the isosurface.
	TRIANGLEVECTOR m_trivecTriangles;

	// Lookup tables used in the construction of the isosurface.
	static const int m_edgeTable[256];
	static const int m_triTable[256][16];

protected:

	// Returns the edge ID.
	EDGEID GetEdgeID(int nX, int nY, int nZ, unsigned int nEdgeNo);

	// Calculates the intersection point of the isosurface with an
	// edge.
	POINT3DID CalculateIntersection(int nX, int nY, int nZ, unsigned int nEdgeNo);

	// Interpolates between two grid points to produce the point at which
	// the isosurface intersects an edge.
	POINT3DID Interpolate(float fX1, float fY1, float fZ1, float fX2, float fY2, float fZ2, T tVal1, T tVal2);
 
	// Renames vertices and triangles so that they can be accessed more
	// efficiently.
	void RenameVerticesAndTriangles();

	// Calculates the normals.
	void CalculateNormals();

	// Cell length in x, y, and z directions.
	float m_fCellLengthX, m_fCellLengthY, m_fCellLengthZ;

	// The buffer holding the scalar field.
	std::unordered_map<HashPos, T, HashFunc> m_ptScalarField;

	// The isosurface value.
	T m_tIsoLevel;

	// Indicates whether a valid surface is present.
	bool m_bValidSurface;

public:
	std::function<uint32_t(const pcl::PointNormal&)> m_pGetAttrFunc;
};
#endif // CISOSURFACE_H
