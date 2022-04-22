//Edited by Yu. YongTao and added by Huang Pengdi
//对齐方式需要调节，因为pcd和las文件的读取方式不一样

//防重复定义头文件
#ifndef LASOPERATOR_H 
#define LASOPERATOR_H 

#include <fstream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#pragma pack(1)
//1的整数倍为不对齐

/*===================================================================
Structure for PUBLIC HEADER BLOCK
The fields included in the public header block are as follows:
1.     File Signature ("LASF")
2.     File Source ID
3.     Reserved (Global Encoding for v1.2 or above)
4-7.   Project ID - GUID Data 1-4
8-9.   Version Major and Minor (Major.Minor)
10.    System Identifier
11.    Generating Software
12-13. File Creation Day and Year
14.    Header Size
15.    Offset to Point Data
16.    Number of Variable Length Records
17.    Point Data Format ID (0-99 for spec)
18.    Point Data Record Length
19.    Number of Point Records
20.    Number of Points by Return
21.    X Scale Factor
22.    Y Scale Factor
23.    Z Scale Factor
24.    X Offset
25.    Y Offset
26.    Z Offset
27.    Max X
28.    Min X
29.    Max Y
30.    Min Y
31.    Max Z
32.    Min Z
===================================================================*/

struct PublicHeaderBlock{

	char fileSign[4];
	unsigned short fileSourceID;
	unsigned short reserved;    // Global Encoding for v1.2 or above
	unsigned long GUID1;
	unsigned short GUID2;
	unsigned short GUID3;
	unsigned char GUID4[8];

	unsigned char versionMajor;
	unsigned char versionMinor;
	char systemID[32];
	char GenSoft[32];
	unsigned short creationDay;
	unsigned short creationYear;

	unsigned short headerSize;
	unsigned long offsetToData;
	unsigned long varRecordNum;
	unsigned char dataFormat;
	unsigned short pointRecordLen;
	unsigned long pointRecordNum;
	unsigned long returnPointNum[5];

	double xScale;
	double yScale;
	double zScale;
	double xOffset;
	double yOffset;
	double zOffset;
	double maxX;
	double minX;
	double maxY;
	double minY;
	double maxZ;
	double minZ;
};

/*===================================================================
Structure for VARIABLE LENGTH RECORD HEADER
The fields included in the variable length record header are as follows:
1.  Reserved
2.  User ID
3.  Record ID
4.  Record Length After Header
5.  Description
===================================================================*/

//#pragma pack(8)

struct VariableLengthRecordHeader{
	unsigned short reserved;
	char userID[16];
	unsigned short recordID;
	unsigned short recordLength;
	char description[32];
};

/*===================================================================
Structure for POINT DATA RECORD FORMAT
The fields included in the point data record are as follows:
1.  X
2.  Y
3.  Z
4.  Intensity
5.  Return Number
6.  Number of Returns (given pulse)
7.  Scan Direction Flag
8.  Edge of Flight Line
9.  Classification
10. Scan Angle Rank (-90 to +90) - Left Side
11. User Data
12. Point Source ID
13. GPS Time 
14. Red
15. Green
16. Blue

Notes:
POINT DATA RECORD FORMAT 0:   1-12 (20 bytes)
POINT DATA RECORD FORMAT 1:   1-13 (28 bytes)
POINT DATA RECORD FORMAT 2:   1-12,14-16 (26 bytes)
POINT DATA RECORD FORMAT 3:   1-16 (34 bytes)

===================================================================*/

struct PointDataRecord{
	long x;
	long y;
	long z;
	unsigned short intensity;
	unsigned char mask;                   // 0-2 bits: Return Number; 
	                                      // 3-5 bits: Number of Returns
	                                      // 6 bit:    Scan Direction Flag
	                                      // 7 bit.    Edge of Flight Line
	unsigned char classification;
	unsigned char scanAngle;
	unsigned char userData;
	unsigned short pointSourceID;
	double GPS;
	unsigned short red;
	unsigned short green;
	unsigned short blue;

};

/*===================================================================
Structure for Point3D
x, y and z correspond to the x, y and z coordination of a point.
===================================================================*/

struct Point3D{
	double x;
	double y;
	double z;
	int classification;
};

struct Point3DI{
	double x;
	double y;
	double z;
	unsigned short intensity;
	int classification;
};

/*===================================================================
Class for CLasFileReader
This class is used to read the information from an LAS file.
===================================================================*/

class CLasOperator{

public:
	 CLasOperator();                                      // Constructor
	~ CLasOperator();                                     // Destructor
	bool readLasFile(const char *file_name);              // Read an LAS file
	bool readXYZFile(const char *fil_ename);              // Read an XYZ file
	                                                      // Export data with specific type
	bool exportData(const char *file_name,                
         bool RGB = false,                                 // RBG value
	     bool intensity = false,                           // Intensity value
	     bool returnNum = false,                           // Return number
	     bool numOfReturns = false,                        // Number of returns (given pulse)
	     bool scandir = false,                             // Scan direction
	     bool flightline = false,                          // Edge of the flight line
	     bool classification = false,                      // Classification
	     bool GPS = false);                                // GPS time                      

	bool lasSampling(const char *file_name, unsigned long inter); // Sample the las file 
	bool xyzSampling(const char *file_name, unsigned long inter); // Sample the xyz file
	unsigned long pointNum();                             // Return the number of point data record
	std::vector<Point3D>& getPointData();                 // Return the points width x,y,z coordinations 
	std::vector<PointDataRecord>& getPointRecords();      // Return the point data records
	PublicHeaderBlock& getPublicHeader();                 // Return the public header block
	bool isLasFile();                                     // Las file or not
	bool available();                                     // Data available or not
	void clearData();  // Clear all the information

private:
	bool readPulicHeaderBlock(std::fstream &file);       // Read public header block
	bool readVariableLengthRecords(std::fstream &file);  // Read variable length records
	bool readPointDataRecords(std::fstream &file);       // Read point data records
	
	std::string filename;                                // Name of the LAS file
	PublicHeaderBlock pHeader;                           // Holding public header block
	std::vector<VariableLengthRecordHeader> varHeader;   // Holding variable length record headers
	std::vector<PointDataRecord> pointData;              // Holding point data records
	std::vector<Point3D> point3d;                        // Holding point data with x,y,z
	bool isLas;                                          // Flag indicating whether the imported file is an las file
	bool avail;  // Flag indicating whether an las or XYZ file has been imported
};

//恢复默认对齐，这个很重要，没这句话BUG一堆，原因在于pcd默认字节对齐和不对齐在某些操作上会出错
#pragma pack()

//God wish
//一段读取pcd点云和las点云格式文件的函数

//Added in 2014.05.03 by huang
void HPDpointclouddataread(const std::string,pcl::PointCloud<pcl::PointXYZ>::Ptr &,std::vector<Point3D> &,int type=1);
//读取带Intensity格式的点云
void HPDpointcloudallread(const std::string,pcl::PointCloud<pcl::PointXYZI>::Ptr &,std::vector<Point3DI> &,int type=1);
//las和pcd互转的函数
void Tranlasorpcdfile(pcl::PointCloud<pcl::PointXYZ>::Ptr &,std::vector<Point3D> &,char *fname);




#endif
