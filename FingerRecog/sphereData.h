#ifndef SPHEREDATA_H
#define SPHEREDATA_H
#include "stdafx.h"
#include "std.h"
#include <vector>
#include "handSphere.h"
using namespace std;

#define SPHERE_FILE "data\\sphere.txt"
#define SPHERE_NUM 48
class sphereData
{
public:
	sphereData();
	inline void fileRelease(){ inFile.close(); }
	void dataLoad();
	void moveCenter(Point2d center, int index);
	void printData(int index);
	void minimize_finger(double scale, int index);

	void modify_scale(double centerDepth, int index);
	inline handSphere getData(int index) { return data[index]; }
	double curAngleData[400][5][6]; // 관절들끼리 이루는 각도를 저장
private:
	ifstream inFile;
	handSphere data[400];
};


#endif
