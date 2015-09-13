#pragma once

#include "std.h"
#include <vector>
#include "handJoint.h"
#include "stdafx.h"
#include "std.h"
using namespace std;

#define JOINT_FILE "data\\joint.txt"
#define JOINT_NUM 21
class jointData
{
public:
	jointData(){ inFile.open(JOINT_FILE); }
	inline void fileRelease(){ inFile.close(); }
	void dataLoad();
	void printData(int index);
	void minimize_finger(double scale, int index);
	void moveCenter(Point2d center,int index);
	void modify_scale(double centerDepth, int index);
	inline handJoint getData(int index) { return data[index]; }
	
private:
	ifstream inFile;
	handJoint data[400];
};


