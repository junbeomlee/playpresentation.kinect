#include "stdafx.h"
#include "jointData.h"

void jointData::dataLoad()
{
	// data[i] : i번째 handpose
	for (int i = 0; i < 400; i++)
	{
		handJoint J;
		J.loadData(inFile);
		data[i] = J;
	}
}

void jointData::printData(int index)
{
	data[index].printData();
}

void jointData::moveCenter(Point2d center, int index)
{
	for (int i = 0; i < JOINT_NUM; i++)
	{
		data[index][i].x -= center.x;
		data[index][i].y -= center.y;
	}
}

void jointData::modify_scale(double centerDepth, int index)
{
	double stdDepth = 500;
	double mod_constant = centerDepth - stdDepth; //모든 Point의 깊이값을 이만큼 빼줘 500거리에 맞춘다.
	for (int i = 0; i < JOINT_NUM; i++)
	{
		double scale = data[index][i].z / (data[index][i].z - mod_constant);
		data[index][i].x *= scale;
		data[index][i].y *= scale;

		data[index][i].z -= mod_constant;
	}
}

void jointData::minimize_finger(double scale, int index)
{
	//17번 joint를 중심으로 축소
	Point3d center = Point3d(data[index][17].x, data[index][17].y, data[index][17].z);
	for (int i = 18; i <= 20; i++)
	{
		data[index][i].x = (data[index][i].x - center.x) * scale + center.x;
		data[index][i].y = (data[index][i].x - center.x) * scale + center.y;
		data[index][i].z = (data[index][i].x - center.x) * scale + center.z;
	}
}