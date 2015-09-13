#include "stdafx.h"
#include "jointData.h"

void jointData::dataLoad()
{
	// data[i] : i��° handpose
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
	double mod_constant = centerDepth - stdDepth; //��� Point�� ���̰��� �̸�ŭ ���� 500�Ÿ��� �����.
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
	//17�� joint�� �߽����� ���
	Point3d center = Point3d(data[index][17].x, data[index][17].y, data[index][17].z);
	for (int i = 18; i <= 20; i++)
	{
		data[index][i].x = (data[index][i].x - center.x) * scale + center.x;
		data[index][i].y = (data[index][i].x - center.x) * scale + center.y;
		data[index][i].z = (data[index][i].x - center.x) * scale + center.z;
	}
}