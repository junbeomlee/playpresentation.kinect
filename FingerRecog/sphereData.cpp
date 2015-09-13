#include "stdafx.h"
#include "sphereData.h"

sphereData::sphereData()
{
	for (int i = 0; i < 400; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			for (int k = 0; k < 6; k++)
			{
				curAngleData[i][j][k] = 1000;
			}
		}
	}

	inFile.open(SPHERE_FILE); 
}
void sphereData::dataLoad()
{
	// data[i] : i번째 handpose
	for (int i = 0; i < 400; i++)
	{
		handSphere S;
		S.loadData(inFile);
		data[i] = S;
	}
}

void sphereData::printData(int index)
{
	data[index].printData();
}

void sphereData::moveCenter(Point2d center, int index)
{
	for (int i = 0; i < SPHERE_NUM; i++)
	{
		data[index][i].x -= center.x;
		data[index][i].y -= center.y;
	}
}

void sphereData::modify_scale(double centerDepth, int index)
{
	double stdDepth = 500;

	double mod_constant = centerDepth - stdDepth; //모든 Point의 깊이값을 이만큼 빼줘 500거리에 맞춘다.
	for (int i = 0; i < SPHERE_NUM; i++)
	{
		double scale = data[index][i].z / (data[index][i].z - mod_constant);
		data[index][i].x *= scale;
		data[index][i].y *= scale;

		data[index][i].z -= mod_constant;
	}
}

void sphereData::minimize_finger(double scale, int index)
{
	//30번 sphere를 중심으로 0~5를 축소
	Point3d center = Point3d(data[index][30].x, data[index][30].y, data[index][30].z);
	for (int i = 0; i <= 5; i++)
	{
		data[index][i].x = (data[index][i].x - center.x) * scale + center.x;
		data[index][i].y = (data[index][i].x - center.x) * scale + center.y;
		data[index][i].z = (data[index][i].x - center.x) * scale + center.z;
	}
}