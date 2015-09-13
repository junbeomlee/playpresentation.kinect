#include "stdafx.h"
#include "handSphere.h"
#include <string>
void handSphere::loadData(ifstream &inFile)
{
	for (int i = 0; i < 48; i++)
	{
		int p; //part, index
		double x, y, z,r;
		inFile >> p;
		inFile >> x;
		inFile >> y;
		inFile >> z;
		inFile >> r;
		(*this).push_back(sphere(p, x, y, -z + 130, r));
	}
}

void handSphere::printData()
{
	string partName[6] = { "thumb", "index", "middle", "ring", "little", "palm"};
	for (int i = 0; i < 48; i++)
	{
		cout << partName[(*this)[i].partindex] << " : " << (*this)[i].x << ", " << (*this)[i].y << ", " << (*this)[i].z << ", rad-> " << (*this)[i].rad << endl;
	}
}

void handSphere::operator () (const handSphere& src)
{
	resize(48);
	for (int i = 0; i < 48; i++)
	{
		(*this)[i] = src[i];
	}
}