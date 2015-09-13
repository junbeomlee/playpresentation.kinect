#ifndef HANDSPHERE_H
#define HANDSPHERE_H

#include "sphere.h"
#include <vector>
#include "std.h"
#include "stdafx.h"
using namespace std;

class handSphere : public vector < sphere >
{
public:
	void loadData(ifstream &inFile);
	
	void printData();
	void operator () (const handSphere& src);
};

#endif