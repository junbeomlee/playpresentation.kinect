#ifndef SPHERE_H
#define SPHERE_H
#include "stdafx.h"
class sphere
{
public:
	sphere(){}
	sphere(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
	sphere(int p, double a, double b, double c, double r) : partindex(p), x(a), y(b), z(c), rad(r) {}

	double x,y,z, rad;
	int partindex; 
};

//partindex
//0. thumb
//1. index
//2. middle
//3. ring
//4. little
//5. palm

#endif