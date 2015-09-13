#pragma once

#include "std.h"
#include "sphereData.h"
#include "stdafx.h"
#define PointSet vector<Point3d>
#define RHO_0 1
#define RHO_1 1e-5
#define EPSILON_0 1
#define EPSILON_1 1e-3

#define PARTICLE 20
#define GENERATION 10

class Similarity
{
public:
	Similarity();
	Similarity(double x, double y, double z, double Phi, double Theta, double Psi);
	Similarity(Point3d axis, double angle);
	~Similarity();
	void printSim();
	void transfo(handSphere src, handSphere& result, Point3d& center);
	void transfo(handSphere src, handSphere& result, Point3d& center, int start, int end);
	void generate_randomParam();
	void pick_randomVector();

	friend Similarity& operator - (const Similarity S1, const Similarity S2);
	friend Similarity& operator + (const Similarity S1, const Similarity S2);

	double quarterAngle;
	double phi, theta, psi; //x, y, z
	double pBestVal;
	double mx, t11, t12, t13;
	double my, t21, t22, t23;
	double mz, t31, t32, t33;
};

