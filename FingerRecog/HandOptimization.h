#pragma once
#include "std.h"
#include "Similarity.h"
#include "sphereData.h"
#include "jointData.h"
#include "handGL.h"
#include "HandInitialize.h"
#include "stdafx.h"
typedef vector<Point3d> pointcloud;

#define THUMB 0
#define INDEX 1
#define MIDDLE 2
#define RING 3
#define LITTLE 4

#define REAL 0
#define PIXEL 1
#define BOTH 2
class CHandOptimization
{
public:
	CHandOptimization();
	~CHandOptimization();

	void generate_randomParameter();
	void loadInitHandPose(int index, double wrist_x, double wrist_y); //m_optJoint, m_optSphere에 inithandpose 데이터를 담아온다.
	void optimization_transRealspaceTospace();
	void corres_sphere(int particleNum); //pointcloud와 sphere를 매칭시킨다.
	void minimize_finger(double scale);

	void optimization_handPose();
	void optimization_palm();
	void optimization_finger(int fingerNum);
	void optimization_thumb();
	double optimization_depth();
	double costFunc_palm(int particleNum, Similarity S);
	double costFunc_depth(int particleNum, Similarity S);
	double costFunc_finger(Similarity S, Point3d center, int start, int end, int cost_type);
	double dist_error(Point3d p, sphere s);


	void optimization_ICP();
	void optimization_gd(int particleNum); // gradient descent
	void optimization_PSO(int& gbestIndex, double& gBestVal);
	Point3d get_wristPoint();
	void display_handPose();
	void print_resultHand_cv();
	void print_handSphere_cv(const handSphere& s, int type, char* name, Point2d center);

	Similarity m_param[PARTICLE];
	Similarity m_randomVector[PARTICLE];
	Similarity m_pbestSim[PARTICLE];

	handSphere m_optSphere;
	Point3d m_optCenter;
	Point3d m_optWrist;
	pointcloud m_vhandCloud;
	int m_a2match_sphere[PARTICLE][256];
	double m_depthErrorList[PARTICLE];
	int m_optimaParticleNum;
	int initHandIndex;
};

double calDist(Point3d p1, sphere p2);
Point3d est_normalVector(Point3d v1, Point3d v2);
Point3d est_perVector(Point3d axis, Point3d thPoint, Point3d srcPoint);
Point3d calVector(sphere p1, sphere p2);
Point3d calVector(Point3d p1, Point3d p2);