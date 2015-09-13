#pragma once
#include "std.h"
#include "FingerDetect.h"
#include "sphereData.h"
#include "jointData.h"
#include "stdafx.h"
#define TIP 4
#define DB_WIDTH 320
#define DB_HEIGHT 240

//#define DEPTH_WIDTH 320
//#define DEPTH_HEIGHT 240

#define ANGLE_WEIGHT 1000
#define SAMPLING_NUM 256

#define DEFAULT_HANDPOSE_INDEX 25
class CHandInitialize
{
public:
	CHandInitialize();
	CHandInitialize(int mask_center_x, int mask_center_y, double mask_center_z, int wrist_x, int wrist_y);
	~CHandInitialize();

	void finger_detect(Mat& src);
	void enlarge_finger();
	void scaling_sample(vector <Point3d>& cloud, Point2d modCenter);

	void print_largeFinger_cv();
	void print_sampleCloud_cv(vector <Point3d> cloud);

	int est_initHandPose(const Mat& input_src);
	int test_est_initHandPose(const Mat& input_src);

	double tipError(joint jo1, handJoint jo2, vector<int>& matchIndex);
	double fingerAngleError(Point3d v1, Point3d v2);


	double test_tipError(joint jo1, handJoint jo2, vector<int>& matchIndex);
	double test_fingerAngleError(Point2d v1, Point2d v2);

	inline double transX(int x, double wrist_x){ return (x - m_detFinger.get_wrist().x) + wrist_x; }
	inline double transY(int y, double wrist_y){ return (m_detFinger.get_wrist().y - y) + wrist_y; }
	inline vector<Point2i> get_handCloud(){ return m_vHandCloud; }

	void cloud_sampling(const Mat& src, vector<Point3d>& cloudPoint, int initial_index);
	void test_cloud_sampling(const Mat& src, vector<Point2d>& cloudPoint, int initial_index);
	CFingerDetect m_detFinger;
	vector <largeFinger> m_vlargeFinger;
	vector<Point2i>m_vHandCloud;
private:
	//vector<Point3d> m_vinputFingerTip;
	vector<extremePoint> m_transFinger;
	
};

//좌표변환은 wrist 지점을 원점으로 하여 xy좌표축을 그린 좌표다 (다르게 말하면 손목으로 부터 거리차라고 보면 된다)
double inner_product(Point3d a, Point3d b);
double vector_abs(int x, int y, int z);
double vector_abs(Point3d v);
Point3d TransSphereToSpace(SphericalCoordinate spherePoint);
Point3d TransSpaceToRealSpace(Point3d spacePoint, double angle);
sphere TransSpaceToRealSpace(sphere spacePoint, Point2d center);

Point3d TransSpaceToRealSpace(Point3d spacePoint, Point2d center);
Point3d TransRealSpaceToSpace(Point3d r, Point2d center);
Point3d TransRealSpaceToSpace(Point3d r);
sphere TransRealSpaceToSpace(sphere r, Point2d center);
void print_Joint_cv(int index);
void print_Sphere_cv(int index);