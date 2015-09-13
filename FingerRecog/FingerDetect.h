#pragma once
#include "std.h"
#include "stdafx.h"
#define INPUT_WIDTH 512
#define INPUT_HEIGHT 424

#define CRITIC_FINGER 400 //20*20
#define PALM_RAD 21000 //손바닥 중심으로부터의 반경을 구하기위한 기준 상수 //upsampling시 무조건 수정
#define FINGER_RATIO 1.7
#define SEGMENT 3
#define DETECTED 2
#define NOTFINGER 4
#define FINGER_RAD 14000 // 손끝으로부터 z-finger를 구하기위한 반경

//160,90
class CFingerDetect
{
public:
	CFingerDetect();
	~CFingerDetect();

	void cvtestVeiw(const Mat& src);//Mat에 저장된 정보를 그림형태로 보는거
	void cvdistrans(const Mat& src);//cv::distanceTransform()

	void makeDistMap(vector<Point2i>& handCloud);
	void makeDepthMap(Mat& src);
	void makeFingerZero(Point2i fingertip,int index); //손가락이 검출이 될때마다 이 함수를 불러 data값을 zero로 만든다.
	void makeFingerDetected(int x, int y, double stdDepth, bool & isSegment);
	

	void recursiveZero(int x, int y);
	void recursiveDetected(int x, int y,int stdx, int stdy, double stdDepth, double preDepth, bool &isSegment); //x,y지점을 DETECTED상태로 만드는 재귀함수
	

	
	void z_fingerRecog(int mdx, int mdy, double depth, bool isSegment, Mat& src);
	////////////////새로운 xy_finger 알고리즘//////////////
	bool xy_fingerRecog(int mdx, int mdy, double dist);
	int nextContour(Point2i& p, const float& contourVal);
	Point2d calCenterPoint(Point2i p1, Point2i p2);
	void makeDistZero(int mdx, int mdy);
	void recursiveDistZero(int x, int y, int& stdx, int& stdy, double &stdDist);
	void recursive_connect_mcp(Point2i& p1, Point2i& p2);
	void makeSegment(int mdx, int mdy);
	void recursiveSegment(int mdx, int mdy);
	void print_cv();
	void print_tipPoint_cv();
	void print_xyFinger(Mat& src);
	void print_zFinger(Mat& src);

	void enlarge_finger(vector <largeFinger> & vLargeFinger);
	///////////////////////////////////////////////////////////
	void detectXYfinger(Mat&src);
	void detectZfinger(Mat& src);
	bool detectFingerTip(int &x, int& y);//손끝의 XY좌표를 검출, 못찾으면 false값을 반환
	bool detectFingerTip_z(int &mdx, int&mdy);
	void detectFinger_mcp(int x, int y);//xy는 손끝 좌표
	bool findContour(Point2i& p, const float& contourVal); //xy좌표 주변의 윤곽을 찾아준다.
	bool addContour(Point2i& p, const float& contourVal);

	

	void test_FingerInfo(int index);
	void test_printHand();
	void test_printSegment();
	void test_printDepthMap();
	void test_printDistVal();
	void print_fingerResult();
	inline void makeZero(int x, int y);//해당좌표의 distMap과 distVal값을 0으로 만든다.

	inline Mat getm_dist() { return m_dist; }
	inline Point2i get_wrist() { return m_wristPoint; }

	inline void set_mask_center(int x, int y, double z) { m_mask_center_X = x; m_mask_center_y = y; m_mask_center_Z = z; }
	inline void set_wristPoint(int x, int y) { m_wristPoint.x = x; m_wristPoint.y = y; }
	int finger_cnt;
	vector<extremePoint> m_vfinger;
	int m_mask_center_X;
	int m_mask_center_y;
	double m_mask_center_Z;
	double m_adepthMap[DEPTH_HEIGHT][DEPTH_WIDTH];
private:
	Mat m_dist; //opencv를 이용한 distancetransform 을 저장할 매트릭스

	float m_adistMap[DEPTH_HEIGHT][DEPTH_WIDTH]; //m_dist의 distance값을 저장하여 반복적으로 업데이트 한다.
	double m_adistVal[DEPTH_HEIGHT][DEPTH_WIDTH];
	
	int minX, minY, maxX, maxY;

	Point2i m_wristPoint;
};

double calDist(Point2i p1, Point2i p2);
double calDist(Point2d p1, Point2d p2);
double calDist(Point3d p1, Point3d p2);
void changeToken(int& token);
void marking_Point(int x, int y, Mat& mat_data, double val);
void marking_Point(int x, int y, Mat& mat_data, int val0, int val1, int val2);