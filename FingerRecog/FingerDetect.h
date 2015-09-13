#pragma once
#include "std.h"
#include "stdafx.h"
#define INPUT_WIDTH 512
#define INPUT_HEIGHT 424

#define CRITIC_FINGER 400 //20*20
#define PALM_RAD 21000 //�չٴ� �߽����κ����� �ݰ��� ���ϱ����� ���� ��� //upsampling�� ������ ����
#define FINGER_RATIO 1.7
#define SEGMENT 3
#define DETECTED 2
#define NOTFINGER 4
#define FINGER_RAD 14000 // �ճ����κ��� z-finger�� ���ϱ����� �ݰ�

//160,90
class CFingerDetect
{
public:
	CFingerDetect();
	~CFingerDetect();

	void cvtestVeiw(const Mat& src);//Mat�� ����� ������ �׸����·� ���°�
	void cvdistrans(const Mat& src);//cv::distanceTransform()

	void makeDistMap(vector<Point2i>& handCloud);
	void makeDepthMap(Mat& src);
	void makeFingerZero(Point2i fingertip,int index); //�հ����� ������ �ɶ����� �� �Լ��� �ҷ� data���� zero�� �����.
	void makeFingerDetected(int x, int y, double stdDepth, bool & isSegment);
	

	void recursiveZero(int x, int y);
	void recursiveDetected(int x, int y,int stdx, int stdy, double stdDepth, double preDepth, bool &isSegment); //x,y������ DETECTED���·� ����� ����Լ�
	

	
	void z_fingerRecog(int mdx, int mdy, double depth, bool isSegment, Mat& src);
	////////////////���ο� xy_finger �˰���//////////////
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
	bool detectFingerTip(int &x, int& y);//�ճ��� XY��ǥ�� ����, ��ã���� false���� ��ȯ
	bool detectFingerTip_z(int &mdx, int&mdy);
	void detectFinger_mcp(int x, int y);//xy�� �ճ� ��ǥ
	bool findContour(Point2i& p, const float& contourVal); //xy��ǥ �ֺ��� ������ ã���ش�.
	bool addContour(Point2i& p, const float& contourVal);

	

	void test_FingerInfo(int index);
	void test_printHand();
	void test_printSegment();
	void test_printDepthMap();
	void test_printDistVal();
	void print_fingerResult();
	inline void makeZero(int x, int y);//�ش���ǥ�� distMap�� distVal���� 0���� �����.

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
	Mat m_dist; //opencv�� �̿��� distancetransform �� ������ ��Ʈ����

	float m_adistMap[DEPTH_HEIGHT][DEPTH_WIDTH]; //m_dist�� distance���� �����Ͽ� �ݺ������� ������Ʈ �Ѵ�.
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