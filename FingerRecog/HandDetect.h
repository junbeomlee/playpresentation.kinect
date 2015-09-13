#include <vector>

#define CONST_AREA 15000

using namespace cv;

class CHandDetect {
	static const int cDepthRange = 150;

private:
	Mat				m_detectMat;
	Point			m_handPoint;
	int				m_handDepth;
	Point			m_wristPoint;
public:
	CHandDetect();
	CHandDetect(Mat depthMat, Point handPoint);
	~CHandDetect();
	void PreProcess(const Mat depthMat, Point handPoint);

	void SetMember(Mat depthMat, Point handPoint);
	Mat GetDetectMat() { return m_detectMat; };
	Point GetHandPoint() { return m_handPoint; };
	Point GetWristPoint() { return m_wristPoint; };
	int GetHandDepth() { return m_handDepth; };
	void FindHand();
	void RemoveArm(Point3d direction, Point wristPoint, Point hand, Point elbow);
	double LineFunction(double a, double b, Point point);
	int GetCircle(Mat srcMat, Point center, double radius, float* dstData, Point hand, Point elbow);
	int FindWrist(Mat srcMat, Point hand, Point elbow, double radius, Point& wrist);
	int DetectHand(Mat distMat, Point hand, Point wrist, Mat& handMat);
	int IsMinDist(Mat distMat, Point curPoint, int index, float& minDist, Point& minPoint);
	int FindNextPixel(Mat distMat, Point curPoint, Point& nextPoint);
	int IsInRange(Point point);
};

