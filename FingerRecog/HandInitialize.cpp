#include "stdafx.h"
#include "HandInitialize.h"

jointData g_jData;
sphereData g_sData;
void print_Sphere_cv(int index)
{
	Mat mat_data = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);
	double aveX = 0, aveY = 0;

	for (int i = 0; i < SPHERE_NUM; i++)
	{
		Point3d r = Point3d(g_sData.getData(index)[i].x, g_sData.getData(index)[i].y, g_sData.getData(index)[i].z);
		//r = TransSpaceToRealSpace(r, center);
		Point3d ttt =
			Point3d(r.x + DEPTH_WIDTH / 2,
			DEPTH_HEIGHT / 2 - r.y,
			r.z);

		//ttt = TransRealSpaceToSpace(ttt);
		//ttt = TransSpaceToRealSpace(ttt, 0);
		int x = ttt.x;

		int y = ttt.y;

		mat_data.at<UINT16>(y, x) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y - 1, x) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y + 1, x) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y + 1, x + 1) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y, x + 1) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y - 1, x + 1) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y + 1, x - 1) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y, x - 1) = pow(2, 16) - 1;
		mat_data.at<UINT16>(y - 1, x - 1) = pow(2, 16) - 1;
	}

	imshow("initHand", mat_data);
}

void print_Joint_cv(int index)
{
	Mat mat_data = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);
	for (int i = 0; i < JOINT_NUM; i++)
	{
		if (i % 4 == 0 || i % 4 == 1){
			Point3d ttt =
				Point3d(g_jData.getData(index)[i].x + DEPTH_WIDTH / 2,
				DEPTH_HEIGHT / 2 - g_jData.getData(index)[i].y,
				g_jData.getData(index)[i].z);

			//ttt = TransRealSpaceToSpace(ttt);
			int x = ttt.x;
			int y = ttt.y;
			mat_data.at<UINT16>(y, x) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y - 1, x) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y + 1, x) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y + 1, x + 1) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y, x + 1) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y - 1, x + 1) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y + 1, x - 1) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y, x - 1) = pow(2, 16) - 1;
			mat_data.at<UINT16>(y - 1, x - 1) = pow(2, 16) - 1;
		}
	}
	marking_Point(INPUT_WIDTH / 2, INPUT_HEIGHT / 2, mat_data, pow(2, 16) - 1);
	imshow("tipjoint", mat_data);
}

double vector_abs(int x, int y, int z)
{
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

double vector_abs(int x, int y)
{
	return sqrt(pow(x, 2) + pow(y, 2));
}

double vector_abs(Point2d v)
{
	return sqrt(pow(v.x, 2) + pow(v.y, 2));
}
double vector_abs(Point3d v)
{
	return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

double inner_product(Point3d a, Point3d b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

double inner_product(Point2d a, Point2d b)
{
	return a.x * b.x + a.y * b.y;
}

Point3d TransSphereToSpace(SphericalCoordinate spherePoint)
{
	double x = spherePoint.radian*cos(spherePoint.phi)*sin(spherePoint.theta);
	double y = spherePoint.radian*sin(spherePoint.phi);
	double z = spherePoint.radian*cos(spherePoint.phi)*cos(spherePoint.theta);

	return Point3d(x, y, z);
}

Point3d TransSpaceToRealSpace(Point3d spacePoint, double angle)
{
	return Point3d(0.00275 * spacePoint.x * spacePoint.z, 0.00275 * spacePoint.y * spacePoint.z, spacePoint.z);
}

sphere TransSpaceToRealSpace(sphere p, Point2d center)
{
	sphere result = p;
	result.x = 0.00275 * (p.x - center.x) * p.z + center.x;
	result.y = 0.00275 * (p.y - center.y) * p.z + center.y;
	return result;
}

Point3d TransSpaceToRealSpace(Point3d p, Point2d center)
{
	Point3d result(p.x - center.x, p.y - center.y, p.z);
	result.x = 0.00275 * result.x * p.z + center.x;
	result.y = 0.00275 * result.y * p.z + center.y;
	return result;
}

Point3d TransRealSpaceToSpace(Point3d r)
{
	return Point3d(
		r.x / 0.00275 / r.z,
		r.y / 0.00275 / r.z,
		r.z);
}

sphere TransRealSpaceToSpace(sphere r, Point2d center)
{
	sphere temp = r;
	temp.x = (r.x - center.x) / 0.00275 / temp.z + center.x;
	temp.y = (r.y - center.y) / 0.00275 / temp.z + center.y;
	temp.z = r.z;
	return temp;
}

Point3d TransRealSpaceToSpace(Point3d r, Point2d center)
{
	return Point3d(
		(r.x - center.x) / 0.00275 / r.z + center.x,
		(r.y - center.y) / 0.00275 / r.z + center.y,
		r.z);
}

CHandInitialize::CHandInitialize()
{

}

CHandInitialize::CHandInitialize(int mask_center_x, int mask_center_y, double mask_center_z, int wrist_x, int wrist_y)
{
	m_detFinger.set_mask_center(mask_center_x, mask_center_y, mask_center_z);
	m_detFinger.set_wristPoint(wrist_x, wrist_y);
}
CHandInitialize::~CHandInitialize()
{
}

void CHandInitialize::finger_detect(Mat& src)
{
	cout << "finger_detect() " << endl;
	m_detFinger.cvdistrans(src); //src를 distance transform시킨다.
	//detFinger.cvtestVeiw(detFinger.getm_dist( ));
	m_detFinger.makeDistMap(m_vHandCloud);
	//m_detFinger.test_printDistVal();
	m_detFinger.detectXYfinger(src);
	m_detFinger.print_xyFinger(src);
	return;
	//TODO Z-finger에서 extremepoint가 segment를 건드릴때만 false;
	m_detFinger.detectZfinger(src);
	//m_detFinger.print_fingerResult();
	//m_detFinger.print_xyFinger(src);
	m_detFinger.print_zFinger(src);
	
	/*for (int i = 0; i < m_detFinger.m_vfinger.size(); i++)
	{
		m_detFinger.test_FingerInfo(i);
	}*/
	/*double maxDist = 0;
	for (int i = 0; i < m_detFinger.m_vfinger.size(); i++)
	{
		for (int j = i+1; j < m_detFinger.m_vfinger.size(); j++)
		{
			double val = calDist(m_detFinger.m_vfinger[i].fingerTip, m_detFinger.m_vfinger[j].fingerTip);
			if (val > maxDist)
				maxDist = val;
		}
	}
	cout << "손가락 최대거리 : " << maxDist << endl;
	m_detFinger.print_cv();	*/

	
	
	////////////data CV veiw//////////////
	//Mat data375 = imread("test375(157,76).jpg", 1);
	//imshow("data375", data375);
	
	//m_detFinger.print_tipPoint_cv();
}

double CHandInitialize::tipError(joint jo1, handJoint jo2,vector<int>& matchIndex)
{//여기서 jo1은 픽셀좌표고, jo2는 원점 좌표다
	//error값과 matching된 손가락이 나온다.
	double minError = 10000000;
	int minIndex;

	for (int i = 1; i <= 5; i++)
	{
		double error = vector_abs(jo1.x - (jo2[i*TIP].x + INPUT_WIDTH / 2), jo1.y - (INPUT_HEIGHT / 2 - jo2[i*TIP].y));
			//jo1.z - jo2[i*TIP].z);
		if (minError > error){
			minError = error;
			minIndex = i;
		}
	}
	matchIndex.push_back(minIndex);
	
	return minError;

}

double CHandInitialize::test_tipError(joint jo1, handJoint jo2, vector<int>& matchIndex)
{
	//error값과 matching된 손가락이 나온다.
	double minError = 10000000;
	int minIndex;
	for (int i = 1; i <= 5; i++)
	{
		double error = vector_abs(jo1.x - jo2[i*TIP].x, jo1.y - jo2[i*TIP].y);
		if (minError > error){
			minError = error;
			minIndex = i;
		}
	}
	matchIndex.push_back(minIndex);

	return minError;

}

int CHandInitialize::est_initHandPose(const Mat& input_src)
{
	cout << "est_initHandPose()" << endl;
	/*for (int i = 0; i < m_vlargeFinger.size(); i++)
	{
		cout << i << "번째 Tip : " << m_vlargeFinger[i].fingerTip.x << ", " << m_vlargeFinger[i].fingerTip.y << ", " << m_vlargeFinger[i].fingerTip.z <<endl;
		cout << i << "번째 Mcp : " << m_vlargeFinger[i].fingerMcp.x << ", " << m_vlargeFinger[i].fingerMcp.y << ", " << m_vlargeFinger[i].fingerMcp.z<<endl;
	}*/

	double errorList[400];
	int minIndex=0;
	double minError = 10000000;

	for (int i= 0; i < 400; i++)
	{
		if (i == 375)
			cout << 375 << "번 data break." << endl;
		double error = 0;
		vector<int> matchIndex;
		///손끝거리구하기
		for (int j = 0; j < m_vlargeFinger.size(); j++)
		{
				double tip_x = m_vlargeFinger[j].fingerTip.x;
				double tip_y = m_vlargeFinger[j].fingerTip.y;
				double tip_z = m_vlargeFinger[j].fingerTip.z;
				error += tipError(joint(tip_x, tip_y, tip_z), g_jData.getData(i), matchIndex);
		}
		///손가락방향구하기
		double beforeError = error;
		for (int j = 0; j < m_vlargeFinger.size(); j++)
		{
				Point3d inputVector;
				inputVector.x = m_vlargeFinger[j].fingerTip.x - m_vlargeFinger[j].fingerMcp.x;
				inputVector.y = m_vlargeFinger[j].fingerTip.y - m_vlargeFinger[j].fingerMcp.y;
				inputVector.z =m_vlargeFinger[j].fingerTip.z - m_vlargeFinger[j].fingerMcp.z;
				Point3d dataVector; //Tip과 mcp를 뺀 벡터(방향)
				dataVector.x = g_jData.getData(i)[matchIndex[j]*4].x - g_jData.getData(i)[matchIndex[j]*4 - 2].x;
				dataVector.y = - (g_jData.getData(i)[matchIndex[j] * 4].y - g_jData.getData(i)[matchIndex[j] * 4 - 2].y);
				dataVector.z = g_jData.getData(i)[matchIndex[j] * 4].z - g_jData.getData(i)[matchIndex[j] * 4 - 2].z;
				error += fingerAngleError(inputVector, dataVector);
		}
		errorList[i] = error;
		///
		if (minError > error){
			minError = error;
			minIndex = i;
			//cout << minIndex << ", " << minError << endl;
		}
		//cout << "거리 error = " << beforeError << endl;
		//cout << "각도 error = " << errorList[i] - beforeError << endl;
	}
	
	return minIndex;
}

int CHandInitialize::test_est_initHandPose(const Mat& input_src)
{
	int minIndex;
	double minError = 10000000;

	//for (int l = 1; l <= 5; l++){
	//	cout << l << ": " << g_jData.getData(i)[l*TIP].x << ", " << g_jData.getData(i)[l*TIP].y << endl;
	//}
	for (int i = 0; i < 400; i++)
	{
		double error = 0;
		vector<int> matchIndex;
		
		for (int j = 0; j < m_detFinger.m_vfinger.size(); j++)
		{
			if (m_detFinger.m_vfinger[j].isFinger)
			{

				double tip_x = m_detFinger.m_vfinger[j].fingerTip.x;
				double tip_y = m_detFinger.m_vfinger[j].fingerTip.y;
				//double tip_z = input_src.at<UINT16>(tip_y, tip_x);
				double wrist_x = g_jData.getData(i)[0].x;
				double wrist_y = g_jData.getData(i)[0].y;

				tip_x = transX(tip_x, wrist_x);
				tip_y = transY(tip_y, wrist_y);
				error += test_tipError(joint(tip_x, tip_y, 0), g_jData.getData(i), matchIndex);
			}
			else
				matchIndex.push_back(-1);
		}
		double angleE = 0;
		for (int j = 0; j < m_detFinger.m_vfinger.size(); j++)
		{
			if (m_detFinger.m_vfinger[j].isFinger){

				Point2d inputVector;
				inputVector.x = m_detFinger.m_vfinger[j].fingerTip.x - m_detFinger.m_vfinger[j].fingerMcp.x;
				inputVector.y = m_detFinger.m_vfinger[j].fingerTip.y - m_detFinger.m_vfinger[j].fingerMcp.y;
				
				Point2d dataVector; //Tip과 mcp를 뺀 벡터(방향)
				dataVector.x = g_jData.getData(i)[matchIndex[j] * 4].x - g_jData.getData(i)[matchIndex[j] * 4 - 3].x;
				dataVector.y = - (g_jData.getData(i)[matchIndex[j] * 4].y - g_jData.getData(i)[matchIndex[j] * 4 - 3].y);
				double AE = test_fingerAngleError(inputVector, dataVector);
				error += AE;
				angleE += AE;
			}
		}
		if (i == 106 || i == 162)
			cout << i << "'s angleError : " << angleE << endl;
		if (minError > error){
			minError = error;
			minIndex = i;
			//cout << minIndex << ", " << minError << endl;
		}
		if (i == 162)
			cout << i << "'s error : " << error << endl;

	}
	cout << minIndex<<", " << minError <<endl;
	return minIndex;
}

double CHandInitialize::fingerAngleError(Point3d v1, Point3d v2)
{
	Point3d real_v1 = v1;
	Point3d real_v2 = v2;
	return acos(inner_product(real_v1, real_v2) / (vector_abs(real_v1)*vector_abs(real_v2))) / (2 * PI) * ANGLE_WEIGHT;
}

double CHandInitialize::test_fingerAngleError(Point2d v1, Point2d v2)
{
	return acos(inner_product(v1, v2) / (vector_abs(v1)*vector_abs(v2))) / (2 * PI) * ANGLE_WEIGHT;
}

void CHandInitialize::cloud_sampling(const Mat& src,vector<Point3d>& cloudPoint,int initial_index)
{//input data(hand)로부터 256개의 sampling 과정을 거친다.
	srand(time(0));
	cout << "CHandInitialize::cloud_sampling()" << endl; 
	cloudPoint.resize(SAMPLING_NUM);
	if (m_vHandCloud.size() != 0)
	{
		for (int i = 0; i < SAMPLING_NUM; i++){
			int randNum = rand() % m_vHandCloud.size();
			cloudPoint[i] =
				Point3d(	m_vHandCloud[randNum].x - DEPTH_WIDTH / 2,
				DEPTH_HEIGHT / 2 - m_vHandCloud[randNum].y,
				src.at<UINT16>(m_vHandCloud[randNum].y, m_vHandCloud[randNum].x) / 10);
			//cout << "샘플 깊이 : " << src.at<UINT16>(m_vHandCloud[randNum].y, m_vHandCloud[randNum].x) / 10;
			m_vHandCloud.erase(m_vHandCloud.begin() + randNum);
		}
		scaling_sample(cloudPoint,
			Point2d(m_detFinger.m_mask_center_X - DEPTH_WIDTH / 2, DEPTH_HEIGHT / 2 - m_detFinger.m_mask_center_y));
		//scaling_sample에서는 뎁스맵에서 sampling한 256개의 point들을 중심깊이 500으로 맞추는 과정이다.
		for (int i = 0; i < SAMPLING_NUM; i++)
		{ //현재 cloudPoint[i]는 원점좌표를 갖는다.
			cloudPoint[i] = TransSpaceToRealSpace(cloudPoint[i],
				Point2d(m_detFinger.m_mask_center_X - DEPTH_WIDTH /2 , DEPTH_HEIGHT / 2 - m_detFinger.m_mask_center_y));
		}
		//print_sampleCloud_cv(cloudPoint);
	}
	
}

void CHandInitialize::test_cloud_sampling(const Mat& src, vector<Point2d>& cloudPoint, int initial_index)
{//input data(hand)로부터 256개의 sampling 과정을 거친다.
	srand(time(0));
	double wrist_x = g_jData.getData(initial_index)[0].x;
	double wrist_y = g_jData.getData(initial_index)[0].y;
	/*int **test_map = new int*[INPUT_HEIGHT];
	for (int i = 0; i < INPUT_HEIGHT; i++){
		test_map[i] = new int[INPUT_WIDTH];
		for (int j = 0; j < INPUT_WIDTH; j++)
		{
			test_map[i][j] = 0;
		}
	}*/

	

	for (int i = 0; i < 256; i++){
		int randNum = rand() % m_vHandCloud.size();
		//test_map[m_vHandCloud[randNum].y][m_vHandCloud[randNum].x] = 1;
		cloudPoint.push_back(
			Point2d(transX(m_vHandCloud[randNum].x, wrist_x),
			transY(m_vHandCloud[randNum].y, wrist_y)));
		//cout << "test : " << (double)m_vHandCloud[randNum].x << ", " << (double)m_detFinger.get_wrist().x << ", " << (double)transX(m_vHandCloud[randNum].x, m_detFinger.get_wrist().x) << endl;
		m_vHandCloud.erase(m_vHandCloud.begin() + randNum);
	}
	
	//int minX = 91;
	//int maxX = 210;
	//int minY = 12;
	//int maxY = 100;

	//for (int i = minY; i <= maxY; i++)
	//{
	//	for (int j = minX; j <= maxX; j++)
	//	{
	//		cout << test_map[i][j] << " ";
	//	}
	//	cout << endl;
	//}
	//for (int i = 0; i < INPUT_HEIGHT; i++)
	//	delete [] test_map[i];
	//delete[] test_map;
}

void CHandInitialize::enlarge_finger()
{
	m_detFinger.enlarge_finger(m_vlargeFinger);
}

void CHandInitialize::scaling_sample(vector <Point3d>& cloud, Point2d modCenter)
{
	//중심으로 옮긴후 확대시킨다음 다시 제자리에 갖다놓는다.
	cout << "scaling_sample" << endl;
	double stdDepth = 500;
	
	double mod_constant = m_detFinger.m_mask_center_Z - stdDepth; //모든 Point의 깊이값을 이만큼 빼줘 500거리에 맞춘다.
	for (int i = 0; i < cloud.size(); i++)
	{
		double scale = cloud[i].z / (cloud[i].z - mod_constant);
		cloud[i].x = (cloud[i].x - modCenter.x) * scale + modCenter.x;
		cloud[i].y = (cloud[i].y - modCenter.y) * scale + modCenter.y;
		cloud[i].z -= mod_constant;
	}
}

void CHandInitialize::print_largeFinger_cv()
{
	cout << "print_largeFinger_cv()" << endl;
	Mat mat_data = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);

	for (int i = 0; i < m_vlargeFinger.size(); i++)
	{
		marking_Point((int)m_vlargeFinger[i].fingerTip.x, (int)m_vlargeFinger[i].fingerTip.y, mat_data, pow(2, 16) - 1);
		marking_Point((int)m_vlargeFinger[i].fingerMcp.x, (int)m_vlargeFinger[i].fingerMcp.y, mat_data, pow(2, 16) - 1);
	}
	marking_Point(INPUT_WIDTH / 2, INPUT_HEIGHT / 2, mat_data, pow(2, 16) - 1);

	imshow("largeFinger", mat_data);
}

void CHandInitialize::print_sampleCloud_cv(vector <Point3d> cloud)
{
	Mat mat_data = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);
	for (int i = 0; i < cloud.size(); i++)
	{
		int x = cloud[i].x + DEPTH_WIDTH / 2;
		int y = DEPTH_HEIGHT / 2 - cloud[i].y;
		if (x < 0 || y < 0 || x> INPUT_WIDTH || y >INPUT_HEIGHT)
			return;
		marking_Point(x, y, mat_data, pow(2, 16) - 1);
	}
	imshow("sampling", mat_data);
}

	