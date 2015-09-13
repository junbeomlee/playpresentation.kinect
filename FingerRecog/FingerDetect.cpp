#include "stdafx.h"
#include "FingerDetect.h"
#define forx(j) for(int j=minX; j<=maxX; j++)
#define fory(i) for(int i=minY; i<=maxY;i++)
#define forHEIGHT(i) for(int i=0;i<INPUT_HEIGHT;i++)
#define forWIDTH(j) for(int j=0;j<INPUT_WIDTH;j++)


int bo[DEPTH_HEIGHT][DEPTH_WIDTH] = {};
float cVal;
extern Mat g_disTransMat;
void clearBoard()
{
	for (int i = 0; i < DEPTH_HEIGHT; i++)
	{
		for (int j = 0; j < DEPTH_WIDTH; j++)
		{
			bo[i][j] = 0;
		}
	}
}
CFingerDetect::CFingerDetect()
{
	finger_cnt = 0;
	for (int i = 0; i < DEPTH_HEIGHT; i++)
	{
		for (int j = 0; j < DEPTH_WIDTH; j++)
		{
			m_adistVal[i][j] = 0;
		}
	}
	minX = 1000, minY = 1000, maxX = 0, maxY = 0;
}


CFingerDetect::~CFingerDetect()
{
}


void CFingerDetect::cvdistrans(const Mat& src)
{
	Mat bw;
	src.convertTo(bw, CV_8UC1);
	//cvtColor(src, bw, CV_BGR2GRAY);
	threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	distanceTransform(bw, g_disTransMat, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	distanceTransform(bw, m_dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	normalize(m_dist, m_dist, 0, 1., NORM_MINMAX);
	normalize(g_disTransMat, g_disTransMat, 0, 1., NORM_MINMAX);
}

void CFingerDetect::cvtestVeiw(const Mat&src)
{
	//imshow("Distance Transform Image", src);
}

void CFingerDetect::makeDistMap(vector<Point2i>& handCloud)
{
	cout << "makeDistMap() " << endl;
	//int minVal=0x7fffffff;
	handCloud.resize(20000);
	int top = 0;
	//cout << "�Ӱ谪 -> " << 12000 / m_mask_center_Z << endl;
	for (int i = 0; i < DEPTH_HEIGHT; i++)
	{
		for (int j = 0; j < DEPTH_WIDTH; j++)
		{
			float val = m_dist.at<float>(i, j);
			if (val != 0) //���� �մ��� Ȯ��
			{
				bo[i][j] = 1000;
				handCloud[top++]= Point2i(j, i);
				//�߽������κ��� �Ÿ��� ���Ͽ� �Ӱ谪�� ����ġ�°����� ���͸��Ѵ�
				double distVal = sqrt(pow(m_mask_center_X - j, 2) + pow(m_mask_center_y - i, 2));
				m_adistMap[i][j] = val;
				////////////////�ܼ� �׽�Ʈ///////////////
				if (val != 0)
				{
					if (j < minX)
						minX = j;
					if (j > maxX)
						maxX = j;
					if (i < minY)
						minY = i;
					if (i > minY)
						maxY = i;
				}
				///////////////////////////////////////////
				
				if (distVal> PALM_RAD / m_mask_center_Z)
				{
					m_adistVal[i][j] = distVal;
					//if (val < minVal)
					//	minVal = val; //�� �۾��� �ʿ��ұ�?
				}
			}
		}
	}
	handCloud.resize(top);
	/*for (int i = minY; i < maxY; i++)
	{
		for (int j = minX; j < maxX; j++)
		{
			if (m_adistVal[i][j] > 0)
				cout << 1 << " ";
			else
				cout << 0 << " ";
		}
		cout << endl;
	}*/
}

bool CFingerDetect::detectFingerTip(int& x, int& y)
{
	double maxVal = 0;
	//	cout << "detectFingertIP() -> maxVal : ";
	for (int i = 0; i < INPUT_HEIGHT; i++)
	{
		for (int j = 0; j < INPUT_WIDTH; j++)
		{
			if (m_adistVal[i][j] > maxVal)
			{
				maxVal = m_adistVal[i][j];
				y = i;
				x = j;
			}
		}
	}
	//cout << maxVal << endl;
	return maxVal; //0�̸� false�̰� tip�� ã���� ture�� �ȴ�.
}

void CFingerDetect::detectXYfinger(Mat& src)
{
	int maxx, maxy;
	cout << "detectXYfinger()" << endl;
	for (int i = 0; i < 6; i++)
	{
		//cout << i <<"��° detect " << endl;
		if (!detectFingerTip(maxx, maxy)){ //�ճ� ã��

			break;//������ �ݺ��� ����
		}
		cVal = m_adistMap[maxy][maxx]; //contour�� �ֱ�
		//test_printDistVal();
		makeDistZero(maxx, maxy);
		
		//test_printDistVal();
		if (xy_fingerRecog(maxx, maxy, 0))
		{
			forx(i)
			{
				fory(j)
				{
					if (m_adistMap[i][j] == DETECTED)
						m_adistMap[i][j] = SEGMENT;
				}
			}
			makeSegment(maxx, maxy);
			cout << "makeSegment() �Ϸ�" << endl;
			m_vfinger[m_vfinger.size() - 1].tip_depth =
				src.at<UINT16>(m_vfinger[m_vfinger.size() - 1].fingerTip.y, m_vfinger[m_vfinger.size() - 1].fingerTip.x) / 10;
			m_vfinger[m_vfinger.size() - 1].mcp_depth =
				src.at<UINT16>(m_vfinger[m_vfinger.size() - 1].fingerMcp.y, m_vfinger[m_vfinger.size() - 1].fingerMcp.x) / 10;
			
			//test_FingerInfo(m_vfinger.size() - 1);
		}
		else
		{
			forx(i)
			{
				fory(j)
				{
					if (m_adistMap[i][j] == DETECTED)
						m_adistMap[i][j] = cVal;
				}
			}
		}
		//detectFinger_mcp(maxx, maxy);//�հ��� ���ۺκ� ã�Ƽ� m_vfinger�� ����
		//makeFingerZero(m_vfinger[i].fingerTip, i);//Ž���� �հ����� map�� zero�� �����.
		//test_printSegment();
	}
	//test_printSegment();
	/*cout << "�հ����� ��� ã�ҽ��ϴ�. ���� : " << m_vfinger.size() << endl;
	for (int i = 0; i < m_vfinger.size(); i++)
	{
		test_FingerInfo(i);
	}*/
	//imshow("1",m_dist);
	//test_printHand();

	//test_printSegment();
}
double calDist(Point2i p1, Point2i p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double calDist(Point2d p1, Point2d p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double calDist(Point3d p1, Point3d p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}
void changeToken(int& token)
{
	if (token == 0)
		token = 1;
	else
		token = 0;
}
void CFingerDetect::detectFinger_mcp(int x, int y)
{
	//cout << "detectFinger_mcp() test : " << endl;
	float contourVal = m_adistMap[y][x];
	//cout << "contourVal = " << contourVal << endl;
	makeZero(x, y);

	int token = 0;
	Point2i p[2];
	p[0] = Point2i(x, y);
	p[1] = Point2i(x, y);

	//cout << "fist x : " << x << ", y : " << y << endl;

	double distData[1000];
	int dIndex = 0;
	double dist = 0;
	while (findContour(p[token], contourVal))
	{

		makeZero(p[token].x, p[token].y); //�̵��� �κ��� ���� zero�� �����
		double nextDist = calDist(p[0], p[1]);
		if (dist < nextDist){ //�Ÿ��� ��� �����Ұ�� token�� ���� �����Ұ�� token������� ��� �̵�
			changeToken(token);
		}
		dist = nextDist;
		distData[dIndex++] = dist;

	}

	double maxData = 0;
	int i, j;
	for (i = dIndex - 1, j = 0; i >= 0 && j<10; i--, j++)
	{
		if (maxData < distData[i])
			maxData = distData[i];
	}
	//cout << "�հ��� �߰� ���� " << endl;
	//�߰������� �հ����� Ȯ���� ������.
	while (addContour(p[token], contourVal))
	{
		makeZero(p[token].x, p[token].y);
		double nextDist = calDist(p[0], p[1]);
		if (nextDist > maxData * 1.3)
			break;
		if (dist < nextDist){ //�Ÿ��� ��� �����Ұ�� token�� ���� �����Ұ�� token������� ��� �̵�
			changeToken(token);
		}
		dist = nextDist;
		distData[dIndex++] = dist;
		///test///
	}
	//cout << "p[0].x : " << p[0].x << ", p[0].y : " << p[0].y << "\t p[1].x : " << p[1].x << ", p[1].y : " << p[1].y << endl;
	//cout << "dist : " << dist << endl;
	extremePoint finger;
	finger.fingerTip.x = x;
	finger.fingerTip.y = y;
	finger.fingerMcp.x = (p[0].x + p[1].x) / 2;
	finger.fingerMcp.y = (p[0].y + p[1].y) / 2;
	finger.fingerLength = calDist(finger.fingerMcp, finger.fingerTip);
	cout << "���� : " << m_mask_center_Z << ", ���� : " << finger.fingerLength << endl;
	//finger.fingerWidth = calDist(p[0], p[1]);
	finger.isFinger = (finger.fingerLength > 15000 / m_mask_center_Z) ||
		(finger.fingerLength > 5000 / m_mask_center_Z&& finger.fingerLength / calDist(p[0], p[1]) > 1.1);
	finger.test_mcp1 = p[0];
	finger.test_mcp2 = p[1];
	if (finger.isFinger)
		finger_cnt++;
	m_vfinger.push_back(finger);
}

void CFingerDetect::makeZero(int x, int y)
{
	m_adistMap[y][x] = (float)DETECTED;
	m_adistVal[y][x] = 0;
}

bool CFingerDetect::findContour(Point2i& p, const float& contourVal)
{//p�� �������� �̵������ش�.
	Point2i move[8] = { Point2i(-1, -1), Point2i(-1, 0), Point2i(-1, 1), Point2i(0, 1), Point2i(0, -1), Point2i(1, -1), Point2i(1, 0), Point2i(1, 1) };
	for (int i = 0; i < 8; i++)
	{
		if (p.y + move[i].y >= 0 && p.y + move[i].y < INPUT_HEIGHT && p.x + move[i].x >= 0 && p.x + move[i].x <INPUT_WIDTH &&
			m_adistVal[p.y + move[i].y][p.x + move[i].x] != 0 && m_adistMap[p.y + move[i].y][p.x + move[i].x] == contourVal)
		{
			p.y += move[i].y;
			p.x += move[i].x;
			return true;
		}
	}
	return false;
}

bool CFingerDetect::addContour(Point2i& p, const float& contourVal)
{
	Point2i move[8] = { Point2i(-1, -1), Point2i(-1, 0), Point2i(-1, 1), Point2i(0, 1), Point2i(0, -1), Point2i(1, -1), Point2i(1, 0), Point2i(1, 1) };
	for (int i = 0; i < 8; i++)
	{
		if (p.y + move[i].y >= 0 && p.y + move[i].y < INPUT_HEIGHT && p.x + move[i].x >= 0 && p.x + move[i].x <INPUT_WIDTH &&
			m_adistMap[p.y + move[i].y][p.x + move[i].x] == contourVal)
		{
			p.y += move[i].y;
			p.x += move[i].x;
			return true;
		}
	}
	return false;
}

void CFingerDetect::makeFingerZero(Point2i fingertip, int index) //fingertip��ǥ�� �Է����� �ָ� �ȴ�.
{
	if (!m_vfinger[index].isFinger)
		return;
	Point2i cur;//fingertip
	Point2i move[8] = { Point2i(-1, -1), Point2i(-1, 0), Point2i(-1, 1), Point2i(0, 1), Point2i(0, -1), Point2i(1, -1), Point2i(1, 0), Point2i(1, 1) };
	bool s = false;
	for (int i = 0; i < 8; i++)
	{
		if (fingertip.y + move[i].y >= 0 && fingertip.y + move[i].y < INPUT_HEIGHT && fingertip.x + move[i].x >= 0 && fingertip.x + move[i].x <INPUT_WIDTH &&
			m_adistVal[fingertip.y + move[i].y][fingertip.x + move[i].x] != 0)
		{
			cur = Point2i(fingertip.x + move[i].x, fingertip.y + move[i].y);
			s = true;
		}
	}
	if (!s)
		return;
	Point2i mcp_cur = m_vfinger[index].test_mcp1;
	while (mcp_cur != m_vfinger[index].test_mcp2)
	{
		m_adistMap[mcp_cur.y][mcp_cur.x] = (float)SEGMENT;
		if (m_vfinger[index].test_mcp2.x != mcp_cur.x)
		{//x�������� ���� �̵�
			mcp_cur.x += (m_vfinger[index].test_mcp2.x - mcp_cur.x) / abs(m_vfinger[index].test_mcp2.x - mcp_cur.x);
			m_adistMap[mcp_cur.y][mcp_cur.x] = (float)SEGMENT;
		}
		if (m_vfinger[index].test_mcp2.y != mcp_cur.y)
		{//x�������� ���� �̵�
			mcp_cur.y += (m_vfinger[index].test_mcp2.y - mcp_cur.y) / abs(m_vfinger[index].test_mcp2.y - mcp_cur.y);
			m_adistMap[mcp_cur.y][mcp_cur.x] = (float)SEGMENT;
		}
	}

	//�ճ������� �߽����� �ֺ��� distVal�� 0�� �ƴ������� ���� ã�´�.
	recursiveZero(cur.x - 1, cur.y);
	recursiveZero(cur.x + 1, cur.y);
	//	recursiveZero(cur.x + 1, cur.y + 1);
	//	recursiveZero(cur.x + 1, cur.y - 1);
	recursiveZero(cur.x, cur.y + 1);
	recursiveZero(cur.x, cur.y - 1);
	//	recursiveZero(cur.x - 1, cur.y + 1);
	//	recursiveZero(cur.x - 1, cur.y - 1);
}

void CFingerDetect::recursiveZero(int x, int y)
{
	if (x < 0 || y < 0 || x >= INPUT_WIDTH || y >= INPUT_HEIGHT)
		return;

	if (m_adistMap[y][x] == (float)DETECTED)
	{
		m_adistMap[y][x] == (float)SEGMENT;
		return;
	}

	if (m_adistMap[y][x] == (float)SEGMENT)
		return;
	m_adistVal[y][x] = 0;
	m_adistMap[y][x] = (float)SEGMENT;
	recursiveZero(x - 1, y);
	recursiveZero(x + 1, y);
	recursiveZero(x, y + 1);
	recursiveZero(x, y - 1);
	//recursiveZero(x + 1, y + 1);
	//recursiveZero(x + 1, y - 1);
	//recursiveZero(x - 1, y + 1);
	//recursiveZero(x - 1, y - 1);
}
void CFingerDetect::test_FingerInfo(int i)
{
	cout << i + 1 << "��° �հ��� Tip : " << m_vfinger[i].fingerTip.x << ", " << m_vfinger[i].fingerTip.y << ", "
		<< m_vfinger[i].tip_depth<< endl;
	cout << i + 1 << "��° �հ��� Mcp : " << m_vfinger[i].fingerMcp.x << ", " << m_vfinger[i].fingerMcp.y << ", "
		<< m_vfinger[i].mcp_depth <<endl;
}

void CFingerDetect::test_printHand()
{
	
	for (int i = minY; i <= maxY; i++)
	{
		for (int j = minX; j <= maxX; j++)
		{
			if (m_dist.at<float>(i, j) > 0)
			{
				bo[i][j] = 1;
			}
			else
				bo[i][j] = 0;
		}
	}
	
	for (int i = 0; i < m_vfinger.size(); i++)
	{
		
		bo[m_vfinger[i].test_mcp1.y][m_vfinger[i].test_mcp1.x] = 7;
		bo[m_vfinger[i].test_mcp2.y][m_vfinger[i].test_mcp2.x] = 7;
		bo[m_vfinger[i].fingerMcp.y][m_vfinger[i].fingerMcp.x] = 7;
		bo[m_vfinger[i].fingerTip.y][m_vfinger[i].fingerTip.x] = 7;
	}

	bo[m_mask_center_y][m_mask_center_X] = 8;
	for (int i = minY; i <= maxY; i++)
	{
		for (int j = minX; j <= maxX; j++)
		{
			cout << bo[i][j] << " ";
		}
		cout << endl;
	}
	cout << "--------------------------------------------------------------------------" << endl;
}

void CFingerDetect::test_printSegment()
{

	for (int i = minY; i <= maxY; i++)
	{
		for (int j = minX; j <= maxX; j++)
		{
			if (i == m_mask_center_y && m_mask_center_X == j)
				cout << 8 << " ";
			else if (m_adistMap[i][j] == (float)SEGMENT)
			{
				cout << 9 << " ";
			}
			else if (m_adistMap[i][j] == cVal)
				cout <<5 << " ";
			else if (m_adistMap[i][j] == DETECTED)
				cout << 7 << " ";
			else if (m_adistMap[i][j] == NOTFINGER)
				cout << 6 << " ";
			else if (m_adistMap[i][j] > 0)
				cout << 1 << " ";
			else
				cout << 0 << " ";
		}
		cout << endl;
	}
}
void CFingerDetect::makeDepthMap(Mat& src)
{
	for (int i = 0; i < DEPTH_HEIGHT; i++)
	{
		for (int j = 0; j < DEPTH_WIDTH; j++)
		{
			if (src.at<UINT16>(i, j) != 0)
			{
				if (src.at<UINT16>(i, j) / 10. < m_mask_center_Z + 100){
					m_adepthMap[i][j] = src.at<UINT16>(i, j) / 10.;
				}
				else
					m_adepthMap[i][j] = 0;
			}
			else
				m_adepthMap[i][j] = 0;
		}
	}
}

bool CFingerDetect::detectFingerTip_z(int& mdx, int&mdy)
{
	//cout << "detectFingerTip_z()" << endl;// ->�˻������׽�Ʈ..." << endl;
	double minDepth = 1000000;
	for (int i = minY; i <= maxY; i++)
	{
		for (int j = minX; j <= maxX; j++)
		{
			if (m_adepthMap[i][j] != 0 && m_adepthMap[i][j] != DETECTED && m_adepthMap[i][j] != SEGMENT
				&& m_adepthMap[i][j] != NOTFINGER &&m_adepthMap[i][j] < minDepth)
			{
				minDepth = m_adepthMap[i][j];
				mdx = j;
				mdy = i;
			}
		}
	}

	return mdx != -1 && mdy !=-1;
}
void CFingerDetect::test_printDepthMap()
{
	for (int i = minY; i <= maxY; i++)
	{
		for (int j = minX; j <= maxX; j++)
		{
			if (m_adepthMap[i][j] == DETECTED)
				cout << 7 << " ";
			else if (m_adepthMap[i][j] == SEGMENT)
				cout << 4 << " ";
			else if (i == m_mask_center_y && j == m_mask_center_X)
				cout << 8 << " "; //���߽�
			/*else if (xmin == j && ymin == i)
				cout << 2 << " ";*/
			else if (m_adepthMap[i][j] > 0 || m_adepthMap[i][j] ==NOTFINGER)
				cout << 1 << " ";
			else
				cout << 0 << " ";
		}
		cout << endl;
	}
}
void CFingerDetect::makeFingerDetected(int mdx, int mdy, double stdDepth, bool& isSegment)
{
	if (m_adistMap[mdy][mdx] == SEGMENT)
	{
		isSegment = true;
	}
	m_adepthMap[mdy][mdx] = DETECTED;
	recursiveDetected(mdx + 1, mdy, mdx, mdy, stdDepth, stdDepth, isSegment);
	recursiveDetected(mdx - 1, mdy, mdx, mdy, stdDepth, stdDepth, isSegment);
	recursiveDetected(mdx, mdy + 1, mdx, mdy, stdDepth, stdDepth, isSegment);
	recursiveDetected(mdx, mdy - 1, mdx, mdy, stdDepth, stdDepth, isSegment);

}

void CFingerDetect::recursiveDetected(int x, int y, int mdx, int mdy, double stdDepth, double preDepth, bool &isSegment)
{
	if (x < 0 || y < 0 || x>= DEPTH_WIDTH || y >= DEPTH_HEIGHT) //������ �����
		return;
	if (m_adepthMap[y][x] == DETECTED || m_adepthMap[y][x] == NOTFINGER || m_adepthMap[y][x] == SEGMENT || m_adepthMap[y][x] == 0)// �̹� Ž���߰ų� ����̸�
		return;
	double curDepth = m_adepthMap[y][x];
	if (sqrt(pow(mdx - x, 2) + pow(mdy - y, 2)) > 12000/stdDepth
		|| m_adepthMap[y][x] - stdDepth > 70 || curDepth - preDepth > 30)
		return; //���� ����Ʈ�� �����Ÿ��� �����
	/*if (m_adistMap[y][x] == SEGMENT)
		isSegment = true;*/
	

	if (sqrt(pow(mdx - x, 2) + pow(mdy - y, 2)) > 5000 / stdDepth
		|| m_adepthMap[y][x] - stdDepth > 30)
	{
		m_adepthMap[y][x] = NOTFINGER;
		if (preDepth < curDepth)
		{
			recursiveDetected(x + 1, y, mdx, mdy, stdDepth, curDepth, isSegment);
			recursiveDetected(x - 1, y, mdx, mdy, stdDepth, curDepth, isSegment);
			recursiveDetected(x, y + 1, mdx, mdy, stdDepth, curDepth, isSegment);
			recursiveDetected(x, y - 1, mdx, mdy, stdDepth, curDepth, isSegment);
		}
	}
	else{
		m_adepthMap[y][x] = DETECTED;
		recursiveDetected(x + 1, y, mdx, mdy, stdDepth, curDepth, isSegment);
		recursiveDetected(x - 1, y, mdx, mdy, stdDepth, curDepth, isSegment);
		recursiveDetected(x, y + 1, mdx, mdy, stdDepth, curDepth, isSegment);
		recursiveDetected(x, y - 1, mdx, mdy, stdDepth, curDepth, isSegment);
	}
}

void CFingerDetect::z_fingerRecog(int mdx, int mdy, double depth, bool isSegment, Mat& src)
{
	int innerCnt = 0;
	int outterCnt = 0;

	double innerRad;
	double outterRad;

	double maxDepth = 0;
	int mx, my;

	if (depth < 1000){
		innerRad = 4;
		outterRad = 36;
		if (!isSegment){
			for (int i = minY; i <= maxY; i++)
			{
				for (int j = minX; j <= maxX; j++)
				{
					if (m_adepthMap[i][j] == DETECTED)
					{
						double val = pow(mdx - j, 2) + pow(mdy - i, 2);
						if (val <= innerRad)
							innerCnt++;
						else if (val > innerRad && val <= outterRad)
							outterCnt++;
						//m_adepthMap[i][j] == SEGMENT;
					}
				}
			}
		}

		

		if (innerCnt >= 10 && outterCnt <= 40){
			//cout << "YES" << endl;
			for (int i = minY; i <= maxY; i++)
			{
				for (int j = minX; j <= maxX; j++)
				{
					if (m_adepthMap[i][j] == DETECTED)
					{
						m_adepthMap[i][j] = SEGMENT;
						if (maxDepth < src.at<UINT16>(i, j))
						{
							maxDepth = src.at<UINT16>(i, j);
							mx = j;
							my = i;
						}
					}
				}
			}
		}
		else{
			isSegment = true;
			//cout << "NO -> " << innerCnt <<", " <<outterCnt << endl;
			for (int i = minY; i <= maxY; i++)
			{
				for (int j = minX; j <= maxX; j++)
				{
					if (m_adepthMap[i][j] == DETECTED)
					{
						m_adepthMap[i][j] = NOTFINGER;
					}
				}
			}
		}
	}
	else if (depth < 1500){
		//cout << "depth 1500 " << endl;
		innerRad = 2;
		outterRad = 18;
		if (!isSegment){
			for (int i = minY; i <= maxY; i++)
			{
				for (int j = minX; j <= maxX; j++)
				{
					if (m_adepthMap[i][j] == DETECTED)
					{
						double val = pow(mdx - j, 2) + pow(mdy - i, 2);
						if (val <= innerRad)
							innerCnt++;
						else if (val > innerRad && val <= outterRad)
							outterCnt++;
						//m_adepthMap[i][j] == SEGMENT;
					}
				}
			}
		}
		if (innerCnt >= 8 && outterCnt <= 7){
			//cout << "YES" << endl;
			//src�޾Ƽ� segment�� ���� ���� �Ÿ����� ã�´�.
			for (int i = minY; i <= maxY; i++)
			{
				for (int j = minX; j <= maxX; j++)
				{
					if (m_adepthMap[i][j] == DETECTED)
					{
						m_adepthMap[i][j] = SEGMENT;
						if (maxDepth < src.at<UINT16>(i, j))
						{
							maxDepth = src.at<UINT16>(i, j);
							mx = j;
							my = i;
						}
					}
				}
			}


		}
		else{
			//cout << "NO" << endl;
			isSegment = true;
			for (int i = minY; i <= maxY; i++)
			{
				for (int j = minX; j <= maxX; j++)
				{
					if (m_adepthMap[i][j] == DETECTED)
					{
						m_adepthMap[i][j] = NOTFINGER;
					}
				}
			}
		}
	}
	else //���� �� �̻��� �Ÿ��� ���ؼ��� �ڵ带 ���� �ʾҴ�.
	{
		isSegment = true;
	}
	if (!isSegment){
		extremePoint finger;
		finger.fingerTip = Point2i(mdx, mdy);
		finger.fingerMcp = Point2i(mx, my);
		finger.mcp_depth = maxDepth / 10;
		finger.tip_depth = src.at<UINT16>(mdy, mdx) / 10;
		finger.isFinger = true;
		m_vfinger.push_back(finger);
	}
}
void CFingerDetect::detectZfinger(Mat& src)
{
	makeDepthMap(src);
	for (int i = 0; i < 1; i++)
	{
		int mdx = -1, mdy = -1;
		if (!detectFingerTip_z(mdx, mdy))
			break;
		//cout << "�ּұ��� " << mdx << ", " << mdy << endl;
		double stdDepth = m_adepthMap[mdy][mdx]; //extreme Point�� depth
		bool isSegment = false; 
		makeFingerDetected(mdx, mdy, stdDepth, isSegment);

		Mat test_mat = Mat::zeros(600, 600, CV_8UC3);
		forHEIGHT(j)
		{
			forWIDTH(k)
			{
				if (m_adepthMap[j][k] == DETECTED || m_adepthMap[j][k] == SEGMENT)
					test_mat.at<Vec3b>(j, k) = Vec3b(255, 0, 0);
				else if (m_adepthMap[j][k] == NOTFINGER)
					test_mat.at<Vec3b>(j, k) = Vec3b(0, 0, 255);
				else if (m_adepthMap[j][k] > 0)
					test_mat.at<Vec3b>(j, k) = Vec3b(255, 255, 255);
			}
		}
		imshow("zfinger", test_mat);
		return;





		z_fingerRecog(mdx, mdy, stdDepth, isSegment, src);

		//if (i == 4){
		//	(1000);
		////	test_printDepthMap();
		//}
	}
	
	//���� �߽����� depth�� ���Ͽ� ����ġ���� �Ÿ��� �־����� �κе��� �����Ѵ�
	/*double minVal = 10000000; int xmin; int ymin;
	
	cout << "detectZfinger()" << endl;
	for (int i = minY; i < maxY; i++)
	{
		for (int j = minX; j < maxX; j++)
		{
			if (i == m_mask_center_y && j == m_mask_center_X)
				cout << 8 << " ";
			else if (xmin == j && ymin == i)
				cout << 2 << " ";
			else if (m_adepthMap[i][j] > 0)
				cout << 1 << " ";
			else
				cout << 0 << " ";
		}
		cout << endl;
	}*/

	//���� ����� ���� ã�´�
	//�� ������ ���� 
}

int CFingerDetect::nextContour(Point2i& p, const float& contourVal)
{
	/*Point2i t;
	t = Point2i(p.x + 1, p.y);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal
		&& m_adistMap[t.y][t.x] != DETECTED)
	{
		p = t;
		return 1;
	}
	t = Point2i(p.x - 1, p.y);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal)
	{
		p = t;
		return 2;
	}
	t = Point2i(p.x + 1, p.y + 1);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal)
	{
		p = t;
		return 3;
	}
	t = Point2i(p.x, p.y + 1);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal)
	{
		p = t;
		return 4;
	}
	t = Point2i(p.x - 1, p.y + 1);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal)
	{
		p = t;
		return 5;
	}
	t = Point2i(p.x + 1, p.y - 1);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal)
	{
		p = t;
		return 6;
	}
	t = Point2i(p.x, p.y - 1);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal)
	{
		p = t;
		return 7;
	}
	t = Point2i(p.x - 1, p.y - 1);
	if (t.y >= 0 && t.y < INPUT_HEIGHT && t.x >= 0 && t.x < INPUT_HEIGHT && m_adistMap[t.y][t.x] == cVal)
	{
		p = t;
		return 8;
	}*/


	Point2i move[8] = { Point2i(-1, -1), Point2i(-1, 0), Point2i(-1, 1), Point2i(0, 1), Point2i(0, -1), Point2i(1, -1), Point2i(1, 0), Point2i(1, 1) };
	for (int i = 0; i < 8; i++)
	{
		if (p.y + move[i].y >= 0 && p.y + move[i].y < INPUT_HEIGHT)
		{
			if (p.x + move[i].x >= 0 && p.x + move[i].x < INPUT_WIDTH)
			{
				if (m_adistMap[p.y + move[i].y][p.x + move[i].x] == contourVal)
				{
					if (m_adistMap[p.y + move[i].y][p.x + move[i].x] != DETECTED) //&& m_adistVal[p.y + move[i].y][p.x + move[i].x] !=0 
					{
						p.y += move[i].y;
						p.x += move[i].x;
						return i + 1;
					}
				}
					
			}
				
		}
		
	}
	return 0;
}

Point2d CFingerDetect::calCenterPoint(Point2i p1, Point2i p2)
{
	return Point2d((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}
bool CFingerDetect::xy_fingerRecog(int mdx, int mdy, double dist)
{
	cout << "xy_fingerRecog()" << endl;
	Point2i p[2];
	p[0] = Point2i(mdx, mdy); p[1] = p[0];
	float contourVal = m_adistMap[mdy][mdx];
	m_adistMap[mdy][mdx] = DETECTED;

	int stdDist = 5; // ������
	int nextResult;
	//test_printSegment();
	while(calDist(Point(mdx, mdy), p[0]) < stdDist)
	{
		//cout << -1.1 << " : " << p[0].x << ", " << p[0].y << endl;
		if ((nextResult = nextContour(p[0], cVal)) == 0)
			break;
		//if (m_adistMap[p[0].y][p[0].x] == DETECTED)
		//	break;
		//cout << -1.2 << " : " << p[0].x << ", " << p[0].y << ", " << nextResult << ", " << m_adistMap[p[1].y][p[1].x] << endl;
		m_adistMap[p[0].y][p[0].x] = DETECTED;
	}
	//test_printSegment();
	//cout << -1 << endl;

	while(calDist(Point(mdx, mdy), p[1]) < stdDist)
	{
		//cout << -0.1 << " : " << p[1].x << ", " << p[1].y << endl;
		if ((nextResult = nextContour(p[1], cVal)) == 0)
			break;
		//if (m_adistMap[p[1].y][p[1].x] == DETECTED)
		//	break;
		//cout << -0.2 << " : " << p[1].x << ", " << p[1].y << ", " << nextResult << ", " << m_adistMap[p[1].y][p[1].x] <<endl;
		m_adistMap[p[1].y][p[1].x] = DETECTED;
	}
	//test_printSegment();
	//cout << 0 << endl;
	Point2d f_mcp = calCenterPoint(p[0], p[1]);
	double f_length = calDist(Point2d(mdx, mdy), f_mcp); //�ճ��� ���������� �Ÿ�
	double f_width = calDist(p[0], p[1]);
	double f_ratio = f_length / f_width;
	bool isFinger = (f_ratio > 0.55);

	double std_length = f_length; 
	double dist_p0 = calDist(Point2i(mdx, mdy), p[0]);
	double dist_p1 = calDist(Point2i(mdx, mdy), p[1]);
	if (!isFinger){
		cout << "break0 : " << f_ratio << endl;
		return false;
	}
	while (1)
	{
		//cout << 1 << endl;

		Point2i temp[2];
		
		
		if (dist_p1 >= dist_p0){ //p1���� �Ÿ��� p0���� ũ�� p0�� �Ÿ��� �� Ŭ������ �̵��� ��Ų��.
			temp[1] = p[1];
			nextContour(p[1], cVal);
			m_adistMap[p[1].y][p[1].x] = DETECTED;
			dist_p1 = calDist(Point2i(mdx, mdy), p[1]);
			while (dist_p0 <= dist_p1){
				//int test;
				//cout << 1.1 << endl;
				temp[0] = p[0];
				int next = nextContour(p[0], cVal);
				if (!next)
					break;

				//cout << 1.1 << " : " << test << endl;
				//if (m_adistMap[p[0].y][p[0].x] == DETECTED)
				//	break;
				m_adistMap[p[0].y][p[0].x] = DETECTED;
				if (dist_p0 < calDist(Point2i(mdx, mdy), p[0]))
					break;
				dist_p0 = calDist(Point2i(mdx, mdy), p[0]);
			}
		}
		else
		{
			temp[0] = p[0];
			nextContour(p[0], cVal);
			m_adistMap[p[0].y][p[0].x] = DETECTED;
			dist_p0 = calDist(Point2i(mdx, mdy), p[0]);
			while (dist_p1 <= dist_p0){
				
				temp[1] = p[1];
				int next = nextContour(p[1], cVal);
				if (!next)
					break;
				//if (m_adistMap[p[0].y][p[0].x] == DETECTED)
				//	break;
				m_adistMap[p[1].y][p[1].x] = DETECTED;
				if (dist_p1 > calDist(Point2i(mdx, mdy), p[1]))
					break;
				dist_p1 = calDist(Point2i(mdx, mdy), p[1]);
			}
		}
		//cout << 2 << endl;
		//���� ��� �̵���Ų��
		double length, width, ratio;
		Point2d mcp = calCenterPoint(p[0], p[1]);
		length = calDist(Point2d(mdx, mdy), mcp);
		width = calDist(p[0], p[1]);
		ratio = length / width;
		if (length <= f_length)
		{
			m_adistMap[p[0].y][p[0].x] = cVal;
			m_adistMap[p[1].y][p[1].x] = cVal;
			p[0] = temp[0];
			p[1] = temp[1];
			cout << "break 1 : " << length << ", " << width << endl;
			//isFinger = false;
			break;
		}
		if (length > std_length && ratio < 0.6)
		{
			m_adistMap[p[0].y][p[0].x] = cVal;
			m_adistMap[p[1].y][p[1].x] = cVal;
			p[0] = temp[0];
			p[1] = temp[1];
			cout << "break 2 : " << length << ", " << width << endl;
			//isFinger = false;
			break;
		}
		if (length > std_length * 2 && ratio < 0.8)
		{
			m_adistMap[p[0].y][p[0].x] = cVal;
			m_adistMap[p[1].y][p[1].x] = cVal;
			p[0] = temp[0];
			p[1] = temp[1];
			cout << "break 3 : " << length << ", " << width << ", " <<length / width <<endl;
			//isFinger = false;
			break;
		}
		if (length > std_length * 3 && ratio < 1.2)
		{
			m_adistMap[p[0].y][p[0].x] = cVal;
			m_adistMap[p[1].y][p[1].x] = cVal;
			p[0] = temp[0];
			p[1] = temp[1];
			cout << "break 4 : " << length << ", " << width << endl;
			//isFinger = false;
			break;
		}

		if (length > std_length * 4 && ratio < 1.6)
		{
			m_adistMap[p[0].y][p[0].x] = cVal;
			m_adistMap[p[1].y][p[1].x] = cVal;
			p[0] = temp[0];
			p[1] = temp[1];
			cout << "break 5 : " << length << ", " << width << endl;
			//isFinger = false;
			break;
		}

		f_length = length; //f_width = width; f_ratio = ratio;
		
	}
	
	double length, width, ratio;
	Point2d mcp = calCenterPoint(p[0], p[1]);
	length = calDist(Point2d(mdx, mdy), mcp);
	width = calDist(p[0], p[1]);
	ratio = length / width;
	isFinger = ratio > 0.9;
	//test_printSegment();

	if (!isFinger)
	{
		cout << "�հ����� �ƴմϴ�" << endl;
		return false;
	}
	extremePoint finger;
	finger.fingerTip = Point2i(mdx, mdy);
	finger.fingerMcp = mcp;
	finger.test_mcp1 = p[0];
	finger.test_mcp2 = p[1];
	m_vfinger.push_back(finger);
	
	////�ֽ�ȭ�� p[0], p[1]�� ���� ó��
	m_adistMap[p[0].y][p[0].x] = SEGMENT;
	m_adistMap[p[1].y][p[1].x] = SEGMENT;
	recursive_connect_mcp(p[0], p[1]);
	
	return true;
}

void CFingerDetect::makeDistZero(int mdx, int mdy) 
{
	cout << "makeDistZero() " << endl;
	m_adistVal[mdy][mdx] = 0;
	double stdDist = PALM_RAD * m_mask_center_Z *1.5;
	recursiveDistZero(mdx + 1, mdy, mdx, mdy, stdDist);
	recursiveDistZero(mdx - 1, mdy, mdx, mdy, stdDist);
	recursiveDistZero(mdx, mdy + 1, mdx, mdy, stdDist);
	recursiveDistZero(mdx, mdy - 1, mdx, mdy, stdDist);
}
void CFingerDetect::recursiveDistZero(int mdx, int mdy,int &stdx, int& stdy, double & stdDist)
{
	if (mdx < 0 || mdy < 0 || mdx >= DEPTH_WIDTH || mdy >= DEPTH_HEIGHT)
		return;
	if (m_adistVal[mdy][mdx] == 0)
		return;
	/*if (calDist(Point2i(mdx, mdy), Point2i(stdx, stdy)) > stdDist)
		return;*/
	m_adistVal[mdy][mdx] = 0;
	recursiveDistZero(mdx + 1, mdy, stdx, stdy, stdDist);
	recursiveDistZero(mdx - 1, mdy, stdx, stdy, stdDist);
	recursiveDistZero(mdx, mdy + 1, stdx, stdy, stdDist);
	recursiveDistZero(mdx, mdy - 1, stdx, stdy, stdDist);
}

void CFingerDetect::test_printDistVal()
{
	for (int i = minY; i <= maxY; i++)
	{
		for (int j = minX; j <= maxX; j++)
		{
			if (m_adistVal[i][j] != 0)
				cout << 1 << " ";
			else
				cout << 0 << " ";
		}
		cout << endl;
	}
}

void CFingerDetect::recursive_connect_mcp(Point2i& p1, Point2i& p2)
{
	//cout << "recursive_connect_mcp() " << endl;
	Point2i center = Point2i((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
	if (m_adistMap[center.y][center.x] == SEGMENT)
		return;
	m_adistMap[center.y][center.x] = SEGMENT;
	recursive_connect_mcp(p1, center);
	recursive_connect_mcp(center, p2);
}

void CFingerDetect::makeSegment(int mdx, int mdy)
{
	cout << "makeSegment()" << endl;


	m_adistMap[mdy][mdx] = SEGMENT;
	recursiveSegment(mdx + 1, mdy);
	recursiveSegment(mdx + 1, mdy+1);
	recursiveSegment(mdx + 1, mdy-1);
	recursiveSegment(mdx - 1, mdy);
	recursiveSegment(mdx - 1, mdy + 1);
	recursiveSegment(mdx - 1, mdy - 1);
	recursiveSegment(mdx , mdy + 1);
	recursiveSegment(mdx , mdy - 1);
}

void CFingerDetect::recursiveSegment(int mdx, int mdy)
{
	if (mdx < 0 || mdy < 0 || mdx >= DEPTH_WIDTH || mdy >= DEPTH_HEIGHT)
		return;
	if (m_adistMap[mdy][mdx] == SEGMENT)
		return;
	if (m_adistMap[mdy][mdx] == 0)
		return;
	m_adistMap[mdy][mdx] = SEGMENT;
	recursiveSegment(mdx + 1, mdy);
	recursiveSegment(mdx - 1, mdy);
	recursiveSegment(mdx, mdy + 1);
	recursiveSegment(mdx, mdy - 1);
}

void CFingerDetect::print_fingerResult()
{
	fory(i)
	{
		forx(j)
		{
			if (m_adistMap[i][j] != 0)
			{
				if (m_adistMap[i][j] == SEGMENT || m_adepthMap[i][j] == SEGMENT)
					cout << 2 << " ";
				else
					cout << 1 << " ";
			}
			else
				cout << 0 << " ";
		}
		cout << endl;
	}
}

void CFingerDetect::print_cv()
{
	Mat src = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);
	for (int i = 0; i < DEPTH_HEIGHT; i++)
	{
		for (int j = 0; j < DEPTH_WIDTH; j++)
		{
			if (m_adistMap[i][j] == SEGMENT|| m_adepthMap[i][j] == SEGMENT)
			{
				src.at<UINT16>(i, j) = (pow(2, 16) / 2);
			}
			else if (m_adistMap[i][j] == 0)
				src.at<UINT16>(i, j) = 0;
			else
				src.at<UINT16>(i, j) = pow(2, 16) - 1;
		}
	}

	int x = m_mask_center_X; int y = m_mask_center_y;
	src.at<UINT16>(y, x) = pow(2, 16) / 2;
	src.at<UINT16>(y - 1, x) = pow(2, 16) / 2;
	src.at<UINT16>(y +1, x) = pow(2, 16) / 2;
	src.at<UINT16>(y, x - 1) = pow(2, 16) / 2;
	src.at<UINT16>(y - 1 , x - 1) = pow(2, 16) / 2;
	src.at<UINT16>(y + 1, x - 1) = pow(2, 16) / 2;
	src.at<UINT16>(y, x + 1) = pow(2, 16) / 2;
	src.at<UINT16>(y - 1, x + 1) = pow(2, 16) / 2;
	src.at<UINT16>(y + 1, x + 1) = pow(2, 16) / 2;

	imshow("result", src);
}

void CFingerDetect::print_xyFinger(Mat& src)
{
	for (int i = 0; i < DEPTH_HEIGHT; i++)
	{
		for (int j = 0; j < DEPTH_WIDTH; j++)
		{
			if (m_adistMap[i][j] == SEGMENT)
			{
				src.at<UINT16>(i, j) = (pow(2, 16) / 2);
			}
			else if (m_adistMap[i][j] == 0)
				src.at<UINT16>(i, j) = 0;
			else
				src.at<UINT16>(i, j) = pow(2, 16) - 1;
		}
	}
	imshow("xyFinger", src);
}

void CFingerDetect::print_zFinger(Mat &src)
{
	for (int i = 0; i < DEPTH_HEIGHT; i++)
	{
		for (int j = 0; j < DEPTH_WIDTH; j++)
		{
			if (m_adepthMap[i][j] == SEGMENT)
			{
				src.at<UINT16>(i, j) = (pow(2, 16) / 2);
			}
		}
	}
	imshow("zFinger", src);
}

void marking_Point(int x, int y, Mat& mat_data, double val)
{
	if (x < 0 || y <0 || x>INPUT_WIDTH || y>INPUT_HEIGHT)
		return;
	mat_data.at<UINT16>(y, x) = val;
	mat_data.at<UINT16>(y - 1, x) = val;
	mat_data.at<UINT16>(y + 1, x) = val;
	mat_data.at<UINT16>(y + 1, x + 1) = val;
	mat_data.at<UINT16>(y, x + 1) = val;
	mat_data.at<UINT16>(y - 1, x + 1) = val;
	mat_data.at<UINT16>(y + 1, x - 1) = val;
	mat_data.at<UINT16>(y, x - 1) = val;
	mat_data.at<UINT16>(y - 1, x - 1) = val;
}

void marking_Point(int x, int y, Mat& mat_data, int val0, int val1, int val2)
{
	if (x < 0 || y <0 || x>INPUT_WIDTH || y>INPUT_HEIGHT)
		return;
	mat_data.at<Vec3b>(y, x)[0] = val0;
	mat_data.at<Vec3b>(y, x)[1] = val1;
	mat_data.at<Vec3b>(y, x)[1] = val2;
	mat_data.at<Vec3b>(y - 1, x)[0] = val0;
	mat_data.at<Vec3b>(y - 1, x)[1] = val1;
	mat_data.at<Vec3b>(y - 1, x)[2] = val2;
	mat_data.at<Vec3b>(y + 1, x)[0] = val0;
	mat_data.at<Vec3b>(y + 1, x)[1] = val1;
	mat_data.at<Vec3b>(y + 1, x)[2] = val2;
	mat_data.at<Vec3b>(y + 1, x + 1)[0] = val0;
	mat_data.at<Vec3b>(y + 1, x + 1)[1] = val1;
	mat_data.at<Vec3b>(y + 1, x + 1)[2] = val2;
	mat_data.at<Vec3b>(y, x + 1)[0] = val0;
	mat_data.at<Vec3b>(y, x + 1)[1] = val1;
	mat_data.at<Vec3b>(y, x + 1)[2] = val2;

	mat_data.at<Vec3b>(y - 1, x + 1)[0] = val0;
	mat_data.at<Vec3b>(y - 1, x + 1)[1] = val1;
	mat_data.at<Vec3b>(y - 1, x + 1)[2] = val2;
	mat_data.at<Vec3b>(y + 1, x - 1)[0] = val0;
	mat_data.at<Vec3b>(y + 1, x - 1)[1] = val1;
	mat_data.at<Vec3b>(y + 1, x - 1)[2] = val2;

	mat_data.at<Vec3b>(y, x - 1)[0] = val0;
	mat_data.at<Vec3b>(y, x - 1)[1] = val1;
	mat_data.at<Vec3b>(y, x - 1)[2] = val2;
	mat_data.at<Vec3b>(y - 1, x - 1)[0] = val0;
	mat_data.at<Vec3b>(y - 1, x - 1)[1] = val1;
	mat_data.at<Vec3b>(y - 1, x - 1)[2] = val2;
}
void CFingerDetect::print_tipPoint_cv()
{
	Mat mat_data = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);

	for (int i = 0; i < m_vfinger.size(); i++)
	{
		marking_Point(m_vfinger[i].fingerTip.x, m_vfinger[i].fingerTip.y, mat_data, pow(2, 16) - 1);
		marking_Point(m_vfinger[i].fingerMcp.x, m_vfinger[i].fingerMcp.y, mat_data, pow(2, 16) - 1);
		
	}
	marking_Point(m_mask_center_X, m_mask_center_y, mat_data, pow(2, 16) - 1);
	imshow("tipPoint", mat_data);
}


void CFingerDetect::enlarge_finger(vector <largeFinger> & vLargeFinger)
{//FingerDetectŬ�������� ������ �ճ� �����͵��� 500�Ÿ��� �°� Ȯ�����
//�߽��� ����� ���߰� CHandInitializationŬ������ �ִ´�.
	cout << "enlarge_finger()" << endl;
	vLargeFinger.resize(m_vfinger.size()); 
	//���� ���̴� 500
	double stdDepth = 500;
	double mod_constant = m_mask_center_Z - stdDepth; //��� Point�� ���̰��� �̸�ŭ ���� 500�Ÿ��� �����.
	for (int i = 0; i < m_vfinger.size(); i++)
	{
		vLargeFinger[i].fingerTip.z = m_vfinger[i].tip_depth - mod_constant;
		double scale = m_vfinger[i].tip_depth / vLargeFinger[i].fingerTip.z;
		vLargeFinger[i].fingerTip.x = (m_vfinger[i].fingerTip.x - m_mask_center_X) * scale + INPUT_WIDTH / 2;
		vLargeFinger[i].fingerTip.y = (m_vfinger[i].fingerTip.y - m_mask_center_y) * scale + INPUT_HEIGHT / 2;
		

		vLargeFinger[i].fingerMcp.x = (m_vfinger[i].fingerMcp.x - m_mask_center_X) * scale + INPUT_WIDTH / 2;
		vLargeFinger[i].fingerMcp.y = (m_vfinger[i].fingerMcp.y - m_mask_center_y) * scale + INPUT_HEIGHT / 2;
		vLargeFinger[i].fingerMcp.z = m_vfinger[i].mcp_depth - mod_constant;
	}
}