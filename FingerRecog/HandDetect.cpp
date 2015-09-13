#include "stdafx.h"
#include "HandDetect.h"
#include <string.h>
#include <stdlib.h>
#include <vector>

using namespace std;

#pragma warning(disable:4996)
#pragma once

#define PI (3.141592653559)

CHandDetect::CHandDetect() :
m_handPoint(0, 0)
, m_handDepth(-1)
{
	m_detectMat = Mat::zeros(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
}

CHandDetect::CHandDetect(Mat depthMat, Point handPoint) :
m_handPoint(0, 0)
{
	m_detectMat.release();
	m_detectMat = Mat::zeros(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
	m_handPoint = handPoint;
	if (handPoint.x > 0 && handPoint.x < DEPTH_WIDTH && handPoint.y > 0 && handPoint.y < DEPTH_HEIGHT)
		m_handDepth = depthMat.at<UINT16>(handPoint);
}

CHandDetect::~CHandDetect()
{
	m_detectMat.release();
}

void CHandDetect::PreProcess(const Mat depthMat, Point handPoint)
{
	if (handPoint.x > 0 && handPoint.x < DEPTH_WIDTH && handPoint.y > 0 && handPoint.y < DEPTH_HEIGHT)
	{
		m_handPoint = handPoint;
		m_handDepth = depthMat.at<UINT16>(handPoint);
	}
	else
	{
		m_handDepth = 0;
		m_detectMat = Mat::zeros(m_detectMat.size(), m_detectMat.type());
		return;
	}

	if (m_handDepth == 0)
	{
		m_detectMat = Mat::zeros(m_detectMat.size(), m_detectMat.type());
		return;
	}

	UINT16 depth = 0;
	double a = m_handDepth - cDepthRange;
	double b = m_handDepth + cDepthRange;

	m_detectMat = Mat::zeros(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);

	const int limitArea = CONST_AREA / m_handDepth;

	for (int y = 0; y < depthMat.rows; y++){
		for (int x = 0; x < depthMat.cols; x++){
			depth = depthMat.at<UINT16>(y, x);
			if (depth > m_handDepth - cDepthRange && depth < m_handDepth + cDepthRange) {
				//&& sqrt(pow(x-m_handPoint.x, 2.0) + pow(y-m_handPoint.y, 2.0)) < limitArea) {
				m_detectMat.at<UCHAR>(y, x) = pow(2, 8) - 1;
			}
		}
	}
}

void CHandDetect::SetMember(Mat depthMat, Point handPoint)
{
	depthMat.copyTo(m_detectMat);
	m_handPoint = handPoint;
}

void CHandDetect::FindHand()
{
	if (m_handDepth == 0)
	{
		return;
	}
	double limitLength = 3.5 * (double)(CONST_AREA) / m_handDepth;
	Point explPoint;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat contourMat = Mat(m_detectMat.size(), m_detectMat.type());
	if (m_handDepth == 0) {
		m_detectMat = Mat::zeros(m_detectMat.size(), m_detectMat.type());
		return;
	}

	if (m_handDepth == 0)
	{
		return;
	}

	findContours(m_detectMat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	m_detectMat = Mat::zeros(m_detectMat.size(), m_detectMat.type());
	if (!contours.empty() && !hierarchy.empty()) {
		for (int i = 0; i >= 0; i = hierarchy[i][0]) {
			contourMat = Mat::zeros(m_detectMat.size(), m_detectMat.type());
			drawContours(contourMat, contours, i, Scalar(pow(2, 8) - 1, pow(2, 8) - 1, pow(2, 8) - 1), CV_FILLED, 8, hierarchy);
			if (contourMat.at<UCHAR>(m_handPoint) > 0) {
				m_detectMat = Mat::zeros(m_detectMat.size(), m_detectMat.type());

				
				for (int i = 0; i < 2 * limitLength; i++)
				{
					for (int j = 0; j < 2 * limitLength; j++)
					{
						explPoint = Point(m_handPoint.x - limitLength + i, m_handPoint.y - limitLength + j);

						if (explPoint.x >= 0 && explPoint.x < contourMat.cols && explPoint.y >= 0 && explPoint.y < contourMat.rows
							&& sqrt(pow(explPoint.x - m_handPoint.x, 2.0) + pow(explPoint.y - m_handPoint.y, 2.0)) <= limitLength
							&& contourMat.at<UCHAR>(explPoint) > 0)
						{
							m_detectMat.at<UCHAR>(explPoint) = 255;
						}
					}
				}

				imshow("DetectHand", m_detectMat);

				contourMat.release();

				return;
			}
		}
	}

	contourMat.release();
}

void CHandDetect::RemoveArm(Point3d direction, Point wristPoint, Point handPoint, Point elbowPoint)
{
	Mat distMat = Mat::zeros(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
	Point handCenter;
	double radius;
	int maxIdx[2];
	Point wrist;
	Mat handMat = Mat::zeros(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
	float* circleData = (float*)calloc(360, sizeof(float));

	distanceTransform(m_detectMat, distMat, CV_DIST_L2, 5);
	minMaxIdx(distMat, NULL, &radius, NULL, maxIdx, m_detectMat);
	handCenter.x = maxIdx[1];
	handCenter.y = maxIdx[0];
	normalize(distMat, distMat, 0, 1., NORM_MINMAX);
	FindWrist(distMat, handCenter, elbowPoint, radius, wrist);
	m_handPoint = Point(handCenter);
	m_wristPoint = Point(wrist);

	imshow("dist", distMat);

	if (DetectHand(distMat, handCenter, wrist, handMat))
	{
		handMat.copyTo(m_detectMat);
		imshow("DetecHand", handMat);
	}
	
	//GetCircle(distMat, handCenter, radius, circleData, handPoint, elbowPoint);
	//circle(*distMat, handCenter, 2, Scalar(255, 255, 255), -1);
	//circle(*distMat, handCenter, (int)(radius + 0.5), Scalar(255, 255, 255), 2);
	
	/*if (direction.y == 0.0)
	{
		return;
	}
	double a = -direction.x / direction.y;
	Point avgPoint(0, 0);
	int count = 0;
	for (int i = 0; i < m_detectMat.rows; i++)
	{
		for (int j = 0; j < m_detectMat.cols; j++)
		{
			if (m_detectMat.at<UCHAR>(i, j) > 0)
			{
				avgPoint.x += j;
				avgPoint.y += i;
				count++;
			}
		}
	}

	if (count == 0)
	{
		return;
	}

	avgPoint.x /= count;
	avgPoint.y /= count;

	double b = wristPoint.y - a*wristPoint.x;

	for (int i = 0; i < m_detectMat.rows; i++)
	{
		for (int j = 0; j < m_detectMat.cols; j++)
		{
			if (m_detectMat.at<UCHAR>(i, j) > 0)
			{
				if (LineFunction(a, b, Point(j, i))*direction.y < 0)
				{
					m_detectMat.at<UCHAR>(i, j) = 0;
				}
			}
		}
	}
	*/
	//imshow("Remove", *distMat);
}

double CHandDetect::LineFunction(double a, double b, Point point)
{
	return point.y - (a*point.x + b);
}

int CHandDetect::GetCircle(Mat srcMat, Point center, double radius, float* dstData, Point hand, Point elbow)
{
	if (srcMat.data == NULL && radius <= 0 && dstData == NULL)
	{
		return 0;
	}

	Mat viewData = Mat(100, 360, CV_32FC1);
	Point dstPoint[360];
	float data;
	float thresh = 0.2f;

	for (int i = 0; i < 360; i++)
	{
		dstPoint[i].x = (int)(center.x + radius * 1.7 * sin(PI * (i / 180.0)) + 0.5);
		dstPoint[i].y = (int)(center.y + radius * 1.7 * cos(PI * (i / 180.0)) + 0.5);

		if (dstPoint[i].x < 0 || dstPoint[i].y < 0 || dstPoint[i].x >= srcMat.cols || dstPoint[i].y >= srcMat.rows)
		{
			dstData[i] = -1;
		}
		else
		{
			dstData[i] = (srcMat.at<float>(dstPoint[i]));
		}

		data = dstData[i];

		for (int j = 0; j < 100; j++)
		{
			viewData.at<float>(j, i) = data;
		}
	}

	int i = 0, j = 0, maxIndex, count = 0;
	float max;
	float nMax;
	int nMaxIndex;
	bool flag = false;
	vector<int> vecMaxIndex;
	char x[10] = "", y[10] = "";
	char str[50] = "";

	if (dstData[0] > thresh && dstData[359] > thresh)
	{
		for (i = 0; dstData[(i + j) % 360] > thresh; i++)
		{

		}
	}

	for (; i < 360; i++)
	{
		if (dstData[i] > thresh)
		{
			maxIndex = i;
			max = dstData[i];
			for (j = 1; dstData[(i + j) % 360] > thresh; j++)
			{
				if (dstData[(i + j) % 360] > max)
				{
					maxIndex = (i + j) % 360;
					max = dstData[(i + j) % 360];
				}
				//dstData[(i + j) % 360] = 0.f;
			}

			i += j;

			if (j > 8 && j < 60)
			{
				count++;
				vecMaxIndex.push_back(maxIndex);
				nMax = max;
				nMaxIndex = maxIndex;
			}

			j = 0;
		}
	}


	/*if (count == 1)
	{
		circle(srcMat, dstPoint[nMaxIndex], 5, Scalar(0, 0, 0), 2);	
	}
	else
	{
		for (int i = 0; i < count; i++)
		{
			itoa(vecMaxIndex[i], str, 10);
			putText(srcMat, str, Point(50, 100 + i * 50), FONT_HERSHEY_SIMPLEX, 1, 2);
			circle(srcMat, dstPoint[vecMaxIndex[i]], 5, Scalar(1.0f, 1.f, 1.f), 2);
		}
	}*/

	Mat threshMat = Mat(viewData.rows, viewData.cols, CV_32FC1);
	threshold(viewData, threshMat, 0.0001, 1.f, CV_THRESH_BINARY);

	double gradi;
	int minLength = 0x0fffffff;
	int minIndex;
	int length;
	Point tempPoint;
	vector<Point> vecWrist;
	
	if (center.x == elbow.x)
	{
		if (center.y - elbow.y > 0)
			gradi = 9999.0;
		else
			gradi = -9999.0;
	}
	else
	{
		gradi = (center.y - elbow.y) / (center.x - elbow.x);
	}

	/*
	if (pow(gradi, 2.0) <= 1.0)
	{
		for (int i = 0; i < vecMaxIndex.size(); i++)
		{
			length = abs(center.y + gradi * (center.x - dstPoint[vecMaxIndex[i]].x) - dstPoint[vecMaxIndex[i]].y);
			if (center.x <= elbow.x)
			{
				if (dstPoint[vecMaxIndex[i]].x < center.x)
				{
					length = sqrt(pow(length, 2.0) + pow(center.x - dstPoint[vecMaxIndex[i]].x, 2.0));
				}
				else if (dstPoint[vecMaxIndex[i]].x > elbow.x)
				{
					length = sqrt(pow(length, 2.0) + pow(elbow.x - dstPoint[vecMaxIndex[i]].x, 2.0));
				}
			}
			else
			{
				if (dstPoint[vecMaxIndex[i]].x < elbow.x)
				{
					length = sqrt(pow(length, 2.0) + pow(elbow.x - dstPoint[vecMaxIndex[i]].x, 2.0));
				}
				else if (dstPoint[vecMaxIndex[i]].x > center.x)
				{
					length = sqrt(pow(length, 2.0) + pow(center.x - dstPoint[vecMaxIndex[i]].x, 2.0));
				}
			}

			if (length < minLength)
			{
				minLength = length;
				minIndex = i;
			}
		}
	}
	else
	{
		for (int i = 0; i < vecMaxIndex.size(); i++)
		{
			if (gradi != 0.0)
				length = abs(center.x + 1 / gradi * (center.y - dstPoint[vecMaxIndex[i]].y) - dstPoint[vecMaxIndex[i]].x);
			else
				length = abs(center.x - dstPoint[vecMaxIndex[i]].x);

			if (center.y <= elbow.y)
			{
				if (dstPoint[vecMaxIndex[i]].y < center.y)
				{
					length = sqrt(pow(length, 2.0) + pow(center.y - dstPoint[vecMaxIndex[i]].y, 2.0));
				}
				else if (dstPoint[vecMaxIndex[i]].y > elbow.y)
				{
					length = sqrt(pow(length, 2.0) + pow(elbow.y - dstPoint[vecMaxIndex[i]].y, 2.0));
				}
			}
			else
			{
				if (dstPoint[vecMaxIndex[i]].y < elbow.y)
				{
					length = sqrt(pow(length, 2.0) + pow(elbow.y - dstPoint[vecMaxIndex[i]].y, 2.0));
				}
				else if (dstPoint[vecMaxIndex[i]].y > center.y)
				{
					length = sqrt(pow(length, 2.0) + pow(center.y - dstPoint[vecMaxIndex[i]].y, 2.0));
				}
			}

			if (length < minLength)
			{
				minLength = length;
				minIndex = i;
			}
		}
	}
	if (vecMaxIndex.size() != 0)
		circle(srcMat, dstPoint[vecMaxIndex[minIndex]], 5, Scalar(0.0f, 0.f, 0.f), 2);
	*/

	
	if (pow(gradi, 2.0) < 1.0)
	{
		if (gradi == 0.0)
		{
			gradi = 00000.1;
		}

		if (center.x > elbow.x)
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].x <= gradi * (center.y - dstPoint[vecMaxIndex[i]].y) + center.x
					&& dstPoint[vecMaxIndex[i]].x >= gradi * (elbow.y - dstPoint[vecMaxIndex[i]].y) + elbow.x)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
		else
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].x >= gradi * (center.y - dstPoint[vecMaxIndex[i]].y) + center.x
					&& dstPoint[vecMaxIndex[i]].x <= gradi * (elbow.y - dstPoint[vecMaxIndex[i]].y) + elbow.x)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
	}
	else
	{
		if (center.y > elbow.y)
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].y <= 1 / gradi * (center.x - dstPoint[vecMaxIndex[i]].x) + center.y
					&& dstPoint[vecMaxIndex[i]].y >= 1 / gradi * (elbow.x - dstPoint[vecMaxIndex[i]].x) + elbow.y)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
		else
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].y >= 1 / gradi * (center.x - dstPoint[vecMaxIndex[i]].x) + center.y
					&& dstPoint[vecMaxIndex[i]].y <= 1 / gradi * (elbow.x - dstPoint[vecMaxIndex[i]].x) + elbow.y)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
	}
	
	itoa(vecWrist.size(), str, 10);

	/*putText(srcMat, str, Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, 2);

	for (int i = 0; i < count; i++)
	{
		itoa(vecMaxIndex[i], str, 10);
		putText(srcMat, str, Point(50, 100 + i * 50), FONT_HERSHEY_SIMPLEX, 1, 2);
		circle(srcMat, dstPoint[vecMaxIndex[i]], 5, Scalar(1.0f, 1.f, 1.f), 2);
	}*/

	float maxData = -99999.0, curData;
	maxIndex = -1;


	for (int i = 0; i < vecWrist.size(); i++)
	{
		curData = srcMat.at<float>(vecWrist[i]);
		if (curData > maxData)
		{
			maxData = curData;
			maxIndex = i;
		}
		circle(srcMat, vecWrist[i], 5, Scalar(0.5f, 0.f, 0.f), 2);
	}

	if (vecWrist.size() != 0)
	{
		circle(srcMat, vecWrist[maxIndex], 5, Scalar(0.f, 0.f, 0.f), 2);
	}

	circle(srcMat, center, 5, Scalar(0.f, 0.f, 0.f), 2);
	circle(srcMat, elbow, 5, Scalar(1.f, 0.f, 0.f), 2);
	
	////imshow("circle", viewData);
	////imshow("threshold", threshMat);
	////imshow("distance", srcMat);


	return 1;
}

int CHandDetect::FindWrist(Mat srcMat, Point hand, Point elbow, double radius, Point& wrist)
{
	if (srcMat.data == NULL && radius <= 0)
	{
		return 0;
	}

	Mat viewData = Mat(100, 360, CV_32FC1);
	Point dstPoint[360];
	float data;
	float dstData[360];

	for (int i = 0; i < 360; i++)
	{
		dstPoint[i].x = (int)(hand.x + radius * 1.7 * sin(PI * (i / 180.0)) + 0.5);
		dstPoint[i].y = (int)(hand.y + radius * 1.7 * cos(PI * (i / 180.0)) + 0.5);

		if (dstPoint[i].x < 0 || dstPoint[i].y < 0 || dstPoint[i].x >= srcMat.cols || dstPoint[i].y >= srcMat.rows)
		{
			dstData[i] = -1;
		}
		else
		{
			dstData[i] = (srcMat.at<float>(dstPoint[i]));
		}

		data = dstData[i];

		for (int j = 0; j < 100; j++)
		{
			viewData.at<float>(j, i) = data;
		}
	}

	int i = 0, j = 0, maxIndex, count = 0;
	float max;
	float nMax;
	int nMaxIndex;
	bool flag = false;
	vector<int> vecMaxIndex;
	char x[10] = "", y[10] = "";
	char str[50] = "";

	if (dstData[0] > 0.0f && dstData[359] > 0.0f)
	{
		for (i = 0; dstData[(i + j) % 360] > 0.0f; i++)
		{

		}
	}

	for (; i < 360; i++)
	{
		if (dstData[i] > 0.0f)
		{
			maxIndex = i;
			max = dstData[i];
			for (j = 1; dstData[(i + j) % 360] > 0.0f; j++)
			{
				if (dstData[(i + j) % 360] > max)
				{
					maxIndex = (i + j) % 360;
					max = dstData[(i + j) % 360];
				}
				//dstData[(i + j) % 360] = 0.f;
			}

			i += j;

			if (j > 8 && j < 60)
			{
				count++;
				vecMaxIndex.push_back(maxIndex);
				nMax = max;
				nMaxIndex = maxIndex;
			}

			j = 0;
		}
	}

	Mat threshMat = Mat(viewData.rows, viewData.cols, CV_32FC1);
	threshold(viewData, threshMat, 0.0001, 1.f, CV_THRESH_BINARY);

	double gradi;
	int minLength = 0x0fffffff;
	int minIndex;
	int length;
	Point tempPoint;
	vector<Point> vecWrist;

	if (hand.x == elbow.x)
	{
		if (hand.y - elbow.y > 0)
			gradi = 9999.0;
		else
			gradi = -9999.0;
	}
	else
	{
		gradi = (hand.y - elbow.y) / (hand.x - elbow.x);
	}

	if (pow(gradi, 2.0) < 1.0)
	{
		if (gradi == 0.0)
		{
			gradi = 00000.1;
		}

		if (hand.x > elbow.x)
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].x <= gradi * (hand.y - dstPoint[vecMaxIndex[i]].y) + hand.x
					&& dstPoint[vecMaxIndex[i]].x >= gradi * (elbow.y - dstPoint[vecMaxIndex[i]].y) + elbow.x)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
		else
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].x >= gradi * (hand.y - dstPoint[vecMaxIndex[i]].y) + hand.x
					&& dstPoint[vecMaxIndex[i]].x <= gradi * (elbow.y - dstPoint[vecMaxIndex[i]].y) + elbow.x)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
	}
	else
	{
		if (hand.y > elbow.y)
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].y <= 1 / gradi * (hand.x - dstPoint[vecMaxIndex[i]].x) + hand.y
					&& dstPoint[vecMaxIndex[i]].y >= 1 / gradi * (elbow.x - dstPoint[vecMaxIndex[i]].x) + elbow.y)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
		else
		{
			for (int i = 0; i < vecMaxIndex.size(); i++)
			{
				if (dstPoint[vecMaxIndex[i]].y >= 1 / gradi * (hand.x - dstPoint[vecMaxIndex[i]].x) + hand.y
					&& dstPoint[vecMaxIndex[i]].y <= 1 / gradi * (elbow.x - dstPoint[vecMaxIndex[i]].x) + elbow.y)
				{
					vecWrist.push_back(dstPoint[vecMaxIndex[i]]);
				}
			}
		}
	}

	itoa(vecWrist.size(), str, 10);

	/*putText(srcMat, str, Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, 2);

	for (int i = 0; i < count; i++)
	{
		itoa(vecMaxIndex[i], str, 10);
		putText(srcMat, str, Point(50, 100 + i * 50), FONT_HERSHEY_SIMPLEX, 1, 2);
		//circle(srcMat, dstPoint[vecMaxIndex[i]], 5, Scalar(1.0f, 1.f, 1.f), 2);
	}*/

	float maxData = -99999.0, curData;
	maxIndex = -1;


	for (int i = 0; i < vecWrist.size(); i++)
	{
		curData = srcMat.at<float>(vecWrist[i]);
		if (curData > maxData)
		{
			maxData = curData;
			maxIndex = i;
		}
		//circle(srcMat, vecWrist[i], 5, Scalar(0.5f, 0.f, 0.f), 2);
	}

	if (vecWrist.size() != 0)
	{
		//circle(srcMat, vecWrist[maxIndex], 5, Scalar(0.f, 0.f, 0.f), 2);
		wrist = Point(vecWrist[maxIndex]);
	}

	//circle(srcMat, hand, 5, Scalar(0.f, 0.f, 0.f), 2);
	//circle(srcMat, elbow, 5, Scalar(1.f, 0.f, 0.f), 2);

	////imshow("circle", viewData);
	////imshow("threshold", threshMat);
	////imshow("distance", srcMat);

	return 1;
}

int CHandDetect::DetectHand(Mat distMat, Point hand, Point wrist, Mat& handMat)
{
	Point rPoint, lPoint;
	float minDist = 999999.0;
	Point minPoint;
	int minIndex;
	float dist;
	Point point;
	int index;
	distMat.copyTo(handMat);
	//circle(handMat, wrist, 5, Scalar(1.f, 1.f, 1.f), 2);
	if (IsInRange(wrist))
		handMat.at<float>(wrist) = 0.f;

	point = Point(wrist.x - 1, wrist.y - 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 0;
		}
	}
	
	
	point = Point(wrist.x, wrist.y - 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 1;
		}
	}

	point = Point(wrist.x + 1, wrist.y - 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 2;
		}
	}

	point = Point(wrist.x + 1, wrist.y);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 3;
		}
	}

	point = Point(wrist.x + 1, wrist.y + 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 4;
		}
	}

	point = Point(wrist.x, wrist.y + 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 5;
		}
	}

	point = Point(wrist.x - 1, wrist.y + 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 6;
		}
	}

	point = Point(wrist.x - 1, wrist.y);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
			minIndex = 7;
		}
	}

	lPoint = Point(minPoint);

	index = (minIndex + 4) % 8;
	minDist = 99999.0;

	for (int i = 0; i < 3; i++)
	{
		IsMinDist(distMat, wrist, (index - 1 + i) % 8, minDist, minPoint);
	}

	rPoint = Point(minPoint);

	/*
	char str[100] = "";
	char temp[100] = "";

	itoa(rPoint.x, temp, 10);
	strcpy(str, temp);
	strcat(str, ", ");
	itoa(rPoint.y, temp, 10);
	strcat(str, temp);
	putText(handMat, str, Point(50, 100), FONT_HERSHEY_SIMPLEX, 1, 2);

	itoa(lPoint.x, temp, 10);
	strcpy(str, temp);
	strcat(str, ", ");
	itoa(lPoint.y, temp, 10);
	strcat(str, temp);
	putText(handMat, str, Point(50, 150), FONT_HERSHEY_SIMPLEX, 1, 2);

	itoa(abs(rPoint.x - lPoint.x) + abs(rPoint.y - lPoint.y), str, 10);
	putText(handMat, str, Point(50, 200), FONT_HERSHEY_SIMPLEX, 1, 2);
	*/

	if (IsInRange(wrist))
		handMat.at<float>(wrist) = 0.f;

	for (int index = 0; index < 8; index++)
	{
		switch (index)
		{
		case 0:
			point = Point(wrist.x - 1, wrist.y - 1);
			break;

		case 1:
			point = Point(wrist.x, wrist.y - 1);
			break;

		case 2:
			point = Point(wrist.x + 1, wrist.y - 1);
			break;

		case 3:
			point = Point(wrist.x + 1, wrist.y);
			break;

		case 4:
			point = Point(wrist.x + 1, wrist.y + 1);
			break;

		case 5:
			point = Point(wrist.x, wrist.y + 1);
			break;

		case 6:
			point = Point(wrist.x - 1, wrist.y + 1);
			break;

		case 7:
			point = Point(wrist.x - 1, wrist.y);
			break;

		default:
			return 0;
			break;
		}

		if (IsInRange(point))
			handMat.at<float>(point) = 0.f;
	}

	do
	{
		if (IsInRange(rPoint))
		{
			handMat.at<float>(rPoint) = 0.f;

			for (int index = 0; index < 8; index++)
			{
				switch (index)
				{
				case 0:
					point = Point(rPoint.x - 1, rPoint.y - 1);
					break;

				case 1:
					point = Point(rPoint.x, rPoint.y - 1);
					break;

				case 2:
					point = Point(rPoint.x + 1, rPoint.y - 1);
					break;

				case 3:
					point = Point(rPoint.x + 1, rPoint.y);
					break;

				case 4:
					point = Point(rPoint.x + 1, rPoint.y + 1);
					break;

				case 5:
					point = Point(rPoint.x, rPoint.y + 1);
					break;

				case 6:
					point = Point(rPoint.x - 1, rPoint.y + 1);
					break;

				case 7:
					point = Point(rPoint.x - 1, rPoint.y);
					break;

				default:
					return 0;
					break;
				}

				if (IsInRange(point))
					handMat.at<float>(point) = 0.f;
			}
			point = Point(rPoint);
			dist = distMat.at<float>(point);
			FindNextPixel(distMat, point, rPoint);
		}
		else
		{
			break;
		}
		
	} while (dist > 0.0f);

	do
	{
		if (IsInRange(lPoint))
		{
			handMat.at<float>(lPoint) = 0.f;

			for (int index = 0; index < 8; index++)
			{
				switch (index)
				{
				case 0:
					point = Point(lPoint.x - 1, lPoint.y - 1);
					break;

				case 1:
					point = Point(lPoint.x, lPoint.y - 1);
					break;

				case 2:
					point = Point(lPoint.x + 1, lPoint.y - 1);
					break;

				case 3:
					point = Point(lPoint.x + 1, lPoint.y);
					break;

				case 4:
					point = Point(lPoint.x + 1, lPoint.y + 1);
					break;

				case 5:
					point = Point(lPoint.x, lPoint.y + 1);
					break;

				case 6:
					point = Point(lPoint.x - 1, lPoint.y + 1);
					break;

				case 7:
					point = Point(lPoint.x - 1, lPoint.y);
					break;

				default:
					return 0;
					break;
				}
				
				if (IsInRange(point))
				{
					handMat.at<float>(point) = 0.f;
				}
			}

			point = Point(lPoint);
			dist = distMat.at<float>(point);
			FindNextPixel(distMat, point, lPoint);
		}
		else
		{
			break;
		}
	} while (dist > 0.0f);

	Mat threshMat = Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
	threshold(handMat, threshMat, 0.00001, 255, CV_THRESH_BINARY);
	threshMat.convertTo(threshMat, CV_8UC1);
	////imshow("Thres", threshMat);
	Point explPoint;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat contourMat = Mat(threshMat.size(), threshMat.type());

	findContours(threshMat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	
	threshMat = Mat::zeros(threshMat.size(), threshMat.type());
	if (!contours.empty() && !hierarchy.empty()) {
		for (int i = 0; i >= 0; i = hierarchy[i][0]) {
			contourMat = Mat::zeros(threshMat.size(), threshMat.type());
			drawContours(contourMat, contours, i, Scalar(pow(2, 8) - 1, pow(2, 8) - 1, pow(2, 8) - 1), CV_FILLED, 8, hierarchy);
			if (IsInRange(hand) && contourMat.at<UCHAR>(hand) > 0) {
				handMat = Mat::zeros(handMat.size(), handMat.type());
				threshold(contourMat, threshMat, 1.0, 255, CV_THRESH_BINARY);
				threshMat.convertTo(handMat, CV_32FC1);

				contourMat.release();

				return 1;
			}
		}
	}

	contourMat.release();
	
	return 1;
}

int CHandDetect::IsMinDist(Mat distMat, Point curPoint, int index, float& minDist, Point& minPoint)
{
	Point point;
	float dist;

	switch (index)
	{
	case 0:
		point = Point(curPoint.x - 1, curPoint.y - 1);
		break;

	case 1:
		point = Point(curPoint.x, curPoint.y - 1);
		break;

	case 2:
		point = Point(curPoint.x + 1, curPoint.y - 1);
		break;

	case 3:
		point = Point(curPoint.x + 1, curPoint.y);
		break;

	case 4:
		point = Point(curPoint.x + 1, curPoint.y + 1);
		break;

	case 5:
		point = Point(curPoint.x, curPoint.y + 1);
		break;

	case 6:
		point = Point(curPoint.x - 1, curPoint.y + 1);
		break;

	case 7:
		point = Point(curPoint.x - 1, curPoint.y);
		break;

	default:
		return 0;
		break;
	}
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (IsInRange(point) && dist < minDist)
		{
			minDist = dist;
			minPoint = Point(point);
		}

		return 1;
	}
	else
	{
		return 0;
	}
}

int CHandDetect::FindNextPixel(Mat distMat, Point curPoint, Point& nextPoint)
{
	float minDist = 999999.0;
	float dist;
	Point point;

	point = Point(curPoint.x - 1, curPoint.y - 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	point = Point(curPoint.x, curPoint.y - 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	point = Point(curPoint.x + 1, curPoint.y - 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	point = Point(curPoint.x + 1, curPoint.y);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	point = Point(curPoint.x + 1, curPoint.y + 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	point = Point(curPoint.x, curPoint.y + 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	point = Point(curPoint.x - 1, curPoint.y + 1);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	point = Point(curPoint.x - 1, curPoint.y);
	if (IsInRange(point))
	{
		dist = distMat.at<float>(point);
		if (dist < minDist)
		{
			minDist = dist;
			nextPoint = Point(point);
		}
	}

	return 1;
}


int CHandDetect::IsInRange(Point point)
{
	if (point.x < 0)
	{
		return 0;
	}

	if (point.x >= DEPTH_WIDTH)
	{
		return 0;
	}

	if (point.y < 0)
	{
		return 0;
	}

	if (point.y >= DEPTH_HEIGHT)
	{
		return 0;
	}

	return 1;
}
