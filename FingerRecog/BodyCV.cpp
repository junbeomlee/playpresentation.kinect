#pragma once

#include "stdafx.h"
#include "BodyCV.h"



CBodyCV::CBodyCV(IKinectSensor* pKinectSensor) :
m_pKinectSensor(NULL),
m_pBodyFrameReader(NULL),
m_bTracked(false),
m_leftHandState(HandState_Unknown),
m_rightHandState(HandState_Unknown)
{
	m_pKinectSensor = pKinectSensor;
	initBodyReader();
	m_bodyMat = Mat::zeros(cBodyHeight, cBodyWidth, CV_16UC1);
}

CBodyCV::CBodyCV() :
	m_pKinectSensor(NULL),
	m_pBodyFrameReader(NULL),
	m_bTracked(false),
	m_leftHandState(HandState_Unknown),
	m_rightHandState(HandState_Unknown)
{
	//initBodyReader();
	m_bodyMat = Mat::zeros(cBodyHeight, cBodyWidth, CV_16UC1);
}

CBodyCV::~CBodyCV()
{
	m_bodyMat.release();

	SafeRelease(m_pCoordinateMapper);
	SafeRelease(m_pBodyFrameReader);

	for (int i = 0; i < BODY_COUNT; i++)
	{
		//SafeRelease(m_ppBodies[i]);
	}
}

HRESULT CBodyCV::initBodyReader()
{
	HRESULT hr;
	IBodyFrameSource* pBodyFrameSource = NULL;

	hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);

	if (SUCCEEDED(hr))
	{
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
	}

	if (SUCCEEDED(hr))
	{
		hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	}

	SafeRelease(pBodyFrameSource);

	if (!m_pKinectSensor || SUCCEEDED(hr))
	{
		return E_FAIL;
	}

	return hr;
}

void CBodyCV::UpdateKinectBodyFrame(Mat bodyIndexMat)
{
	if (!m_pBodyFrameReader)
	{
		return;
	}

	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	bodyIndexMat.copyTo(m_bodyMat);

	if (SUCCEEDED(hr))
	{
		m_ppBodies[BODY_COUNT] = { 0 };
		hr = pBodyFrame->GetAndRefreshBodyData(_countof(m_ppBodies), m_ppBodies);
		if (SUCCEEDED(hr)) {
			ProcessBody(pBodyFrame);
		}
	}

	SafeRelease(pBodyFrame);
}


void CBodyCV::ProcessBody(IBodyFrame* pBodyFrame)
{
	HRESULT hr;

	if (m_pCoordinateMapper){
		for (int i = 0; i < BODY_COUNT; i++) {
			m_pBody = m_ppBodies[i];
			if (m_pBody) {
				hr = m_pBody->get_IsTracked(&m_bTracked);

				if (i == JointType_HandRight) {
					m_bHandTracked = m_bTracked;
				}

				if (SUCCEEDED(hr) && m_bTracked) {
					m_pBody->get_HandLeftState(&m_leftHandState);
					m_pBody->get_HandRightState(&m_rightHandState);
					hr = m_pBody->GetJoints(_countof(m_aJoints), m_aJoints);
					if (SUCCEEDED(hr))
					{
						for (int j = 0; j < _countof(m_aJointPoints); j++) {
							m_aJointPoints[j] = BodyToScreen(m_aJoints[j].Position, cBodyWidth, cBodyHeight);
						}

						DrawBody(m_aJoints, m_aJointPoints);

						DrawHand(m_leftHandState, m_aJointPoints[JointType_HandLeft]);
						DrawHand(m_rightHandState, m_aJointPoints[JointType_HandRight]);
					}
				}
			}
		}
	}
}


Point CBodyCV::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
	DepthSpacePoint depthPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / cBodyWidth;
	float screenPointY = static_cast<float>(depthPoint.Y * height) / cBodyHeight;

	if (screenPointX < 0) {
		screenPointX = 0;
	}
	if (screenPointX >= width) {
		screenPointX = width - 1;
	}
	if (screenPointY < 0) {
		screenPointY = 0;
	}
	if (screenPointY >= height) {
		screenPointY = height - 1;
	}

	return cv::Point(screenPointX, screenPointY);
}

void CBodyCV::DrawBody(const Joint* pJoints, const Point* pJointPoints)
{
	// Draw the bones

	// Torso
	DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
	DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
	DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
	DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
	DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
	DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
	DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
	DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
	DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
	DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
	DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

	// Draw the joints
	for (int i = 0; i < JointType_Count; ++i)
	{
		if (pJoints[i].TrackingState == TrackingState_Inferred)
		{
			circle(m_bodyMat, pJointPoints[i], 5, Scalar(pow(2, 16) - 1, 0, 0), -1, CV_AA);
			if (i == JointType_HandRight) {
				circle(m_bodyMat, pJointPoints[i], 5, Scalar(0, pow(2, 16) - 1, pow(2, 16) - 1), -1, CV_AA);
			}
		}
		else if (pJoints[i].TrackingState == TrackingState_Tracked)
		{
			circle(m_bodyMat, pJointPoints[i], 5, Scalar(0, pow(2, 16) - 1, 0), -1, CV_AA);
			if (i == JointType_HandRight) {
				circle(m_bodyMat, pJointPoints[i], 5, Scalar(0, pow(2, 16) - 1, pow(2, 16) - 1), -1, CV_AA);
			}
		}
	}
}

void CBodyCV::DrawBone(const Joint* pJoints, const Point* pJointPoints, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		cv::line(m_bodyMat, pJointPoints[joint0], pJointPoints[joint1], Scalar(pow(2, 16) - 1, 0, 0), 6);
	}
	else
	{
		cv::line(m_bodyMat, pJointPoints[joint0], pJointPoints[joint1], Scalar(0, pow(2, 16) - 1, 0), 1);
	}
}

void CBodyCV::DrawHand(HandState handState, const Point& handPosition)
{
	/*
	switch (handState)
	{
	case HandState_Closed:
	circle(*m_pBodyMat, handPosition, 30, Scalar(pow(2, 16) - 1, 0, 0), -1, CV_AA);
	break;

	case HandState_Open:
	circle(*m_pBodyMat, handPosition, 30, Scalar(0, 0, pow(2, 16) - 1), -1, CV_AA);
	break;

	case HandState_Lasso:
	circle(*m_pBodyMat, handPosition, 30, Scalar(0, pow(2, 16) - 1, 0), -1, CV_AA);
	break;
	}
	*/
}