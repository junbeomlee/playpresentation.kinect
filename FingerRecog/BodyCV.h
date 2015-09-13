#pragma once

#include "stdafx.h"

class CBodyCV {
	static const int cBodyWidth = 512;
	static const int cBodyHeight = 424;

private:
	IKinectSensor*			m_pKinectSensor;
	IBodyFrameReader*		m_pBodyFrameReader;
	ICoordinateMapper*		m_pCoordinateMapper;
	Mat						m_bodyMat;
	BOOLEAN					m_bTracked;
	Joint					m_aJoints[JointType_Count];
	Point					m_aJointPoints[JointType_Count];
	HandState				m_leftHandState = HandState_Unknown;
	HandState				m_rightHandState = HandState_Unknown;
	IBody* m_pBody;
	IBody* m_ppBodies[BODY_COUNT];
	BOOLEAN					m_bHandTracked;

public:
	CBodyCV(IKinectSensor* pKinectSensor);
	CBodyCV();
	~CBodyCV();

	HRESULT					initBodyReader();
	void					UpdateKinectBodyFrame(Mat pBodyIndexMat);
	void					ProcessBody(IBodyFrame* pBodyFrame);
	Point					BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);
	void                    DrawBody(const Joint* pJoints, const Point* pJointPoints);
	void                    DrawHand(HandState handState, const Point& handPosition);
	void                    DrawBone(const Joint* pJoints, const Point* pJointPoints, JointType joint0, JointType joint1);

	Mat						GetBodyMat() { return m_bodyMat; };
	BOOLEAN					GetIsTrack() { return m_bTracked; };
	Joint*					GetJoint()   { return m_aJoints; };
	Point*					GetJointPoint() { return m_aJointPoints; };
	HandState				GetLeftHandState() { return m_leftHandState; };
	HandState				GetRightHandState() { return m_rightHandState; };
	IBody*					GetBody()			{ return m_pBody; };
	IBody**					GetBodies()			{ return m_ppBodies; };
	BOOLEAN					GetHandTracked()	{ return m_bHandTracked; };
};

