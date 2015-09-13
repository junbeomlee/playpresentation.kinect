// HandDetectionDlg.cpp : 구현 파일입니다.
//


#include "stdafx.h"
#include "FingerRecog.h"
#include "HandDetectionDlg.h"
#include "afxdialogex.h"
#include "ColorCV.h"
#include "DepthCV.h"
#include "BodyCV.h"
#include "BodyIndexCV.h"
#include "HandDetect.h"

//jointData* g_jmodData; //g_jData를 moveCenter와 scaling작업을 거친후 픽셀좌표로 저장을한다.
//sphereData* g_smodData;

IMPLEMENT_DYNAMIC(CHandDetectionDlg, CDialogEx)

CHandDetectionDlg::CHandDetectionDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CHandDetectionDlg::IDD, pParent),
	m_pKinectSensor(NULL)
{
	Mat depthMat;
	Mat handMat;
	IBody** body;
	BOOLEAN temp;
	HRESULT hr;
	Joint joints[JointType_Count];
	Point3d handTip3D, wrist3D, elbow3D, hand3D, shou3D;
	Point3d direction;
	Point handTip, wrist, elbow, hand, shou;

	//Point hand;
	CHandDetect handDetect;
	EnableAutomation();
	initKinectSensor();
	CColorCV colorCV(m_pKinectSensor);
	CDepthCV depthCV(m_pKinectSensor);
	CBodyIndexCV bodyIndexCV(m_pKinectSensor);
	CBodyCV bodyCV(m_pKinectSensor);
	
	while (1) {
		colorCV.UpdateKinectColorFrame();
		depthCV.UpdateKinectDepthFrame();
		bodyIndexCV.UpdateKinectBodyIndexFrame(depthCV.GetDepthMat());
		bodyCV.UpdateKinectBodyFrame(bodyIndexCV.GetBodyIndexMat());
		body = bodyCV.GetBodies();
		depthMat = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);
		depthCV.GetDepthMat().convertTo(depthMat, CV_16UC1, (pow(2, 16) - 1) / 8000, 0.0);
		
		for (int i = 0; i < BODY_COUNT; i++) {
			if (body[i]) {
				hr = body[i]->get_IsTracked(&temp);
				if (SUCCEEDED(hr) && temp) {
					body[i]->GetJoints(_countof(joints), joints);
					hand = bodyCV.BodyToScreen(joints[JointType_HandRight].Position, 512, 424);
					handTip = bodyCV.BodyToScreen(joints[JointType_HandTipRight].Position, 512, 424);
					wrist = bodyCV.BodyToScreen(joints[JointType_WristRight].Position, 512, 424);
					elbow = bodyCV.BodyToScreen(joints[JointType_ElbowRight].Position, 512, 424);
					shou = bodyCV.BodyToScreen(joints[JointType_ShoulderRight].Position, 512, 424);
					handTip3D.x = handTip.x;
					handTip3D.y = handTip.y;
					handTip3D.z = joints[JointType_HandTipRight].Position.Z * 1000;
					wrist3D.x = wrist.x;
					wrist3D.y = wrist.y;
					wrist3D.z = joints[JointType_WristRight].Position.Z * 1000;
					elbow3D.x = elbow.x;
					elbow3D.y = elbow.y;
					elbow3D.z = joints[JointType_ElbowRight].Position.Z * 1000;
					hand3D.x = hand.x;
					hand3D.y = hand.y;
					hand3D.z = joints[JointType_HandRight].Position.Z * 1000;
					shou3D.x = shou.x;
					shou3D.y = shou.y;
					shou3D.z = joints[JointType_ShoulderRight].Position.Z * 1000;

					direction = wrist3D - elbow3D;
					/*
					handDetect = CHandDetect(depthCV.GetDepthMat(), hand);
					handDetect.PreProcess(depthCV.GetDepthMat(), hand);
					handDetect.FindHand();
					handDetect.RemoveArm(direction, wrist, hand, elbow);

					circle(handDetect.GetDetectMat(), hand, 5, Scalar(0, 255, 0), 3);
					circle(handDetect.GetDetectMat(), wrist, 5, Scalar(0, 0, 255), 3);
					*/
				}
			}
		}
		
		imshow("color", colorCV.GetColorMat());
		imshow("Depth", depthMat);
		imshow("BodyIndex", bodyIndexCV.GetBodyIndexMat());
		imshow("Body", bodyCV.GetBodyMat());
		//imshow("Hand", handDetect.GetDetectMat());

		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}

	destroyAllWindows();

	/*delete g_jmodData;
	delete g_smodData;*/
}

CHandDetectionDlg::~CHandDetectionDlg()
{
}

HRESULT CHandDetectionDlg::initKinectSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{

		hr = m_pKinectSensor->Open();
	}

	return hr;
}

void CHandDetectionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CHandDetectionDlg, CDialogEx)
END_MESSAGE_MAP()


// CHandDetectionDlg 메시지 처리기입니다.
