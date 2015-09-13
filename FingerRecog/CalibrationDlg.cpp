// CalibrationDlg.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include <strsafe.h>
#include <math.h>
#include <limits>
#include <Wincodec.h>
#include <WinUser.h>
#include "resource.h"
#include "FingerRecog.h"
#include "CalibrationDlg.h"
#include "afxdialogex.h"
#include "ColorCV.h"
#include "DepthCV.h"
#include "BodyIndexCV.h"
#include <Windows.h>
#include <d2d1.h>
#pragma comment(lib,"d2d1")


// CCalibrationDlg 대화 상자입니다.

IMPLEMENT_DYNAMIC(CCalibrationDlg, CDialogEx)

CCalibrationDlg::CCalibrationDlg(CWnd* pParent /*=NULL*/)
: CDialogEx(CCalibrationDlg::IDD, pParent),
m_pDepthCoordinates(NULL),
m_pCoordinateMapper(NULL)
{
	EnableAutomation();
	initKinectSensor();
	CColorCV colorCV = CColorCV(m_pKinectSensor);
	CDepthCV depthCV = CDepthCV(m_pKinectSensor);
	CBodyIndexCV bodyIndexCV = CBodyIndexCV(m_pKinectSensor);
	Mat depthMat;
	Mat calibMat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	m_pDepthCoordinates = new DepthSpacePoint[COLOR_WIDTH * COLOR_HEIGHT];

	namedWindow("Color");
	namedWindow("Depth");
	namedWindow("BodyIndex");
	namedWindow("Calib");

	while (1) {
		colorCV.UpdateKinectColorFrame();
		depthCV.UpdateKinectDepthFrame();
		bodyIndexCV.UpdateKinectBodyIndexFrame(depthCV.GetDepthMat());

		depthMat = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);
		depthCV.GetDepthMat().convertTo(depthMat, CV_16UC1, (pow(2, 16) - 1) / 8000, 0.0);
		
		if (m_pDepthCoordinates && bodyIndexCV.GetBodyIndexBuffer())
		{
			HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(DEPTH_WIDTH * DEPTH_HEIGHT, (UINT16*)depthCV.GetDepthMat().data, COLOR_WIDTH * COLOR_HEIGHT, m_pDepthCoordinates);

			calibMat = Mat::zeros(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
			for (int colorIndex = 0; colorIndex < COLOR_WIDTH*COLOR_HEIGHT; colorIndex++)
			{
				// default setting source to copy from the background pixel
				DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

				int depthX = static_cast<int>(p.X + 0.5f);
				int depthY = static_cast<int>(p.Y + 0.5f);

				if ((depthX >= 0 && depthX < DEPTH_WIDTH) && (depthY >= 0 && depthY < DEPTH_HEIGHT))
				{
					BYTE player = bodyIndexCV.GetBodyIndexBuffer()[depthX + (depthY * DEPTH_WIDTH)];

					// if we're tracking a player for the current pixel, draw from the color camera
					if (player != 0xff)
					{
						// set source for copy to the color pixel
						calibMat.at<Vec4b>(colorIndex / COLOR_WIDTH, colorIndex % COLOR_WIDTH) = colorCV.GetColorMat().at<Vec4b>(colorIndex / COLOR_WIDTH, colorIndex % COLOR_WIDTH);
					}
				}
			}
		}
		
		//imshow("Color", *(colorCV.GetColorMat()));
		//imshow("Depth", depthMat);
		////imshow("BodyIndex", *(bodyIndexCV.GetBodyIndexMat()));
		//imshow("Calib", calibMat);

		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}

	destroyAllWindows();
}

CCalibrationDlg::~CCalibrationDlg()
{
	if (m_pDepthCoordinates)
	{
		delete[] m_pDepthCoordinates;
		m_pDepthCoordinates = NULL;
	}
}
/*
void CCalibrationDlg::run()
{
	EnableAutomation();
	initKinectSensor();
	CColorCV* colorCV = new CColorCV(m_pKinectSensor);
	CDepthCV* depthCV = new CDepthCV(m_pKinectSensor);
	CBodyIndexCV* bodyIndexCV = new CBodyIndexCV(m_pKinectSensor);
	Mat depthMat;
	Mat calibMat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	Mat background(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	RGBQUAD* backgroundRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];
	RGBQUAD* outputRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];
	background = Mat::zeros(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	m_pDepthCoordinates = new DepthSpacePoint[COLOR_WIDTH * COLOR_HEIGHT];

	const RGBQUAD c_black = { 0, 0, 0 };

	for (int i = 0; i < COLOR_WIDTH * COLOR_HEIGHT; i++)
	{
		backgroundRGBX[i] = c_black;
	}

	namedWindow("Color");
	namedWindow("Depth");
	namedWindow("BodyIndex");
	namedWindow("Calib");

	while (1) {
		colorCV->UpdateKinectColorFrame();
		depthCV->UpdateKinectDepthFrame();
		bodyIndexCV->UpdateKinectBodyIndexFrame(depthCV->GetDepthMat());

		depthMat = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1);
		depthCV->GetDepthMat()->convertTo(depthMat, CV_16UC1, (pow(2, 16) - 1) / 8000, 0.0);

		UINT16* test = (UINT16*)depthCV->GetDepthBuffer();
		if (depthCV->GetDepthBuffer() != NULL && m_pDepthCoordinates && bodyIndexCV->GetBodyIndexBuffer())
		{
			UINT16* test = (UINT16*)depthCV->GetDepthBuffer();
			HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(DEPTH_WIDTH * DEPTH_HEIGHT, (UINT16*)depthCV->GetDepthBuffer(), COLOR_WIDTH * COLOR_HEIGHT, m_pDepthCoordinates);

			for (int colorIndex = 0; colorIndex < COLOR_WIDTH*COLOR_HEIGHT; colorIndex++)
			{
				// default setting source to copy from the background pixel
				RGBQUAD* pSrc = backgroundRGBX + colorIndex;
				DepthSpacePoint p = m_pDepthCoordinates[colorIndex];
				calibMat = cv::Scalar(0, 0, 0, 0);
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					int depthX = static_cast<int>(p.X + 0.5f);
					int depthY = static_cast<int>(p.Y + 0.5f);

					if ((depthX >= 0 && depthX < DEPTH_WIDTH) && (depthY >= 0 && depthY < DEPTH_HEIGHT))
					{
						BYTE player = bodyIndexCV->GetBodyIndexBuffer()[depthX + (depthY * DEPTH_WIDTH)];

						// if we're tracking a player for the current pixel, draw from the color camera
						if (player != 0xff)
						{
							// set source for copy to the color pixel
							//calibMat.at<Vec4b>(colorIndex / COLOR_WIDTH, colorIndex / COLOR_HEIGHT) = colorCV->GetColorMat()->at<Vec4b>(colorIndex / COLOR_WIDTH, colorIndex % COLOR_HEIGHT);
							pSrc = colorCV->GetColorBuffer() + colorIndex;
						}
					}
				}

				//uchar* pDst = calibMat.data + colorIndex * 3;
				// write output
				//pDst[0] = pSrc->rgbBlue;
				//pDst[1] = pSrc->rgbGreen;
				//pDst[2] = pSrc->rgbRed;
				//outputRGBX[colorIndex] = *pSrc;
			}

			//m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(outputRGBX), COLOR_WIDTH * COLOR_HEIGHT * sizeof(RGBQUAD));
			//for (int colorIndex = 0; colorIndex < COLOR_WIDTH*COLOR_HEIGHT; colorIndex++)
			//{
			//calibMat.at<Vec3b>(colorIndex / COLOR_WIDTH, colorIndex / COLOR_HEIGHT)[0] = outputRGBX[colorIndex].rgbBlue;
			//calibMat.at<Vec3b>(colorIndex / COLOR_WIDTH, colorIndex / COLOR_HEIGHT)[1] = outputRGBX[colorIndex].rgbGreen;
			//calibMat.at<Vec3b>(colorIndex / COLOR_WIDTH, colorIndex / COLOR_HEIGHT)[2] = outputRGBX[colorIndex].rgbRed;
			//}
		}

		//imshow("Color", *(colorCV->GetColorMat()));
		//imshow("Depth", depthMat);
		//imshow("BodyIndex", *(bodyIndexCV->GetBodyIndexMat()));
		//imshow("Calib", calibMat);

		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}

	destroyAllWindows();

	delete colorCV;
	colorCV = NULL;

	delete depthCV;
	depthCV = NULL;

	delete bodyIndexCV;
	bodyIndexCV = NULL;
}
*/

HRESULT CCalibrationDlg::initKinectSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

		hr = m_pKinectSensor->Open();
	}

	return hr;
}

void CCalibrationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CCalibrationDlg, CDialogEx)
END_MESSAGE_MAP()


// CCalibrationDlg 메시지 처리기입니다.
