
// stdafx.h : ���� ��������� ���� ��������� �ʴ�
// ǥ�� �ý��� ���� ���� �� ������Ʈ ���� ���� ������ 
// ��� �ִ� ���� �����Դϴ�.

#pragma once

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // ���� ������ �ʴ� ������ Windows ������� �����մϴ�.
#endif

#ifdef _DEBUG
#pragma comment(linker, "/entry.WinMainCRTStartup /subsystem:console")
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // �Ϻ� CString �����ڴ� ��������� ����˴ϴ�.

// MFC�� ���� �κа� ���� ������ ��� �޽����� ���� ����⸦ �����մϴ�.
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC �ٽ� �� ǥ�� ���� ����Դϴ�.
#include <afxext.h>         // MFC Ȯ���Դϴ�.


#include <afxdisp.h>        // MFC �ڵ�ȭ Ŭ�����Դϴ�.

#include <Shlobj.h>

// Direct2D Header Files
#include <d2d1.h>


#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // Internet Explorer 4 ���� ��Ʈ�ѿ� ���� MFC �����Դϴ�.
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // Windows ���� ��Ʈ�ѿ� ���� MFC �����Դϴ�.
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC�� ���� �� ��Ʈ�� ���� ����

#include <Kinect.h>
#include <cv.h>
#include <highgui.h>
#include <opencv.hpp>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080
#define TIME_LIMIT 1.0
//#define PI 3.14159265
#define DIST_LIMIT 0.8
#define ARM_ANGLE_LIMIT 30
#define ARM_DIST_LIMIT 200
#define ARM_COUNT_LIMIT 0.0
#define ANGLE_DEV_LIMIT 3.0
#define WAIT_TIME 1.0
#define LENGTH_LIMIT 15
#define DISTANCE_LIMIT 70
#define DISTANCE_PER_PIXEL 0.00275
#define ANGLE_LOW_LIMIT 95.0
#define ANGLE_HIGH_LIMIT 130.0
#define RANGE_LIMIT 1300

using namespace cv;
using namespace std;

typedef struct _SphericalCoordinate {
	double radian;
	double theta;
	double phi;
} SphericalCoordinate;

typedef struct _ObjectData {
	char objectName[100];
	int index;
	SphericalCoordinate locate;
	Scalar color;
	int minArea;
	int maxArea;
} ObjectData;

typedef struct _PointingData {
	clock_t time;
	Point3d pointDirection;
	Point3d midArm;
	double angle;
	Point3d startPoint;
} PointingData;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

template<class Interface>
inline void CvSafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->release();
		delete pInterfaceToRelease;
		pInterfaceToRelease = NULL;
	}
}

Point PointInRange(Point point)
{
	Point p;
	if (point.x < 0)
	{
		p.x = 0;
	}

	if (point.x >= DEPTH_WIDTH)
	{
		p.x = DEPTH_WIDTH - 1;
	}

	if (point.y < 0)
	{
		p.y = 0;
	}

	if (point.y >= DEPTH_HEIGHT)
	{
		p.y = DEPTH_HEIGHT - 1;
	}

	return p;
}



#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif


