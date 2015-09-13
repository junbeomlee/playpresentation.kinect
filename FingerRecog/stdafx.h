
// stdafx.h : 자주 사용하지만 자주 변경되지는 않는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이 
// 들어 있는 포함 파일입니다.

#pragma once

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 거의 사용되지 않는 내용은 Windows 헤더에서 제외합니다.
#endif

#ifdef _DEBUG
#pragma comment(linker, "/entry.WinMainCRTStartup /subsystem:console")
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 일부 CString 생성자는 명시적으로 선언됩니다.

// MFC의 공통 부분과 무시 가능한 경고 메시지에 대한 숨기기를 해제합니다.
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC 핵심 및 표준 구성 요소입니다.
#include <afxext.h>         // MFC 확장입니다.


#include <afxdisp.h>        // MFC 자동화 클래스입니다.

#include <Shlobj.h>

// Direct2D Header Files
#include <d2d1.h>


#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // Internet Explorer 4 공용 컨트롤에 대한 MFC 지원입니다.
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // Windows 공용 컨트롤에 대한 MFC 지원입니다.
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC의 리본 및 컨트롤 막대 지원

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


