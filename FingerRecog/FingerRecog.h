
// FingerRecog.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CFingerRecogApp:
// �� Ŭ������ ������ ���ؼ��� FingerRecog.cpp�� �����Ͻʽÿ�.
//

class CFingerRecogApp : public CWinApp
{
public:
	CFingerRecogApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CFingerRecogApp theApp;