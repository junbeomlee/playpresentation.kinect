#pragma once


// CHandDetectionDlg 대화 상자입니다.

class CHandDetectionDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CHandDetectionDlg)

public:
	CHandDetectionDlg(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CHandDetectionDlg();
	HRESULT initKinectSensor();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_HAND_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()

private:
	IKinectSensor* m_pKinectSensor;
};

