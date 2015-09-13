#pragma once


// CHandDetectionDlg ��ȭ �����Դϴ�.

class CHandDetectionDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CHandDetectionDlg)

public:
	CHandDetectionDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CHandDetectionDlg();
	HRESULT initKinectSensor();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_HAND_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()

private:
	IKinectSensor* m_pKinectSensor;
};

