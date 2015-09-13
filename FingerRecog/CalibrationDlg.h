#pragma once


// CCalibrationDlg ��ȭ �����Դϴ�.

class CCalibrationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CCalibrationDlg)

public:
	CCalibrationDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CCalibrationDlg();
	HRESULT initKinectSensor();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_CALIB_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()

private:
	IKinectSensor* m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;
	DepthSpacePoint*        m_pDepthCoordinates;
};
