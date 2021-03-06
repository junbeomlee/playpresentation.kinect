#pragma once


// CCalibrationDlg 대화 상자입니다.

class CCalibrationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CCalibrationDlg)

public:
	CCalibrationDlg(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CCalibrationDlg();
	HRESULT initKinectSensor();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_CALIB_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()

private:
	IKinectSensor* m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;
	DepthSpacePoint*        m_pDepthCoordinates;
};
